import argparse
import json
import os
import queue
import socket
import struct
import sys
import threading
import time
import types
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


# Some local pyqtgraph installs import colorama even when it is only used for
# optional terminal coloring. Provide the symbols pyqtgraph imports so the GUI
# can run on machines where colorama is not installed.
if "colorama" not in sys.modules:
    colorama = types.ModuleType("colorama")
    colorama.__path__ = []

    colorama_win32 = types.ModuleType("colorama.win32")
    colorama_win32.windll = None

    colorama_winterm = types.ModuleType("colorama.winterm")
    colorama_winterm.WinColor = type("WinColor", (), {"BLACK": 0, "RED": 4, "GREEN": 2, "YELLOW": 6, "BLUE": 1, "MAGENTA": 5, "CYAN": 3, "WHITE": 7, "GREY": 8, "RESET": 7})
    colorama_winterm.WinStyle = type("WinStyle", (), {"NORMAL": 0, "BRIGHT": 8, "BRIGHT_BACKGROUND": 8, "RESET_ALL": 0})
    colorama_winterm.WinTerm = type("WinTerm", (), {})

    colorama_ansi = types.ModuleType("colorama.ansi")
    colorama_ansi.AnsiFore = type("AnsiFore", (), {"BLACK":"", "RED":"", "GREEN":"", "YELLOW":"", "BLUE":"", "MAGENTA":"", "CYAN":"", "WHITE":"", "RESET":""})
    colorama_ansi.AnsiBack = type("AnsiBack", (), {"BLACK":"", "RED":"", "GREEN":"", "YELLOW":"", "BLUE":"", "MAGENTA":"", "CYAN":"", "WHITE":"", "RESET":""})
    colorama_ansi.AnsiStyle = type("AnsiStyle", (), {"BRIGHT":"", "DIM":"", "NORMAL":"", "RESET_ALL":""})
    colorama_ansi.clear_screen = lambda *args, **kwargs: ""
    colorama_ansi.clear_line = lambda *args, **kwargs: ""
    colorama_ansi.set_title = lambda *args, **kwargs: ""
    colorama_ansi.Cursor = type("Cursor", (), {})

    colorama_ansitowin32 = types.ModuleType("colorama.ansitowin32")
    colorama_ansitowin32.AnsiToWin32 = type("AnsiToWin32", (), {"__init__": lambda self, *a, **k: None})

    colorama_initialise = types.ModuleType("colorama.initialise")
    colorama_initialise.init = lambda *args, **kwargs: None
    colorama_initialise.deinit = lambda *args, **kwargs: None
    colorama_initialise.reinit = lambda *args, **kwargs: None
    colorama_initialise.wrap_stream = lambda stream, *args, **kwargs: stream

    colorama.win32 = colorama_win32
    colorama.winterm = colorama_winterm
    colorama.ansi = colorama_ansi
    colorama.ansitowin32 = colorama_ansitowin32
    colorama.initialise = colorama_initialise
    colorama.init = colorama_initialise.init
    colorama.deinit = colorama_initialise.deinit
    colorama.reinit = colorama_initialise.reinit

    sys.modules["colorama"] = colorama
    sys.modules["colorama.win32"] = colorama_win32
    sys.modules["colorama.winterm"] = colorama_winterm
    sys.modules["colorama.ansi"] = colorama_ansi
    sys.modules["colorama.ansitowin32"] = colorama_ansitowin32
    sys.modules["colorama.initialise"] = colorama_initialise

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg


DEFAULT_BIND_IP = "0.0.0.0"
DEFAULT_DATA_PORT = 5000
DEFAULT_CONTROL_IP = "192.168.1.10"
DEFAULT_CONTROL_PORT = 5001
DEFAULT_FRAME_WORDS = 131072
DEFAULT_PRETRIGGER_WORDS = 16384
MIN_FRAME_WORDS = 1
MAX_DISPLAY_POINTS = 3000
UDP_BUFFER_SIZE = 65535
UDP_SOCKET_RCVBUF = 16 * 1024 * 1024
UDP_MAGIC = 0xADC64096
UDP_HEADER_SIZE = 20
SAVE_QUEUE_DEPTH = 512
SAVE_SHARD_FRAMES = 1000
STREAM_SEGMENT_BYTES = 1024 * 1024 * 1024
STREAM_SEGMENT_FRAMES = 1000

CTRL_CMD_MAGIC = 0x4354524C
CTRL_REPLY_MAGIC = 0x43545252
CTRL_OP_READ = 1
CTRL_OP_WRITE = 2
CTRL_OP_DUMP = 3
CTRL_STATUS_TEXT = {
    0: "OK",
    1: "BAD_OP",
    2: "BAD_REG",
    3: "BAD_LEN",
}

ADC_REGISTERS = {
    "control": 0,
    "status": 1,
    "frame_size": 2,
    "pretrigger_size": 3,
    "sample_decimation": 4,
    "trigger_cfg": 5,
    "self_threshold": 6,
    "channel_mask": 7,
    "output_cfg": 8,
    "baseline_shift": 9,
}

CONTROL_REPLY_STRUCT = struct.Struct("!IIHHIIIIIIIIII")
CONTROL_COMMAND_STRUCT = struct.Struct("!IIHHI")
FOOTER_MAGIC = 0x00000000FEE70001
PL_FOOTER_WORDS = 17
# Footer version 3 replaced the duplicated Baseline Shift byte in Word 15
# with a 32-bit trigger-accept timestamp in Word 15[63:32], keeping Config
# Sequence in Word 15[31:0]. Version 4 gives the whole of Word 15 to a
# 64-bit timestamp. Version 5 reports Config Sequence in Word 10[31:0] and
# keeps trigger-loss reason counters in Word 16.
# Baseline Shift is reported in Word 1 for every version.
FOOTER_VERSION_TIMESTAMP = 3
FOOTER_VERSION_TIMESTAMP64 = 4
FOOTER_VERSION_CONFIG_SEQ_WORD10 = 5
TIMESTAMP_TICK_NS = 8.0
TIMESTAMP_MODULUS_32 = 1 << 32
TIMESTAMP_MODULUS_64 = 1 << 64
TIMESTAMP_MODULUS = TIMESTAMP_MODULUS_32
FOOTER_SEARCH_WORDS = 128
OUTPUT_CFG_APPEND_FOOTER = 0x2
OUTPUT_CFG_PEAK_SIGNED_MAX = 0x8
OUTPUT_CFG_FOOTER_ONLY = 0x10
OUTPUT_CFG_RAW_EVERY_N = 0x40
OUTPUT_CFG_RAW_PERIOD_SHIFT = 8
OUTPUT_CFG_RAW_PERIOD_MASK = 0xFFFFFF
CONTROL_SOFTWARE_RESET = 1 << 4
CONTROL_SOFT_TRIGGER = 1 << 1
CONTROL_ACQUISITION_ENABLE = 1 << 5
SELF_TRIGGER_FALLING = 1 << 3
TRIGGER_EXTERNAL = 1 << 0
SOFT_TRIGGER_LOW_SETTLE_S = 0.002
SOFT_TRIGGER_HIGH_PULSE_S = 0.002
POST_APPLY_DISCARD_S = 1.0
FIFO_STATUS_ADC1_FULL = 1 << 0
FIFO_STATUS_ADC2_FULL = 1 << 1
FIFO_STATUS_FRAME_INVALID = (1 << 2) | (1 << 3)
FIFO_STATUS_TRIGGER_REJECTED = 1 << 4
PL_STATUS_ADC1_FIFO_FULL = 1 << 0
PL_STATUS_ADC2_FIFO_FULL = 1 << 1
PL_STATUS_ANY_FIFO_FULL = 1 << 2
PL_STATUS_CAPTURE_OVERRUN = 1 << 3


@dataclass
class Frame:
    frame_id: int
    raw_words: np.ndarray
    received_at: float
    packets: int
    bytes_received: int
    dropped_packets: int
    incomplete_frames: int


@dataclass
class ControlReply:
    seq: int
    status: int
    reg: int
    value: int
    control: int
    frame_size: int
    pretrigger_size: int
    sample_decimation: int
    trigger_cfg: int
    self_threshold: int
    channel_mask: int
    output_cfg: int
    baseline_shift: int


def decode_channels(raw_words: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    words = raw_words.astype(np.uint64, copy=False)
    ch1 = ((words >> 12) & 0x0FFF).astype(np.uint16)
    ch2 = ((words >> 0) & 0x0FFF).astype(np.uint16)
    ch3 = ((words >> 44) & 0x0FFF).astype(np.uint16)
    ch4 = ((words >> 32) & 0x0FFF).astype(np.uint16)
    return ch1, ch2, ch3, ch4


def decode_corrected_channels(raw_words: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    words = raw_words.astype(np.uint64, copy=False)
    ch1 = ((words >> 0) & 0xFFFF).astype(np.uint16).view(np.int16).copy()
    ch2 = ((words >> 16) & 0xFFFF).astype(np.uint16).view(np.int16).copy()
    ch3 = ((words >> 32) & 0xFFFF).astype(np.uint16).view(np.int16).copy()
    ch4 = ((words >> 48) & 0xFFFF).astype(np.uint16).view(np.int16).copy()
    return ch1, ch2, ch3, ch4


def codes_to_mv(codes: np.ndarray, gain: float, offset_mv: float, fullscale_mv: float = 2000.0) -> np.ndarray:
    centered = codes.astype(np.float32) - 2048.0
    return (centered / 2048.0) * (fullscale_mv * 0.5) * gain + offset_mv


def corrected_codes_to_mv(codes: np.ndarray, gain: float, fullscale_mv: float = 2000.0) -> np.ndarray:
    return (codes.astype(np.float32) / 2048.0) * (fullscale_mv * 0.5) * gain


def adc_code_to_mv(code: int, gain: float, offset_mv: float, fullscale_mv: float = 2000.0) -> float:
    centered = float(int(code) - 2048)
    return (centered / 2048.0) * (float(fullscale_mv) * 0.5) * float(gain) + float(offset_mv)


def mv_to_adc_code(mv: float, gain: float, offset_mv: float, fullscale_mv: float = 2000.0) -> int:
    scale = float(fullscale_mv) * 0.5 * float(gain)
    if abs(scale) < 1.0e-12:
        return 2048
    code = int(round(((float(mv) - float(offset_mv)) / scale) * 2048.0 + 2048.0))
    return max(0, min(4095, code))


def downsample_for_plot(y: np.ndarray, max_points: int = MAX_DISPLAY_POINTS) -> tuple[np.ndarray, np.ndarray]:
    if y.size <= max_points:
        return np.arange(y.size, dtype=np.int32), y

    # Preserve time order for display. A min/max envelope with duplicate x
    # positions can make valid ADC data look broken or vertically smeared.
    idx = np.linspace(0, y.size - 1, max_points, dtype=np.int64)
    return idx.astype(np.int32), y[idx]


def send_control_command(
    host: str,
    port: int,
    op: int,
    reg: int = 0,
    value: int = 0,
    timeout_s: float = 1.0,
) -> ControlReply:
    seq = int(time.time_ns() & 0xFFFFFFFF)
    packet = CONTROL_COMMAND_STRUCT.pack(
        CTRL_CMD_MAGIC,
        seq,
        int(op) & 0xFFFF,
        int(reg) & 0xFFFF,
        int(value) & 0xFFFFFFFF,
    )

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout_s)
        sock.sendto(packet, (host, int(port)))
        data, _sender = sock.recvfrom(UDP_BUFFER_SIZE)

    if len(data) < CONTROL_REPLY_STRUCT.size:
        raise RuntimeError(f"short control reply: {len(data)} bytes")

    (
        magic,
        reply_seq,
        status,
        reply_reg,
        reply_value,
        control,
        frame_size,
        pretrigger_size,
        sample_decimation,
        trigger_cfg,
        self_threshold,
        channel_mask,
        output_cfg,
        baseline_shift,
    ) = CONTROL_REPLY_STRUCT.unpack_from(data, 0)

    if magic != CTRL_REPLY_MAGIC:
        raise RuntimeError(f"bad control reply magic: 0x{magic:08X}")
    if reply_seq != seq:
        raise RuntimeError(f"control reply sequence mismatch: {reply_seq} != {seq}")

    return ControlReply(
        seq=reply_seq,
        status=status,
        reg=reply_reg,
        value=reply_value,
        control=control,
        frame_size=frame_size,
        pretrigger_size=pretrigger_size,
        sample_decimation=sample_decimation,
        trigger_cfg=trigger_cfg,
        self_threshold=self_threshold,
        channel_mask=channel_mask,
        output_cfg=output_cfg,
        baseline_shift=baseline_shift,
    )


def find_footer_start(raw_words: np.ndarray) -> int | None:
    if raw_words.size < PL_FOOTER_WORDS:
        return None

    exact_index = raw_words.size - PL_FOOTER_WORDS
    if int(raw_words[exact_index]) == FOOTER_MAGIC:
        return exact_index

    search_start = max(0, raw_words.size - FOOTER_SEARCH_WORDS)
    for idx in range(raw_words.size - PL_FOOTER_WORDS, search_start - 1, -1):
        if int(raw_words[idx]) == FOOTER_MAGIC:
            return idx
    return None


def split_waveform_footer(raw_words: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
    footer_start = find_footer_start(raw_words)
    if footer_start is not None:
        return raw_words[:footer_start], raw_words[footer_start : footer_start + PL_FOOTER_WORDS].copy()
    return raw_words, None


def footer_output_cfg(footer_words: np.ndarray | None) -> int | None:
    if footer_words is None or footer_words.size < 2:
        return None

    info = int(footer_words[1])
    return (info >> 32) & 0xFF


def footer_capture_config(footer_words: np.ndarray | None) -> dict[str, int | None] | None:
    if footer_words is None or footer_words.size < 16:
        return None

    words = [int(word) for word in footer_words[:PL_FOOTER_WORDS]]
    info = words[1]
    version = (info >> 48) & 0xFFFF
    loss_word = words[16] if len(words) >= 17 else 0
    return {
        "footer_version": version,
        "frame_size": words[11] >> 32,
        "pretrigger_size": words[11] & 0xFFFFFFFF,
        "post_samples": words[12] >> 32,
        "sample_decimation": words[12] & 0xFFFFFFFF,
        "trigger_cfg": words[13] >> 32,
        "self_threshold": words[13] & 0xFFFFFFFF,
        "channel_mask": words[14] >> 32,
        "output_cfg": words[14] & 0xFFFFFFFF,
        # Word 1 is the authoritative location for Baseline Shift.
        "baseline_shift": (info >> 16) & 0xFF,
        # Footer v5 reports the config sequence in word 10[31:0].
        # Version 3 used word 15[31:0]; version 4 has a full 64-bit
        # timestamp in word 15 and no config sequence.
        "config_seq": (
            (words[10] & 0xFFFFFFFF)
            if version >= FOOTER_VERSION_CONFIG_SEQ_WORD10
            else (
                (words[15] & 0xFFFFFFFF)
                if version >= FOOTER_VERSION_TIMESTAMP
                else None
            )
        ),
        # Free-running 125 MHz counter latched when the trigger was
        # accepted. None on footers older than version 3.
        "trig_time": parse_trig_time(version, words[15]),
        "timestamp_bits": (
            64 if version >= FOOTER_VERSION_TIMESTAMP64 else 32
        ),
        "loss_ignored": (loss_word >> 48) & 0xFFFF,
        "loss_busy": (loss_word >> 32) & 0xFFFF,
        "loss_stopped": (loss_word >> 16) & 0xFFFF,
        "loss_fifo": loss_word & 0xFFFF,
    }


def timestamp_modulus_for_version(version: int) -> int:
    """Wrap modulus of the PL trigger timestamp for a footer version."""
    if int(version) >= FOOTER_VERSION_TIMESTAMP64:
        return TIMESTAMP_MODULUS_64
    return TIMESTAMP_MODULUS_32


def parse_trig_time(version: int, word15: int) -> int | None:
    """Extract the trigger-accept timestamp from footer Word 15.

    Version 4 uses all 64 bits. Version 3 keeps the timestamp in the upper
    half and Config Sequence in the lower half of Word 15. Older footers
    have no timestamp at all.
    """
    version = int(version)
    word15 = int(word15)
    if version >= FOOTER_VERSION_TIMESTAMP64:
        return word15 & 0xFFFFFFFFFFFFFFFF
    if version >= FOOTER_VERSION_TIMESTAMP:
        return (word15 >> 32) & 0xFFFFFFFF
    return None


def timestamp_delta_ticks(
    current: int, previous: int, modulus: int = TIMESTAMP_MODULUS_32
) -> int:
    """Ticks elapsed between two trigger timestamps.

    The 32-bit counter of footer version 3 wraps about every 34.36 s, so
    modular subtraction is used. The 64-bit counter of version 4 never wraps
    in practice, and the same modular subtraction is still correct there.
    """
    return (int(current) - int(previous)) % int(modulus)


def format_timestamp_ticks(ticks: int) -> str:
    seconds = ticks * TIMESTAMP_TICK_NS * 1.0e-9
    if seconds >= 1.0:
        return f"{seconds:.6f} s"
    if seconds >= 1.0e-3:
        return f"{seconds * 1.0e3:.3f} ms"
    if seconds >= 1.0e-6:
        return f"{seconds * 1.0e6:.3f} us"
    return f"{ticks * TIMESTAMP_TICK_NS:.0f} ns"


def output_mask_from_channel_mask(channel_mask: int) -> int:
    channel_mask = int(channel_mask) & 0xFF
    output_mask = (channel_mask >> 4) & 0xF
    if output_mask == 0:
        output_mask = channel_mask & 0xF
    return output_mask & 0xF


def payload_to_words(payload: bytes, target_bytes: int) -> np.ndarray:
    raw = np.frombuffer(payload[:target_bytes], dtype="<u8").copy()
    if find_footer_start(raw) is None:
        swapped = raw.byteswap()
        if find_footer_start(swapped) is not None:
            return swapped
    return raw


class FrameAssembler:
    """Assemble frames from the Vitis UDP header or raw UDP chunks.

    Vitis header, network byte order:
        magic, frame_id, packet_index, total_packets,
        payload_bytes, reserved, frame_bytes
    """

    def __init__(self, frame_words: int, footer_words: int = 0):
        self.frame_words = frame_words
        self.footer_words = max(0, int(footer_words))
        self.frame_bytes = (frame_words + self.footer_words) * 8
        self.raw_buffer = bytearray()
        self.raw_frame_id = 0
        self.frames: dict[int, dict] = {}
        self.total_dropped = 0
        self.incomplete_frames = 0
        self.header_mode_seen = False
        self.latest_frame_id: int | None = None

    def add_packet(self, data: bytes) -> Frame | None:
        parsed = self._parse_header(data)
        if parsed is None:
            if self.header_mode_seen:
                # Once framed UDP is detected, never mix malformed/headerless
                # datagrams into the ADC payload. Drop this datagram and let
                # the next trigger/frame proceed cleanly.
                self.total_dropped += 1
                return None
            return self._add_raw_packet(data)

        self.header_mode_seen = True
        frame_id, packet_id, packet_count, payload, source_frame_bytes = parsed

        if self.latest_frame_id is not None and frame_id < self.latest_frame_id:
            self.total_dropped += 1
            return None

        if self.latest_frame_id is None or frame_id > self.latest_frame_id:
            self._drop_older_incomplete_frames(frame_id)
            self.latest_frame_id = frame_id

        entry = self.frames.setdefault(
            frame_id,
            {
                "created_at": time.time(),
                "packet_count": packet_count,
                "source_frame_bytes": source_frame_bytes,
                "payloads": {},
                "bytes": 0,
            },
        )

        if packet_count != entry["packet_count"]:
            self._drop_frame(frame_id)
            return None

        if packet_id not in entry["payloads"]:
            entry["payloads"][packet_id] = payload
            entry["bytes"] += len(payload)

        if len(entry["payloads"]) < packet_count:
            return None

        chunks = [entry["payloads"].get(i, b"") for i in range(packet_count)]
        del self.frames[frame_id]
        payload_all = b"".join(chunks)
        target_bytes = int(entry.get("source_frame_bytes", self.frame_bytes))
        if target_bytes <= 0:
            target_bytes = self.frame_bytes
        return self._make_frame(
            frame_id=frame_id,
            payload=payload_all[:target_bytes],
            packets=packet_count,
            bytes_received=len(payload_all),
            dropped_packets=self.total_dropped,
            target_bytes=target_bytes,
        )

    def _drop_frame(self, frame_id: int) -> None:
        entry = self.frames.pop(frame_id, None)
        if entry is None:
            return
        self.incomplete_frames += 1
        self.total_dropped += max(0, int(entry["packet_count"]) - len(entry["payloads"]))

    def _drop_older_incomplete_frames(self, current_frame_id: int) -> None:
        old_ids = [fid for fid in self.frames if fid < current_frame_id]
        for old_id in old_ids:
            self._drop_frame(old_id)

    def _parse_header(self, data: bytes):
        if len(data) < UDP_HEADER_SIZE:
            return None
        magic, frame_id, packet_id, packet_count, payload_len, _reserved, frame_bytes = struct.unpack_from("!IIHHHHI", data, 0)
        if magic != UDP_MAGIC:
            return None
        if packet_count == 0 or packet_count > 4096:
            return None
        if packet_id >= packet_count:
            return None
        if payload_len == 0 or payload_len > len(data) - UDP_HEADER_SIZE:
            return None
        if packet_count < 2 and payload_len < frame_bytes:
            return None
        payload = data[UDP_HEADER_SIZE : UDP_HEADER_SIZE + payload_len]
        return frame_id, packet_id, packet_count, payload, frame_bytes

    def _add_raw_packet(self, data: bytes) -> Frame | None:
        self.raw_buffer.extend(data)
        if len(self.raw_buffer) < self.frame_bytes:
            return None

        payload = bytes(self.raw_buffer[: self.frame_bytes])
        del self.raw_buffer[: self.frame_bytes]
        self.raw_frame_id += 1
        return self._make_frame(
            frame_id=self.raw_frame_id,
            payload=payload,
            packets=0,
            bytes_received=len(payload),
            dropped_packets=self.total_dropped,
            target_bytes=self.frame_bytes,
        )

    def _make_frame(
        self,
        frame_id: int,
        payload: bytes,
        packets: int,
        bytes_received: int,
        dropped_packets: int,
        target_bytes: int,
    ) -> Frame | None:
        if len(payload) < target_bytes:
            return None
        raw = payload_to_words(payload, target_bytes)
        return Frame(frame_id, raw, time.time(), packets, bytes_received, dropped_packets, self.incomplete_frames)


class UdpReceiver(QtCore.QObject):
    frame_ready = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(str)

    def __init__(self, bind_ip: str, port: int, frame_words: int, footer_words: int = 0):
        super().__init__()
        self.bind_ip = bind_ip
        self.port = port
        self.frame_words = frame_words
        self.footer_words = max(0, int(footer_words))
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        assembler = FrameAssembler(self.frame_words, self.footer_words)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_SOCKET_RCVBUF)
                sock.bind((self.bind_ip, self.port))
                sock.settimeout(0.2)
                self.status.emit(f"Listening on {self.bind_ip}:{self.port}")
                while not self._stop.is_set():
                    try:
                        data, _sender = sock.recvfrom(UDP_BUFFER_SIZE)
                    except socket.timeout:
                        continue
                    frame = assembler.add_packet(data)
                    if frame is not None:
                        self.frame_ready.emit(frame)
        except OSError as exc:
            self.status.emit(f"UDP receiver error: {exc}")
        finally:
            self.status.emit("UDP receiver stopped")


class FrameSaver(QtCore.QObject):
    status = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.enabled = True
        self.output_dir = Path.cwd() / "captures"
        self.save_format = "raw-bin"
        self.queue: queue.Queue[object] = queue.Queue(maxsize=SAVE_QUEUE_DEPTH)
        self.queued_count = 0
        self.saved_count = 0
        self.dropped_count = 0
        self._stream_file = None
        self._stream_index = None
        self._stream_dir: Path | None = None
        self._stream_path: Path | None = None
        self._stream_segment = 0
        self._stream_offset = 0
        self._stream_frames = 0
        self._stream_first_frame_id: int | None = None
        self._stream_last_frame_id: int | None = None
        self._stream_first_received_at: float | None = None
        self._stream_last_received_at: float | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def save_later(self, frame: Frame, settings: dict, channels: tuple[np.ndarray, ...]) -> bool:
        if not self.enabled:
            return False
        try:
            self.queue.put_nowait((frame, settings, channels))
            self.queued_count += 1
            if self.queued_count <= 3 or (self.queued_count % 100) == 0:
                self.status.emit(
                    f"Queued save frame {frame.frame_id} "
                    f"({self.queue.qsize()} pending) -> {self.output_dir}"
                )
            return True
        except queue.Full:
            self.dropped_count += 1
            if self.dropped_count <= 3 or (self.dropped_count % 100) == 0:
                self.status.emit(
                    f"Save queue full; skipped {self.dropped_count} frames "
                    f"({self.queue.qsize()} pending)"
                )
            return False

    def finalize_segment(self) -> None:
        try:
            self.queue.put_nowait(("flush", None))
        except queue.Full:
            self.status.emit("Save queue full; stream finalize deferred")

    def stop(self, timeout_s: float = 5.0) -> None:
        self.finalize_segment()
        self._stop.set()
        self._thread.join(timeout=max(0.0, float(timeout_s)))

    def _frame_dir(self, frame_id: int) -> Path:
        return self.output_dir / f"{frame_id // SAVE_SHARD_FRAMES:06d}"

    def _close_stream(self) -> None:
        if self._stream_path is not None and self._stream_frames > 0 and self._stream_dir is not None:
            manifest_path = self._stream_dir / "stream_manifest.jsonl"
            record = {
                "segment": str(self._stream_path.name),
                "frames": int(self._stream_frames),
                "bytes": int(self._stream_offset),
                "first_frame_id": self._stream_first_frame_id,
                "last_frame_id": self._stream_last_frame_id,
                "first_received_at": self._stream_first_received_at,
                "last_received_at": self._stream_last_received_at,
                "first_received_iso": (
                    datetime.fromtimestamp(self._stream_first_received_at).isoformat(timespec="milliseconds")
                    if self._stream_first_received_at is not None else
                    None
                ),
                "last_received_iso": (
                    datetime.fromtimestamp(self._stream_last_received_at).isoformat(timespec="milliseconds")
                    if self._stream_last_received_at is not None else
                    None
                ),
            }
            with open(manifest_path, "a", encoding="utf-8", buffering=1) as manifest:
                manifest.write(json.dumps(record, separators=(",", ":")) + "\n")
        if self._stream_index is not None:
            self._stream_index.close()
            self._stream_index = None
        if self._stream_file is not None:
            self._stream_file.close()
            self._stream_file = None
        self._stream_path = None
        self._stream_offset = 0
        self._stream_frames = 0
        self._stream_first_frame_id = None
        self._stream_last_frame_id = None
        self._stream_first_received_at = None
        self._stream_last_received_at = None

    def _ensure_stream(self, frame: Frame, next_bytes: int) -> Path:
        stream_dir = self.output_dir / "stream"
        needs_new = (
            self._stream_file is None or
            self._stream_index is None or
            self._stream_dir != stream_dir or
            self._stream_offset + next_bytes > STREAM_SEGMENT_BYTES or
            self._stream_frames >= STREAM_SEGMENT_FRAMES
        )
        if not needs_new:
            return self._stream_path if self._stream_path is not None else stream_dir

        self._close_stream()
        stream_dir.mkdir(parents=True, exist_ok=True)
        self._stream_dir = stream_dir
        stamp = datetime.fromtimestamp(frame.received_at).strftime("%Y%m%d_%H%M%S")
        base = f"frames_{stamp}_{self._stream_segment:04d}"
        self._stream_segment += 1
        self._stream_path = stream_dir / f"{base}.bin"
        index_path = stream_dir / f"{base}.jsonl"
        self._stream_file = open(self._stream_path, "ab", buffering=1024 * 1024)
        self._stream_index = open(index_path, "a", encoding="utf-8", buffering=1)
        return self._stream_path

    def _save_stream_frame(self, frame: Frame, settings: dict) -> Path:
        words = frame.raw_words.astype("<u8", copy=False)
        path = self._ensure_stream(frame, int(words.nbytes))
        offset = self._stream_offset
        words.tofile(self._stream_file)
        record = {
            "frame_id": int(frame.frame_id),
            "received_at": float(frame.received_at),
            "received_iso": datetime.fromtimestamp(frame.received_at).isoformat(timespec="milliseconds"),
            "segment": str(path.name),
            "offset": int(offset),
            "segment_frame_index": int(self._stream_frames),
            "words": int(words.size),
            "bytes": int(words.nbytes),
            "packets": int(frame.packets),
            "bytes_received": int(frame.bytes_received),
            "dropped_packets": int(frame.dropped_packets),
            "incomplete_frames": int(frame.incomplete_frames),
            "output_cfg": int(settings["output_cfg"]),
            "baseline_shift": int(settings["baseline_shift"]),
        }
        self._stream_index.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._stream_index.flush()
        if self._stream_first_frame_id is None:
            self._stream_first_frame_id = int(frame.frame_id)
            self._stream_first_received_at = float(frame.received_at)
        self._stream_last_frame_id = int(frame.frame_id)
        self._stream_last_received_at = float(frame.received_at)
        self._stream_offset += int(words.nbytes)
        self._stream_frames += 1
        if (self.saved_count % 256) == 0:
            self._stream_file.flush()
            self._stream_index.flush()
        return path

    def _run(self) -> None:
        try:
            while not self._stop.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.2)
                except queue.Empty:
                    continue

                try:
                    if isinstance(item, tuple) and len(item) == 2 and item[0] == "flush":
                        self._close_stream()
                        self.status.emit("Stream segment finalized")
                        continue

                    frame, settings, channels = item
                    self.output_dir.mkdir(parents=True, exist_ok=True)
                    stamp = datetime.fromtimestamp(frame.received_at).strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    if self.save_format == "raw-stream":
                        path = self._save_stream_frame(frame, settings)
                    elif self.save_format == "raw-bin":
                        frame_dir = self._frame_dir(frame.frame_id)
                        frame_dir.mkdir(parents=True, exist_ok=True)
                        path = frame_dir / f"frame_{frame.frame_id:08d}_{stamp}.bin"
                        frame.raw_words.astype("<u8", copy=False).tofile(path)
                    else:
                        frame_dir = self._frame_dir(frame.frame_id)
                        frame_dir.mkdir(parents=True, exist_ok=True)
                        ch1, ch2, ch3, ch4 = channels
                        path = frame_dir / f"frame_{frame.frame_id:08d}_{stamp}.npz"
                        save_fn = (
                            np.savez_compressed
                            if self.save_format == "npz-compressed" else
                            np.savez
                        )
                        save_fn(
                            path,
                            frame_id=np.uint32(frame.frame_id),
                            received_at=np.float64(frame.received_at),
                            raw_words=frame.raw_words,
                            ch1=ch1,
                            ch2=ch2,
                            ch3=ch3,
                            ch4=ch4,
                            mode=np.int32(settings["mode"]),
                            gain=np.float32(settings["gain"]),
                            offset_mv=np.float32(settings["offset_mv"]),
                            fullscale_mv=np.float32(settings["fullscale_mv"]),
                            output_cfg=np.uint32(settings["output_cfg"]),
                            baseline_shift=np.uint32(settings["baseline_shift"]),
                        )
                    self.saved_count += 1
                    if self.saved_count <= 3 or (self.saved_count % 100) == 0:
                        self.status.emit(
                            f"Saved {self.saved_count} frames "
                            f"({self.queue.qsize()} pending) -> {path}"
                        )
                except Exception as exc:
                    self.status.emit(f"Save error: {exc}")
                finally:
                    self.queue.task_done()
        finally:
            self._close_stream()


class MainWindow(QtWidgets.QMainWindow):
    _updating_self_mask = False

    @staticmethod
    def _make_combo(items: list[tuple[str, int]], current_value: int) -> QtWidgets.QComboBox:
        combo = QtWidgets.QComboBox()
        for label, value in items:
            combo.addItem(label, int(value))
        MainWindow._set_combo_value(combo, current_value)
        return combo

    @staticmethod
    def _combo_value(combo: QtWidgets.QComboBox) -> int:
        return int(combo.currentData())

    @staticmethod
    def _set_combo_value(combo: QtWidgets.QComboBox, value) -> None:
        index = combo.findData(value)
        if index < 0:
            try:
                numeric_value = int(value)
            except (TypeError, ValueError):
                numeric_value = None
            if numeric_value is not None:
                combo.addItem(f"Custom 0x{numeric_value:X}", numeric_value)
                index = combo.findData(numeric_value)
        if index < 0:
            index = 0
        combo.setCurrentIndex(index)

    def _channel_mask_value(self) -> int:
        self_mask = 0
        output_mask = 0

        for bit, check in enumerate(self.self_mask_checks):
            if check.isChecked():
                self_mask |= (1 << bit)

        if self_mask != 0:
            self_mask = self_mask & (~self_mask + 1)

        for bit, check in enumerate(self.output_mask_checks):
            if check.isChecked():
                output_mask |= (1 << bit)

        return ((output_mask & 0xF) << 4) | (self_mask & 0xF)

    def _set_channel_mask_value(self, value: int) -> None:
        value = int(value) & 0xFF
        self_mask = value & 0xF
        output_mask = (value >> 4) & 0xF

        if output_mask == 0:
            output_mask = self_mask

        if self_mask != 0:
            self_mask = self_mask & (~self_mask + 1)

        for bit, check in enumerate(self.self_mask_checks):
            check.setChecked((self_mask & (1 << bit)) != 0)

        for bit, check in enumerate(self.output_mask_checks):
            check.setChecked((output_mask & (1 << bit)) != 0)

    def _handle_self_mask_changed(self, changed_index: int, state: int) -> None:
        if self._updating_self_mask or state != QtCore.Qt.Checked:
            return

        self._updating_self_mask = True
        try:
            for bit, check in enumerate(self.self_mask_checks):
                if bit != changed_index:
                    check.setChecked(False)
        finally:
            self._updating_self_mask = False

    def _output_cfg_value(self) -> int:
        value = 0

        for bit, check in enumerate(self.output_cfg_checks):
            if check.isChecked():
                value |= (1 << bit)

        if self.raw_every_n_enable.isChecked():
            period = int(self.raw_every_n_interval.value())
            value |= OUTPUT_CFG_RAW_EVERY_N
            value |= (period & OUTPUT_CFG_RAW_PERIOD_MASK) << OUTPUT_CFG_RAW_PERIOD_SHIFT

        return value | OUTPUT_CFG_APPEND_FOOTER

    def _set_output_cfg_value(self, value: int) -> None:
        value = (int(value) & 0xFFFFFF5F) | OUTPUT_CFG_APPEND_FOOTER

        for bit, check in enumerate(self.output_cfg_checks):
            check.setChecked((value & (1 << bit)) != 0)

        self.raw_every_n_enable.setChecked((value & OUTPUT_CFG_RAW_EVERY_N) != 0)
        period = (value >> OUTPUT_CFG_RAW_PERIOD_SHIFT) & OUTPUT_CFG_RAW_PERIOD_MASK
        self.raw_every_n_interval.setValue(max(1, period))

    def _trigger_cfg_value(self) -> int:
        value = self._combo_value(self.trigger_cfg) & 0x7
        if self.self_falling_radio.isChecked():
            value |= SELF_TRIGGER_FALLING
        return value

    def _set_trigger_cfg_value(self, value: int) -> None:
        value = int(value) & 0xF
        self._set_combo_value(self.trigger_cfg, value & 0x7)
        falling = (value & SELF_TRIGGER_FALLING) != 0
        self.self_falling_radio.setChecked(falling)
        self.self_rising_radio.setChecked(not falling)

    def _apply_compact_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget#centralSurface {
                background: #eaf1f8;
                color: #18212b;
                font-size: 10pt;
            }
            QLabel, QCheckBox, QGroupBox {
                color: #18212b;
            }
            QWidget#topBar {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #e0f2fe, stop:0.38 #eef2ff, stop:0.72 #f0fdf4, stop:1 #fff7ed);
                border: 1px solid #b8c7d9;
                border-radius: 6px;
            }
            QPushButton {
                background: #ffffff;
                color: #172033;
                border: 1px solid #b9c2cc;
                border-radius: 4px;
                padding: 4px 10px;
                min-height: 22px;
            }
            QPushButton:hover {
                background: #f1f5f9;
                border-color: #8fa0b2;
            }
            QPushButton#startButton {
                background: #dcfce7;
                border-color: #86efac;
                color: #14532d;
                font-weight: 600;
            }
            QPushButton#stopButton {
                background: #fee2e2;
                border-color: #fca5a5;
                color: #7f1d1d;
                font-weight: 600;
            }
            QPushButton#resetButton {
                background: #ffedd5;
                border-color: #fdba74;
                color: #7c2d12;
                font-weight: 600;
            }
            QPushButton#softButton {
                background: #dbeafe;
                border-color: #93c5fd;
                color: #1e3a8a;
                font-weight: 600;
            }
            QPushButton#settingsButton {
                background: #ede9fe;
                border-color: #c4b5fd;
                color: #4c1d95;
                font-weight: 600;
            }
            QPushButton#browseButton {
                background: #fef9c3;
                border-color: #fde047;
                color: #713f12;
            }
            QPushButton:pressed {
                background: #e2e8f0;
            }
            QPushButton:disabled {
                color: #9aa3ad;
                background: #eef1f4;
            }
            QPushButton#startButton[active="true"] {
                background: #16a34a;
                border: 2px solid #166534;
                color: #ffffff;
            }
            QPushButton#startButton[active="true"]:disabled {
                background: #16a34a;
                border: 2px solid #166534;
                color: #ffffff;
            }
            QPushButton#stopButton[active="true"] {
                background: #dc2626;
                border: 2px solid #991b1b;
                color: #ffffff;
            }
            QPushButton#resetButton[active="true"] {
                background: #f97316;
                border: 2px solid #9a3412;
                color: #ffffff;
            }
            QPushButton#resetButton[active="true"]:disabled {
                background: #f97316;
                border: 2px solid #9a3412;
                color: #ffffff;
            }
            QLabel#statusBadge {
                border-radius: 6px;
                padding: 5px 12px;
                min-width: 116px;
                font-weight: 700;
                qproperty-alignment: AlignCenter;
            }
            QLabel#statusBadge[status="idle"] {
                background: #e2e8f0;
                border: 1px solid #94a3b8;
                color: #334155;
            }
            QLabel#statusBadge[status="listening"] {
                background: #bbf7d0;
                border: 1px solid #22c55e;
                color: #14532d;
            }
            QLabel#statusBadge[status="stopped"] {
                background: #fecaca;
                border: 1px solid #ef4444;
                color: #7f1d1d;
            }
            QLabel#statusBadge[status="resetting"] {
                background: #fed7aa;
                border: 1px solid #f97316;
                color: #7c2d12;
            }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                background: #ffffff;
                color: #172033;
                border: 1px solid #b9c2cc;
                border-radius: 4px;
                padding: 3px 6px;
                min-height: 22px;
            }
            QCheckBox {
                spacing: 6px;
            }
            QGroupBox#frameFooterBox {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #f8fafc, stop:0.5 #f5f3ff, stop:1 #ecfeff);
                border: 1px solid #bac8dc;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 8px;
                font-weight: 600;
            }
            QGroupBox#frameFooterBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #475569;
            }
            QDialog QGroupBox {
                background: #fbfdff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: 600;
            }
            QDialog QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #334155;
            }
            QTableWidget#statsTable, QTableWidget#footerTable {
                background: #fbfdff;
                color: #172033;
                border: 1px solid #d6dce3;
                border-radius: 4px;
                gridline-color: #e2e8f0;
                selection-background-color: #dbeafe;
            }
            QTableWidget#statsTable::item, QTableWidget#footerTable::item {
                padding-left: 4px;
                padding-right: 4px;
            }
            QHeaderView::section {
                background: #e0f2fe;
                border: 0;
                border-right: 1px solid #d6dce3;
                border-bottom: 1px solid #d6dce3;
                padding: 3px 4px;
                font-weight: 600;
            }
            QLabel#metaName {
                color: #64748b;
                font-size: 9pt;
            }
            QLabel#metaValue {
                color: #0f172a;
                font-size: 9pt;
            }
            QPlainTextEdit#logBox {
                background: #fbfdff;
                border: 1px solid #bac8dc;
                border-radius: 5px;
                padding: 4px;
                color: #243041;
                font-family: Consolas, monospace;
                font-size: 9pt;
            }
            """
        )

    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("Digitizer UDP Receiver")
        self.resize(1500, 950)

        pg.setConfigOptions(antialias=False, background="#0b0f14", foreground="#d6dde6")

        self.receiver: UdpReceiver | None = None
        self._last_decode_warning = ""
        self._last_trig_time: int | None = None
        self.last_frame_received_at: float | None = None
        self.frame_rate_times: deque[float] = deque()
        self.frame_rate_window_s = 5.0
        self._last_save_disabled_log_at = 0.0
        self.shots_received = 0
        self.active_shot_limit = 0
        self.shot_limit_uses_save_queue = False
        self.active_plot_enabled = True
        self._shot_limit_stop_pending = False
        self._updating_self_threshold = False
        self.ignore_received_frames = False
        self.post_apply_drop_frames = 0
        self.discard_frames_until_s = 0.0
        self.post_apply_discard_count = 0
        self.saver = FrameSaver()
        self.saver.output_dir = Path(args.output_dir)
        self.saver.status.connect(self.log)

        central = QtWidgets.QWidget()
        central.setObjectName("centralSurface")
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(8)
        self.setCentralWidget(central)
        self._apply_compact_style()

        self.settings_dialog = QtWidgets.QDialog(self)
        self.settings_dialog.setWindowTitle("Receiver Settings")
        self.settings_dialog.resize(1180, 680)
        settings_root = QtWidgets.QVBoxLayout(self.settings_dialog)
        settings_root.setContentsMargins(12, 12, 12, 12)
        settings_root.setSpacing(10)
        settings_content = QtWidgets.QWidget()
        controls = QtWidgets.QGridLayout(settings_content)
        controls.setHorizontalSpacing(12)
        controls.setVerticalSpacing(10)
        settings_scroll = QtWidgets.QScrollArea()
        settings_scroll.setWidgetResizable(True)
        settings_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        settings_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        settings_scroll.setWidget(settings_content)
        settings_root.addWidget(settings_scroll, 1)
        settings_bottom = QtWidgets.QHBoxLayout()
        settings_bottom.addStretch(1)
        self.close_settings_button = QtWidgets.QPushButton("Close")
        settings_bottom.addWidget(self.close_settings_button)
        settings_root.addLayout(settings_bottom)

        top_bar = QtWidgets.QWidget()
        top_bar.setObjectName("topBar")
        top_bar_layout = QtWidgets.QVBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(8, 6, 8, 6)
        top_bar_layout.setSpacing(5)
        command_controls = QtWidgets.QHBoxLayout()
        command_controls.setSpacing(8)
        save_controls = QtWidgets.QHBoxLayout()
        save_controls.setSpacing(8)
        top_bar_layout.addLayout(command_controls)
        top_bar_layout.addLayout(save_controls)
        root.addWidget(top_bar)

        self.bind_ip = QtWidgets.QLineEdit(args.bind_ip)
        self.data_port = QtWidgets.QSpinBox()
        self.data_port.setRange(1, 65535)
        self.data_port.setValue(args.data_port)
        self.control_ip = QtWidgets.QLineEdit(args.control_ip)
        self.control_port = QtWidgets.QSpinBox()
        self.control_port.setRange(1, 65535)
        self.control_port.setValue(args.control_port)

        display_mode = 1 if int(args.mode) == 2 else int(args.mode)
        self.mode = self._make_combo(
            [
                ("Raw ADC code", 0),
                ("Voltage (mV)", 1),
            ],
            display_mode,
        )
        self.gain = QtWidgets.QDoubleSpinBox()
        self.gain.setRange(0.0001, 1000.0)
        self.gain.setDecimals(6)
        self.gain.setValue(args.gain)
        self.offset_mv = QtWidgets.QDoubleSpinBox()
        self.offset_mv.setRange(-1_000_000.0, 1_000_000.0)
        self.offset_mv.setDecimals(3)
        self.offset_mv.setValue(args.offset_mv)
        self.fullscale_mv = QtWidgets.QDoubleSpinBox()
        self.fullscale_mv.setRange(1.0, 1_000_000.0)
        self.fullscale_mv.setDecimals(3)
        self.fullscale_mv.setValue(args.fullscale_mv)
        self.frame_words = QtWidgets.QSpinBox()
        self.frame_words.setRange(MIN_FRAME_WORDS, 16_000_000)
        self.frame_words.setValue(max(MIN_FRAME_WORDS, int(args.frame_words)))
        self.pretrigger_words = QtWidgets.QSpinBox()
        self.pretrigger_words.setRange(0, 16_000_000)
        self.pretrigger_words.setValue(args.pretrigger_words)
        self.sample_decimation = QtWidgets.QSpinBox()
        self.sample_decimation.setRange(1, 1_000_000)
        self.sample_decimation.setValue(args.sample_decimation)
        self.trigger_cfg = self._make_combo(
            [
                ("Trigger disabled", 0x0),
                ("External only", 0x1),
                ("Soft only", 0x2),
                ("External + soft", 0x3),
                ("Self only", 0x4),
                ("External + self", 0x5),
                ("Soft + self", 0x6),
                ("External + soft + self", 0x7),
            ],
            int(args.trigger_cfg) & 0x7,
        )
        self.self_polarity_widget = QtWidgets.QWidget()
        self.self_polarity_layout = QtWidgets.QHBoxLayout(self.self_polarity_widget)
        self.self_polarity_layout.setContentsMargins(0, 0, 0, 0)
        self.self_polarity_layout.setSpacing(10)
        self.self_rising_radio = QtWidgets.QRadioButton("Rising edge")
        self.self_falling_radio = QtWidgets.QRadioButton("Falling edge")
        self.self_polarity_layout.addWidget(self.self_rising_radio)
        self.self_polarity_layout.addWidget(self.self_falling_radio)
        self.self_polarity_layout.addStretch(1)
        if (int(args.trigger_cfg) & SELF_TRIGGER_FALLING) != 0:
            self.self_falling_radio.setChecked(True)
        else:
            self.self_rising_radio.setChecked(True)
        self.self_threshold = QtWidgets.QSpinBox()
        self.self_threshold.setRange(0, 4095)
        self.self_threshold.setValue(args.self_threshold)
        self.self_threshold_mv = QtWidgets.QDoubleSpinBox()
        self.self_threshold_mv.setRange(-1_000_000.0, 1_000_000.0)
        self.self_threshold_mv.setDecimals(3)
        self.self_threshold_mv.setSuffix(" mV")
        self.self_threshold_mv.setValue(
            adc_code_to_mv(
                int(args.self_threshold),
                float(args.gain),
                float(args.offset_mv),
                float(args.fullscale_mv),
            )
        )
        self.self_threshold_widget = QtWidgets.QWidget()
        self.self_threshold_layout = QtWidgets.QHBoxLayout(self.self_threshold_widget)
        self.self_threshold_layout.setContentsMargins(0, 0, 0, 0)
        self.self_threshold_layout.setSpacing(8)
        self.self_threshold_layout.addWidget(QtWidgets.QLabel("Code"))
        self.self_threshold_layout.addWidget(self.self_threshold)
        self.self_threshold_layout.addWidget(QtWidgets.QLabel("mV"))
        self.self_threshold_layout.addWidget(self.self_threshold_mv)
        self.self_threshold_layout.addStretch(1)
        self.self_mask_checks: list[QtWidgets.QCheckBox] = []
        self.output_mask_checks: list[QtWidgets.QCheckBox] = []
        channel_names = ["ADC1_A", "ADC1_B", "ADC2_A", "ADC2_B"]
        self.channel_mask_box = QtWidgets.QGroupBox("Channel mask")
        channel_mask_layout = QtWidgets.QGridLayout(self.channel_mask_box)
        channel_mask_layout.setContentsMargins(10, 8, 10, 8)
        channel_mask_layout.setHorizontalSpacing(14)
        channel_mask_layout.setVerticalSpacing(4)
        channel_mask_layout.setColumnMinimumWidth(0, 76)
        channel_mask_layout.setColumnMinimumWidth(1, 86)
        channel_mask_layout.setColumnMinimumWidth(2, 70)
        self_trigger_header = QtWidgets.QLabel("Self trigger")
        self_trigger_header.setAlignment(QtCore.Qt.AlignCenter)
        output_header = QtWidgets.QLabel("Output")
        output_header.setAlignment(QtCore.Qt.AlignCenter)
        channel_mask_layout.addWidget(self_trigger_header, 0, 1)
        channel_mask_layout.addWidget(output_header, 0, 2)
        for row, name in enumerate(channel_names, start=1):
            self_check = QtWidgets.QCheckBox()
            output_check = QtWidgets.QCheckBox()
            self.self_mask_checks.append(self_check)
            self.output_mask_checks.append(output_check)
            self_check.stateChanged.connect(
                lambda state, index=row - 1: self._handle_self_mask_changed(index, state)
            )
            channel_mask_layout.addWidget(QtWidgets.QLabel(name), row, 0)
            channel_mask_layout.addWidget(self_check, row, 1, alignment=QtCore.Qt.AlignCenter)
            channel_mask_layout.addWidget(output_check, row, 2, alignment=QtCore.Qt.AlignCenter)
        self._set_channel_mask_value(args.channel_mask)
        self.output_cfg_checks: list[QtWidgets.QCheckBox] = []
        self.output_cfg_box = QtWidgets.QGroupBox("Output cfg")
        output_cfg_layout = QtWidgets.QGridLayout(self.output_cfg_box)
        output_cfg_layout.setContentsMargins(10, 8, 10, 8)
        output_cfg_layout.setHorizontalSpacing(10)
        output_cfg_layout.setVerticalSpacing(4)
        output_cfg_items = [
            ("PL baseline correction", 0),
            ("Append footer", 1),
            ("Output mask", 2),
            ("Peak signed max", 3),
            ("Footer only", 4),
        ]
        for row, (label, bit) in enumerate(output_cfg_items):
            check = QtWidgets.QCheckBox()
            if bit == 1:
                check.setChecked(True)
                check.setEnabled(False)
            self.output_cfg_checks.append(check)
            output_cfg_layout.addWidget(QtWidgets.QLabel(label), row, 0)
            output_cfg_layout.addWidget(check, row, 1, alignment=QtCore.Qt.AlignCenter)
        self.raw_every_n_enable = QtWidgets.QCheckBox()
        self.raw_every_n_interval = QtWidgets.QSpinBox()
        self.raw_every_n_interval.setRange(1, OUTPUT_CFG_RAW_PERIOD_MASK)
        self.raw_every_n_interval.setSuffix(" frames")
        self.raw_every_n_widget = QtWidgets.QWidget()
        self.raw_every_n_layout = QtWidgets.QHBoxLayout(self.raw_every_n_widget)
        self.raw_every_n_layout.setContentsMargins(0, 0, 0, 0)
        self.raw_every_n_layout.setSpacing(8)
        self.raw_every_n_layout.addWidget(self.raw_every_n_enable)
        self.raw_every_n_layout.addWidget(self.raw_every_n_interval)
        self.raw_every_n_layout.addStretch(1)
        output_cfg_layout.addWidget(QtWidgets.QLabel("Raw every N"), len(output_cfg_items), 0)
        output_cfg_layout.addWidget(self.raw_every_n_widget, len(output_cfg_items), 1)
        self._set_output_cfg_value(args.output_cfg)
        self.baseline_shift = QtWidgets.QSpinBox()
        self.baseline_shift.setRange(0, 20)
        self.baseline_shift.setPrefix("N = ")
        self.baseline_shift.setValue(args.baseline_shift)
        self.baseline_window_label = QtWidgets.QLabel()
        self.baseline_window_label.setObjectName("metaValue")
        self.save_enable = QtWidgets.QCheckBox("Save frames")
        self.save_enable.setChecked(True)
        self.save_format = QtWidgets.QComboBox()
        self.save_format.addItem("raw stream", "raw-stream")
        self.save_format.addItem("raw .bin", "raw-bin")
        self.save_format.addItem("npz", "npz")
        self.save_format.addItem("compressed npz", "npz-compressed")
        MainWindow._set_combo_value(self.save_format, args.save_format)
        self.plot_enable = QtWidgets.QCheckBox("Plot waveform")
        self.plot_enable.setChecked(True)
        self.continuous_shots = QtWidgets.QCheckBox("Continuous")
        self.continuous_shots.setChecked(args.shot_limit <= 0)
        self.continuous_shots.setToolTip("Receive frames continuously until Stop UDP is pressed.")
        self.shot_limit = QtWidgets.QSpinBox()
        self.shot_limit.setRange(1, 1_000_000)
        self.shot_limit.setValue(max(1, args.shot_limit))
        self.shot_limit.setEnabled(not self.continuous_shots.isChecked())
        self.shot_limit.setToolTip("Stop UDP receiver automatically after this many completed frames.")
        self.output_dir = QtWidgets.QLineEdit(str(Path(args.output_dir)))

        self.start_button = QtWidgets.QPushButton("Start UDP")
        self.stop_button = QtWidgets.QPushButton("Stop UDP")
        self.reset_udp_button = QtWidgets.QPushButton("Reset")
        self.settings_button = QtWidgets.QPushButton("Settings")
        self.apply_button = QtWidgets.QPushButton("Apply PL config")
        self.restore_defaults_button = QtWidgets.QPushButton("Restore defaults")
        self.soft_button = QtWidgets.QPushButton("Soft trigger")
        self.browse_button = QtWidgets.QPushButton("Browse")
        self.status_badge = QtWidgets.QLabel("IDLE")
        self.status_badge.setObjectName("statusBadge")
        self.reset_udp_button.setToolTip("Disable PL acquisition, request CONTROL[4] reset, re-enable PL acquisition, and restart UDP if active.")
        self.restore_defaults_button.setToolTip("Restore GUI fields to startup defaults. Use Apply PL config to write them to PL.")
        self.output_dir.setMinimumWidth(320)
        self.output_dir.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.start_button.setObjectName("startButton")
        self.stop_button.setObjectName("stopButton")
        self.reset_udp_button.setObjectName("resetButton")
        self.soft_button.setObjectName("softButton")
        self.settings_button.setObjectName("settingsButton")
        self.browse_button.setObjectName("browseButton")
        for button in [
            self.start_button,
            self.stop_button,
            self.reset_udp_button,
            self.soft_button,
            self.settings_button,
            self.browse_button,
        ]:
            button.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.status_badge.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.config_change_widgets = [
            self.bind_ip,
            self.data_port,
            self.control_ip,
            self.control_port,
            self.mode,
            self.gain,
            self.offset_mv,
            self.fullscale_mv,
            self.frame_words,
            self.pretrigger_words,
            self.sample_decimation,
            self.trigger_cfg,
            self.self_polarity_widget,
            self.self_threshold_widget,
            self.channel_mask_box,
            self.output_cfg_box,
            self.baseline_shift,
            self.apply_button,
            self.restore_defaults_button,
        ]

        def make_settings_group(title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QGridLayout]:
            box = QtWidgets.QGroupBox(title)
            layout = QtWidgets.QGridLayout(box)
            layout.setContentsMargins(12, 12, 12, 10)
            layout.setHorizontalSpacing(10)
            layout.setVerticalSpacing(8)
            layout.setColumnStretch(1, 1)
            return box, layout

        def add_setting_row(
            layout: QtWidgets.QGridLayout,
            row: int,
            label: str,
            widget: QtWidgets.QWidget,
            tooltip: str = "",
        ) -> None:
            label_widget = QtWidgets.QLabel(label)
            if tooltip:
                label_widget.setToolTip(tooltip)
                widget.setToolTip(tooltip)
            layout.addWidget(label_widget, row, 0)
            layout.addWidget(widget, row, 1)

        network_box, network_layout = make_settings_group("1. Network")
        add_setting_row(network_layout, 0, "Bind IP", self.bind_ip, "Local interface for UDP data packets.")
        add_setting_row(network_layout, 1, "Data port", self.data_port, "UDP port used for waveform frame data.")
        add_setting_row(network_layout, 2, "Control IP", self.control_ip, "PL control endpoint address.")
        add_setting_row(network_layout, 3, "Control port", self.control_port, "UDP port used for PL register control.")

        display_box, display_layout = make_settings_group("2. Display")
        add_setting_row(display_layout, 0, "Mode", self.mode, "GUI display only. Baseline correction is controlled by PL output.")
        add_setting_row(display_layout, 1, "Gain (x)", self.gain, "Display voltage scale multiplier.")
        add_setting_row(display_layout, 2, "Offset (mV)", self.offset_mv, "Display-only voltage offset.")
        add_setting_row(display_layout, 3, "Full-scale (mVpp)", self.fullscale_mv, "ADC full-scale peak-to-peak voltage.")

        capture_box, capture_layout = make_settings_group("3. Capture")
        add_setting_row(capture_layout, 0, "Frame words", self.frame_words, "Number of waveform samples per accepted trigger.")
        add_setting_row(capture_layout, 1, "Pretrigger", self.pretrigger_words, "Samples kept before the trigger point.")
        add_setting_row(capture_layout, 2, "Decimation", self.sample_decimation, "Sample decimation written to PL.")

        trigger_box, trigger_layout = make_settings_group("4. Trigger")
        add_setting_row(trigger_layout, 0, "Trigger source", self.trigger_cfg, "Select which trigger sources PL accepts.")
        add_setting_row(trigger_layout, 1, "Self threshold (raw ADC)", self.self_threshold_widget, "Raw ADC threshold used by PL self trigger. Code and mV fields are synchronized; PL receives the code value.")
        add_setting_row(trigger_layout, 2, "Self polarity", self.self_polarity_widget, "Select rising or falling threshold crossing for self trigger.")
        trigger_layout.addWidget(self.channel_mask_box, 3, 0, 1, 2)

        output_box, output_layout = make_settings_group("5. PL Output")
        output_layout.addWidget(self.output_cfg_box, 0, 0, 1, 2)
        baseline_window_widget = QtWidgets.QWidget()
        baseline_window_layout = QtWidgets.QHBoxLayout(baseline_window_widget)
        baseline_window_layout.setContentsMargins(0, 0, 0, 0)
        baseline_window_layout.setSpacing(8)
        baseline_window_layout.addWidget(self.baseline_shift)
        baseline_window_layout.addWidget(self.baseline_window_label)
        baseline_window_layout.addStretch(1)
        add_setting_row(
            output_layout,
            1,
            "Baseline window",
            baseline_window_widget,
            "PL baseline sample count. The register stores exponent N; window length is 2^N samples.",
        )
        controls.addWidget(network_box, 0, 0)
        controls.addWidget(display_box, 0, 1)
        controls.addWidget(capture_box, 1, 0)
        controls.addWidget(trigger_box, 1, 1)
        controls.addWidget(output_box, 2, 0, 1, 2)
        controls.setColumnStretch(0, 1)
        controls.setColumnStretch(1, 1)

        settings_actions = QtWidgets.QHBoxLayout()
        settings_actions.setSpacing(8)
        settings_actions.addWidget(self.apply_button)
        settings_actions.addWidget(self.restore_defaults_button)
        settings_actions.addStretch(1)
        controls.addLayout(settings_actions, 3, 0, 1, 2)

        command_controls.addWidget(self.start_button)
        command_controls.addWidget(self.stop_button)
        command_controls.addWidget(self.reset_udp_button)
        command_controls.addWidget(self.soft_button)
        command_controls.addWidget(self.settings_button)
        command_controls.addSpacing(8)
        command_controls.addWidget(self.status_badge)
        command_controls.addStretch(1)
        save_controls.addWidget(self.save_enable)
        save_controls.addWidget(QtWidgets.QLabel("Format"))
        save_controls.addWidget(self.save_format)
        save_controls.addWidget(self.plot_enable)
        save_controls.addWidget(self.continuous_shots)
        save_controls.addWidget(QtWidgets.QLabel("Shots"))
        save_controls.addWidget(self.shot_limit)
        save_controls.addWidget(QtWidgets.QLabel("Save dir"))
        save_controls.addWidget(self.output_dir, 1)
        save_controls.addWidget(self.browse_button)

        self.stats = QtWidgets.QTableWidget(8, 4)
        self.stats.setObjectName("statsTable")
        self.stats.setHorizontalHeaderLabels(["Item", "Value", "Item", "Value"])
        self.stats.verticalHeader().setVisible(False)
        self.stats.horizontalHeader().setVisible(False)
        self.stats.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.stats.verticalHeader().setDefaultSectionSize(22)
        self.stats.horizontalHeader().setFixedHeight(0)
        self.stats.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.stats.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.stats.setFixedHeight(184)
        self.stats.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)

        self.footer_box = QtWidgets.QWidget()
        footer_layout = QtWidgets.QGridLayout(self.footer_box)
        footer_layout.setContentsMargins(0, 0, 0, 0)
        footer_layout.setHorizontalSpacing(16)
        footer_layout.setVerticalSpacing(4)
        footer_layout.setColumnMinimumWidth(0, 440)
        footer_layout.setColumnStretch(1, 1)

        footer_summary = QtWidgets.QWidget()
        footer_summary_layout = QtWidgets.QGridLayout(footer_summary)
        footer_summary_layout.setContentsMargins(0, 0, 0, 0)
        footer_summary_layout.setHorizontalSpacing(10)
        footer_summary_layout.setVerticalSpacing(4)
        footer_summary_layout.setColumnMinimumWidth(0, 72)
        footer_summary_layout.setColumnStretch(1, 1)
        self.footer_value_labels: dict[str, QtWidgets.QLabel] = {}
        footer_fields = [
            "Status",
            "Event",
            "Magic",
            "Format",
            "Output cfg",
            "Timestamp",
            "Capture cfg",
        ]
        for row, field in enumerate(footer_fields):
            name_label = QtWidgets.QLabel(field)
            name_label.setObjectName("metaName")
            value_label = QtWidgets.QLabel("-")
            value_label.setObjectName("metaValue")
            value_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value_label.setWordWrap(field in {"Status", "Capture cfg", "Timestamp", "Event", "Format"})
            value_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
            footer_summary_layout.addWidget(name_label, row, 0)
            footer_summary_layout.addWidget(value_label, row, 1)
            self.footer_value_labels[field] = value_label
        footer_layout.addWidget(footer_summary, 0, 0)

        self.footer_channel_table = QtWidgets.QTableWidget(4, 5)
        self.footer_channel_table.setObjectName("footerTable")
        self.footer_channel_table.setHorizontalHeaderLabels(
            [
                "Channel",
                "Baseline mean",
                "Peak value",
                "Peak index",
                "Integral",
            ]
        )
        self.footer_channel_table.setVerticalHeaderLabels([])
        self.footer_channel_table.verticalHeader().setVisible(False)
        self.footer_channel_table.horizontalHeader().setStretchLastSection(False)
        self.footer_channel_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.footer_channel_table.verticalHeader().setDefaultSectionSize(23)
        self.footer_channel_table.horizontalHeader().setFixedHeight(26)
        self.footer_channel_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.footer_channel_table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.footer_channel_table.setFixedHeight(128)
        self.footer_channel_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.footer_channel_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        footer_layout.addWidget(self.footer_channel_table, 0, 1)

        raw_name_label = QtWidgets.QLabel("Raw words")
        raw_name_label.setObjectName("metaName")
        raw_value_label = QtWidgets.QLabel("-")
        raw_value_label.setObjectName("metaValue")
        raw_value_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        raw_value_label.setWordWrap(False)
        raw_value_label.setMaximumHeight(22)
        raw_value_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        footer_layout.addWidget(raw_name_label, 1, 0)
        footer_layout.addWidget(raw_value_label, 1, 1)
        self.footer_value_labels["Raw words"] = raw_value_label
        raw_name_label.hide()
        raw_value_label.hide()

        self._update_footer_panel(None)

        plot_panel = QtWidgets.QWidget()
        plot_panel.setObjectName("plotPanel")
        plot_panel.setMinimumHeight(610)
        plot_panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        plot_grid = QtWidgets.QGridLayout(plot_panel)
        plot_grid.setContentsMargins(0, 0, 0, 0)
        plot_grid.setSpacing(8)

        self.plots: list[pg.PlotWidget] = []
        self.curves = []
        colors = ["#60a5fa", "#34d399", "#fbbf24", "#f87171"]
        for idx, color in enumerate(colors, start=1):
            plot = pg.PlotWidget()
            plot.setBackground("#0b0f14")
            plot.setMinimumSize(320, 260)
            plot.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            plot.setTitle(f"CH{idx}", color=color, size="11pt")
            plot.setLabel("left", f"CH{idx}", units="mV")
            if idx > 2:
                plot.setLabel("bottom", "Sample Index")
            else:
                plot.setLabel("bottom", "")
            plot.showGrid(x=True, y=True, alpha=0.18)
            plot.setMouseEnabled(x=False, y=False)
            plot.enableAutoRange(x=False, y=False)
            plot.getViewBox().setMenuEnabled(False)
            plot.hideButtons()
            plot.getAxis("left").setPen(pg.mkPen("#5b6470"))
            plot.getAxis("bottom").setPen(pg.mkPen("#5b6470"))
            plot.getAxis("left").setTextPen(pg.mkPen("#c8d0da"))
            plot.getAxis("bottom").setTextPen(pg.mkPen("#c8d0da"))
            curve = plot.plot(pen=pg.mkPen(color, width=1.8))
            self.plots.append(plot)
            self.curves.append(curve)
            plot_grid.addWidget(plot, (idx - 1) // 2, (idx - 1) % 2)

        plot_grid.setRowStretch(0, 1)
        plot_grid.setRowStretch(1, 1)
        plot_grid.setColumnStretch(0, 1)
        plot_grid.setColumnStretch(1, 1)
        root.addWidget(plot_panel, stretch=10)

        frame_footer_box = QtWidgets.QGroupBox("Frame / footer")
        frame_footer_box.setObjectName("frameFooterBox")
        frame_footer_box.setMinimumHeight(236)
        frame_footer_box.setMaximumHeight(268)
        frame_footer_layout = QtWidgets.QHBoxLayout(frame_footer_box)
        frame_footer_layout.setContentsMargins(10, 10, 10, 8)
        frame_footer_layout.setSpacing(12)
        frame_footer_layout.addWidget(self.stats, 1)
        frame_footer_layout.addWidget(self.footer_box, 2)
        root.addWidget(frame_footer_box)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setObjectName("logBox")
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(56)
        root.addWidget(self.log_box)

        self.start_button.clicked.connect(self.start_udp)
        self.stop_button.clicked.connect(self.stop_udp)
        self.reset_udp_button.clicked.connect(self.reset_udp)
        self.settings_button.clicked.connect(self.show_settings_dialog)
        self.close_settings_button.clicked.connect(self.settings_dialog.hide)
        self.apply_button.clicked.connect(self.apply_settings)
        self.restore_defaults_button.clicked.connect(self.restore_default_settings)
        self.soft_button.clicked.connect(self.send_soft_trigger)
        self.browse_button.clicked.connect(self.choose_output_dir)
        self.continuous_shots.stateChanged.connect(
            lambda _value: self.shot_limit.setEnabled(not self.continuous_shots.isChecked())
        )
        self.self_threshold.valueChanged.connect(lambda _value: self._update_self_threshold_mv_from_code())
        self.self_threshold_mv.valueChanged.connect(lambda _value: self._update_self_threshold_code_from_mv())
        self.mode.currentIndexChanged.connect(lambda _value: self._lock_plot_ranges())
        self.frame_words.valueChanged.connect(lambda _value: self._lock_plot_ranges())
        self.gain.valueChanged.connect(lambda _value: self._display_scale_changed())
        self.offset_mv.valueChanged.connect(lambda _value: self._display_scale_changed())
        self.fullscale_mv.valueChanged.connect(lambda _value: self._display_scale_changed())
        self.baseline_shift.valueChanged.connect(lambda _value: self._update_baseline_window_label())
        for check in self.output_cfg_checks:
            check.stateChanged.connect(lambda _value: self._lock_plot_ranges())

        self.stop_button.setEnabled(False)
        self.reset_udp_button.setEnabled(True)
        self.set_configuration_enabled(True)
        self._set_run_state("idle")
        self._set_stat("Status", "Idle")
        self._set_stat(
            "Frame bytes",
            f"136..{(int(self.frame_words.value()) + PL_FOOTER_WORDS) * 8}",
            1,
        )
        self._set_stat("last frame age", "-", 10)
        self._set_stat("frames/s", "-", 11)
        self._update_baseline_window_label()
        self._update_self_threshold_mv_from_code()
        self._lock_plot_ranges()

        self.health_timer = QtCore.QTimer(self)
        self.health_timer.timeout.connect(self.update_receiver_health)
        self.health_timer.start(1000)
        QtCore.QTimer.singleShot(0, self.sync_settings_on_start)

    def show_settings_dialog(self) -> None:
        self.settings_dialog.show()
        self.settings_dialog.raise_()
        self.settings_dialog.activateWindow()

    def _update_baseline_window_label(self) -> None:
        exponent = int(self.baseline_shift.value())
        samples = 1 << max(0, min(20, exponent))
        self.baseline_window_label.setText(f"2^{exponent} = {samples:,} samples")

    def _threshold_code_to_mv(self, code: int) -> float:
        return adc_code_to_mv(
            int(code),
            float(self.gain.value()),
            float(self.offset_mv.value()),
            float(self.fullscale_mv.value()),
        )

    def _threshold_mv_to_code(self, mv: float) -> int:
        return mv_to_adc_code(
            float(mv),
            float(self.gain.value()),
            float(self.offset_mv.value()),
            float(self.fullscale_mv.value()),
        )

    def _update_self_threshold_mv_from_code(self) -> None:
        if self._updating_self_threshold:
            return
        self._updating_self_threshold = True
        self.self_threshold_mv.setValue(
            self._threshold_code_to_mv(int(self.self_threshold.value()))
        )
        self._updating_self_threshold = False

    def _update_self_threshold_code_from_mv(self) -> None:
        if self._updating_self_threshold:
            return
        self._updating_self_threshold = True
        self.self_threshold.setValue(
            self._threshold_mv_to_code(float(self.self_threshold_mv.value()))
        )
        self._updating_self_threshold = False

    def _display_scale_changed(self) -> None:
        self._lock_plot_ranges()
        self._update_self_threshold_mv_from_code()

    def restore_default_settings(self) -> None:
        if self.stop_button.isEnabled():
            self.log("Stop UDP before restoring settings")
            return

        self.bind_ip.setText(DEFAULT_BIND_IP)
        self.data_port.setValue(DEFAULT_DATA_PORT)
        self.control_ip.setText(DEFAULT_CONTROL_IP)
        self.control_port.setValue(DEFAULT_CONTROL_PORT)
        self._set_combo_value(self.mode, 1)
        self.gain.setValue(1.0)
        self.offset_mv.setValue(0.0)
        self.fullscale_mv.setValue(2000.0)
        self.frame_words.setValue(DEFAULT_FRAME_WORDS)
        self.pretrigger_words.setValue(DEFAULT_PRETRIGGER_WORDS)
        self.sample_decimation.setValue(1)
        self._set_trigger_cfg_value(1)
        self.self_threshold.setValue(2048)
        self._update_self_threshold_mv_from_code()
        self._set_channel_mask_value(0xF)
        self._set_output_cfg_value(OUTPUT_CFG_APPEND_FOOTER)
        self.baseline_shift.setValue(10)
        self._lock_plot_ranges()
        self.log("Default settings restored in GUI; click Apply PL config to write them to PL")

    def set_configuration_enabled(self, enabled: bool) -> None:
        for widget in self.config_change_widgets:
            widget.setEnabled(enabled)

        if not enabled:
            self.apply_button.setToolTip("Stop UDP before changing PL or receiver settings.")
        else:
            self.apply_button.setToolTip("")

    @staticmethod
    def _refresh_style(widget: QtWidgets.QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def _set_button_active(self, button: QtWidgets.QPushButton, active: bool) -> None:
        button.setProperty("active", "true" if active else "false")
        self._refresh_style(button)

    def _set_run_state(self, state: str) -> None:
        labels = {
            "idle": "IDLE",
            "listening": "LISTENING",
            "stopped": "STOPPED",
            "resetting": "RESETTING",
        }
        state = state if state in labels else "idle"
        self.status_badge.setText(labels[state])
        self.status_badge.setProperty("status", state)
        self._refresh_style(self.status_badge)

        self._set_button_active(self.start_button, state == "listening")
        self._set_button_active(self.stop_button, state == "stopped")
        self._set_button_active(self.reset_udp_button, state == "resetting")

    def clear_runtime_state(self, status: str) -> None:
        self.last_frame_received_at = None
        self.frame_rate_times.clear()
        self._last_decode_warning = ""
        self._last_trig_time = None

        for curve in self.curves:
            curve.setData([], [])
            curve.setVisible(True)

        self._update_footer_panel(None)
        self._set_stat("Status", status, 0)
        for row in range(1, self.stats.rowCount() * 2):
            self._set_stat("", "", row)
        self._set_stat("last frame age", "-", 10)
        self._set_stat("frames/s", "-", 11)

    def record_frame_rate_sample(self, timestamp: float) -> None:
        self.frame_rate_times.append(timestamp)
        self.trim_frame_rate_samples(timestamp)

    def trim_frame_rate_samples(self, now: float) -> None:
        cutoff = now - self.frame_rate_window_s
        while self.frame_rate_times and self.frame_rate_times[0] < cutoff:
            self.frame_rate_times.popleft()

    def current_frame_rate(self, now: float | None = None) -> float:
        if now is None:
            now = time.time()

        self.trim_frame_rate_samples(now)
        if len(self.frame_rate_times) < 2:
            return 0.0

        elapsed = self.frame_rate_times[-1] - self.frame_rate_times[0]
        if elapsed <= 0.0:
            return 0.0

        return (len(self.frame_rate_times) - 1) / elapsed

    def update_frame_rate_stat(self, now: float | None = None) -> None:
        fps = self.current_frame_rate(now)
        self._set_stat("frames/s", f"{fps:.2f}", 11)

    def settings(self) -> dict:
        return {
            "mode": self._combo_value(self.mode),
            "gain": float(self.gain.value()),
            "offset_mv": float(self.offset_mv.value()),
            "fullscale_mv": float(self.fullscale_mv.value()),
            "channel_mask": self._channel_mask_value(),
            "output_cfg": self._output_cfg_value(),
            "baseline_shift": int(self.baseline_shift.value()),
        }

    def _lock_plot_ranges(
        self,
        sample_count: int | None = None,
        pl_corrected_override: bool | None = None,
        y_ranges: list[tuple[float, float] | None] | None = None,
    ) -> None:
        if sample_count is None:
            sample_count = int(self.frame_words.value())

        sample_count = max(2, int(sample_count))
        x_max = sample_count - 1
        mode = self._combo_value(self.mode)
        if pl_corrected_override is None:
            pl_corrected = (self._output_cfg_value() & 0x1) != 0
        else:
            pl_corrected = bool(pl_corrected_override)

        if mode == 0:
            if pl_corrected:
                default_y_min = -4096.0
                default_y_max = 4096.0
            else:
                default_y_min = 0.0
                default_y_max = 4095.0
            unit = "code"
        else:
            gain = float(self.gain.value())
            offset = float(self.offset_mv.value())
            fullscale = float(self.fullscale_mv.value())
            y_span = max(1.0, abs(fullscale * 0.5 * gain))
            default_y_min = offset - y_span
            default_y_max = offset + y_span

            unit = "mV"

        for idx, plot in enumerate(self.plots, start=1):
            y_min, y_max = default_y_min, default_y_max
            if y_ranges is not None and idx - 1 < len(y_ranges):
                channel_range = y_ranges[idx - 1]
                if channel_range is not None:
                    y_min, y_max = channel_range

            plot.setLabel("left", f"CH{idx}", units=unit)
            view_box = plot.getViewBox()
            view_box.setMouseEnabled(x=False, y=False)
            view_box.setMenuEnabled(False)
            view_box.setLimits(
                xMin=0,
                xMax=x_max,
                yMin=y_min,
                yMax=y_max,
            )
            plot.setXRange(0, x_max, padding=0)
            plot.setYRange(y_min, y_max, padding=0)

    @staticmethod
    def _auto_y_range(values: np.ndarray) -> tuple[float, float] | None:
        if values.size == 0:
            return None

        finite_values = values[np.isfinite(values)]
        if finite_values.size == 0:
            return None

        y_min = float(np.min(finite_values))
        y_max = float(np.max(finite_values))
        if y_min == y_max:
            padding = max(1.0, abs(y_min) * 0.05)
        else:
            padding = max(1.0, (y_max - y_min) * 0.08)

        return y_min - padding, y_max + padding

    def _channels_for_display(
        self,
        channels_raw: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
        settings: dict,
        pl_corrected: bool = False,
    ) -> list[np.ndarray]:
        mode = int(settings["mode"])

        if mode == 0:
            return [
                channel.astype(np.float32, copy=False)
                for channel in channels_raw
            ]

        if pl_corrected:
            return [
                corrected_codes_to_mv(
                    channel,
                    settings["gain"],
                    settings["fullscale_mv"],
                )
                for channel in channels_raw
            ]

        channels_mv = [
            codes_to_mv(
                channel,
                settings["gain"],
                settings["offset_mv"],
                settings["fullscale_mv"],
            )
            for channel in channels_raw
        ]
        return channels_mv

    def start_udp(self, ignore_frames: bool = False) -> None:
        if not isinstance(ignore_frames, bool):
            ignore_frames = False
        self.saver.enabled = self.save_enable.isChecked()
        self.saver.output_dir = Path(self.output_dir.text())
        self.saver.save_format = str(self.save_format.currentData())
        self.active_plot_enabled = self.plot_enable.isChecked()
        footer_words = PL_FOOTER_WORDS
        self._last_decode_warning = ""
        self._last_trig_time = None
        self.last_frame_received_at = None
        self.frame_rate_times.clear()
        self.shots_received = 0
        self._shot_limit_stop_pending = False
        self.shot_limit_uses_save_queue = bool(self.save_enable.isChecked())
        self.ignore_received_frames = ignore_frames
        self.post_apply_drop_frames = 0
        if time.time() >= self.discard_frames_until_s:
            self.discard_frames_until_s = 0.0
        self.post_apply_discard_count = 0
        self.active_shot_limit = (
            0
            if self.ignore_received_frames or self.continuous_shots.isChecked()
            else int(self.shot_limit.value())
        )
        self.receiver = UdpReceiver(
            self.bind_ip.text(),
            int(self.data_port.value()),
            int(self.frame_words.value()),
            footer_words,
        )
        self.receiver.frame_ready.connect(self.handle_frame)
        self.receiver.status.connect(self.log)
        self.receiver.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.reset_udp_button.setEnabled(True)
        self.save_enable.setEnabled(False)
        self.save_format.setEnabled(False)
        self.plot_enable.setEnabled(False)
        self.continuous_shots.setEnabled(False)
        self.shot_limit.setEnabled(False)
        self.set_configuration_enabled(False)
        self._set_run_state("listening")
        self._set_stat("Status", "Listening")
        self._set_stat("last frame age", "waiting", 10)
        self._set_stat("frames/s", "0.00", 11)
        self._set_stat(
            "shots",
            (
                "ignored"
                if self.ignore_received_frames else
                f"0 / {self.active_shot_limit}"
                if self.active_shot_limit > 0 else
                "continuous"
            ),
            12,
        )
        self.log(
            f"UDP expected frame: waveform=0..{int(self.frame_words.value())} words, "
            f"footer={footer_words} words, bytes=136..{(int(self.frame_words.value()) + footer_words) * 8}"
        )
        if self.active_shot_limit > 0:
            if self.shot_limit_uses_save_queue:
                self.log(f"Shot limit enabled: stop after {self.active_shot_limit} frames are queued for saving")
            else:
                self.log(f"Shot limit enabled: stop after {self.active_shot_limit} received frames")
        if self.ignore_received_frames:
            self.log("Frame processing disabled for temporary UDP receiver")
        elif self.saver.enabled:
            self.log(f"Frame saving enabled: {self.saver.output_dir} ({self.saver.save_format})")
        else:
            self.log("Frame saving disabled")
        self.log("Waveform plotting enabled" if self.active_plot_enabled else "Waveform plotting disabled")
        if not self.active_plot_enabled:
            for curve in self.curves:
                curve.setData([], [])
                curve.setVisible(False)

    def stop_udp(self) -> None:
        if self.receiver:
            self.receiver.stop()
            self.receiver = None
        self.ignore_received_frames = False
        self.post_apply_drop_frames = 0
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.reset_udp_button.setEnabled(True)
        self.save_enable.setEnabled(True)
        self.save_format.setEnabled(True)
        self.plot_enable.setEnabled(True)
        self.continuous_shots.setEnabled(True)
        self.shot_limit.setEnabled(not self.continuous_shots.isChecked())
        self.set_configuration_enabled(True)
        self.clear_runtime_state("Stopped")
        self._set_run_state("stopped")
        if self.save_enable.isChecked() and self.saver.save_format == "raw-stream":
            pending = self.saver.queue.qsize()
            self.saver.finalize_segment()
            self.log(f"UDP stopped; stream finalize queued ({pending} saves pending)")
        else:
            self.log("UDP stopped; receiver state cleared")

    def stop_udp_after_shot_limit(self) -> None:
        if self.receiver:
            self.receiver.stop()
            self.receiver = None
        self.ignore_received_frames = False
        self.post_apply_drop_frames = 0
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.reset_udp_button.setEnabled(True)
        self.save_enable.setEnabled(True)
        self.save_format.setEnabled(True)
        self.plot_enable.setEnabled(True)
        self.continuous_shots.setEnabled(True)
        self.shot_limit.setEnabled(not self.continuous_shots.isChecked())
        self.set_configuration_enabled(True)
        self._set_run_state("stopped")
        self._set_stat("Status", "Shot limit reached", 0)
        self._set_stat("last frame age", "-", 10)
        shot_kind = "save-queued" if self.shot_limit_uses_save_queue else "received"
        if self.save_enable.isChecked() and self.saver.save_format == "raw-stream":
            self.saver.finalize_segment()
        self.log(
            f"Shot limit reached: {shot_kind} {self.shots_received} / "
            f"{self.active_shot_limit} frames; UDP stopped"
        )

    def resume_frame_processing_after_apply(self) -> None:
        if not self.receiver:
            self.ignore_received_frames = False
            return

        remaining_s = self.discard_frames_until_s - time.time()
        if remaining_s > 0.0:
            QtCore.QTimer.singleShot(
                max(1, int(remaining_s * 1000.0)),
                self.resume_frame_processing_after_apply,
            )
            return

        self.ignore_received_frames = False
        self.discard_frames_until_s = 0.0
        self.post_apply_drop_frames = 0
        self._set_stat("Status", "Listening", 0)
        self._set_stat(
            "shots",
            (
                f"{self.shots_received} / {self.active_shot_limit}"
                if self.active_shot_limit > 0 else
                f"{self.shots_received} / continuous"
            ),
            12,
        )
        self.log(
            f"Frame processing resumed after PL config apply "
            f"({self.post_apply_discard_count} frame(s) discarded)"
        )
        self.post_apply_discard_count = 0

    def begin_post_apply_discard_window(self) -> None:
        self.ignore_received_frames = False
        self.post_apply_drop_frames = 0
        self.post_apply_discard_count = 0
        self.discard_frames_until_s = time.time() + POST_APPLY_DISCARD_S
        self._set_stat("Status", "Discarding post-apply frames", 0)
        self._set_stat("shots", "ignored", 12)
        self.log(
            f"Discarding frames received during the {POST_APPLY_DISCARD_S:.1f} s "
            "post-apply flush window"
        )
        QtCore.QTimer.singleShot(
            int(POST_APPLY_DISCARD_S * 1000.0),
            self.resume_frame_processing_after_apply,
        )

    def reset_udp(self) -> None:
        """Reset PL acquisition/control path, then restart UDP if it was active."""
        was_listening = self.stop_button.isEnabled()
        try:
            self._set_pl_acquisition_enabled(False, "PL acquisition disabled for reset")
            reply = self._write_register("control", CONTROL_SOFTWARE_RESET)
            self._log_control_reply("Reset requested", reply)
            self._set_pl_acquisition_enabled(True, "PL acquisition re-enabled after reset")
        except Exception as exc:
            self.log(
                f"Reset not confirmed: no control reply from "
                f"{self.control_ip.text()}:{int(self.control_port.value())} ({exc})"
            )
            self._set_stat("Status", "Reset failed")
            return

        if not was_listening:
            self._set_stat("Status", "Reset complete")
            self.log("Reset complete")
            return

        if self.receiver:
            self.receiver.stop()
            self.receiver = None

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.reset_udp_button.setEnabled(False)
        self.set_configuration_enabled(False)
        self._set_run_state("resetting")
        self._set_stat("Status", "Resetting")
        self._set_stat("last frame age", "-", 10)
        self.log("Resetting PL acquisition, DMA, UDP receiver, and frame assembler")
        QtCore.QTimer.singleShot(350, self.start_udp)

    def update_receiver_health(self) -> None:
        if not self.stop_button.isEnabled():
            return

        if self.last_frame_received_at is None:
            self._set_stat("last frame age", "waiting", 10)
            self._set_stat("frames/s", "0.00", 11)
            return

        now = time.time()
        elapsed_s = max(0.0, now - self.last_frame_received_at)
        self._set_stat("last frame age", f"{elapsed_s:.1f} s", 10)
        self.update_frame_rate_stat(now)

    def apply_settings(self) -> None:
        frame_words = max(MIN_FRAME_WORDS, int(self.frame_words.value()))
        pretrigger_words = min(int(self.pretrigger_words.value()), frame_words)
        if frame_words != int(self.frame_words.value()):
            self.frame_words.setValue(frame_words)
        if pretrigger_words != int(self.pretrigger_words.value()):
            self.pretrigger_words.setValue(pretrigger_words)
            self.log(f"Pretrigger clamped to frame size: {pretrigger_words} words")

        writes = [
            ("frame_size", frame_words),
            ("pretrigger_size", pretrigger_words),
            ("sample_decimation", int(self.sample_decimation.value())),
            ("trigger_cfg", self._trigger_cfg_value()),
            ("self_threshold", int(self.self_threshold.value())),
            ("channel_mask", self._channel_mask_value()),
            ("output_cfg", self._output_cfg_value()),
            ("baseline_shift", int(self.baseline_shift.value())),
        ]
        temporary_udp = not self.stop_button.isEnabled()
        paused_existing_udp = False
        apply_completed = False

        try:
            if temporary_udp:
                self.start_udp(ignore_frames=True)
                QtWidgets.QApplication.processEvents()
                self.log("Temporary UDP receiver started for PL config apply")
            else:
                self.ignore_received_frames = True
                paused_existing_udp = True
                self._set_stat("Status", "Applying config", 0)
                self._set_stat("shots", "ignored", 12)
                self.log("Frame processing paused for PL config apply")

            self._set_pl_acquisition_enabled(False, "PL acquisition disabled for config apply")

            reply = None
            for name, value in writes:
                reply = self._write_register(name, value)
            if reply is not None:
                self._apply_reply_to_widgets(reply)
                self._log_control_reply("Applied PL config", reply)

            reply = self._write_register("control", CONTROL_SOFTWARE_RESET)
            self._log_control_reply("Config apply reset requested", reply)
            self._set_pl_acquisition_enabled(True, "PL acquisition re-enabled after config apply")
            apply_completed = True
        except Exception as exc:
            self.log(f"Control send error: {exc}")
        finally:
            if temporary_udp and self.stop_button.isEnabled():
                if apply_completed:
                    self.begin_post_apply_discard_window()
                self.stop_udp()
                self.log("Temporary UDP receiver stopped after PL config apply")
            elif paused_existing_udp:
                if apply_completed:
                    self.begin_post_apply_discard_window()
                else:
                    self.resume_frame_processing_after_apply()

    def dump_control_registers(self) -> None:
        try:
            reply = send_control_command(
                self.control_ip.text(),
                int(self.control_port.value()),
                CTRL_OP_DUMP,
            )
            self._apply_reply_to_widgets(reply)
            self._log_control_reply("Synced settings from PL", reply)
            status_reply = self._read_register("status")
            self._log_control_reply("PL realtime status", status_reply)
        except Exception as exc:
            self.log(f"PL sync failed: no control reply from {self.control_ip.text()}:{int(self.control_port.value())} ({exc})")

    def sync_settings_on_start(self) -> None:
        self.dump_control_registers()
        self.enable_pl_acquisition_on_start()

    def _set_pl_acquisition_enabled(self, enabled: bool, label: str) -> None:
        reply = send_control_command(
            self.control_ip.text(),
            int(self.control_port.value()),
            CTRL_OP_READ,
            ADC_REGISTERS["control"],
        )
        control = int(reply.value)
        if enabled:
            control |= CONTROL_ACQUISITION_ENABLE
        else:
            control &= ~CONTROL_ACQUISITION_ENABLE
        reply = self._write_register("control", control)
        self._log_control_reply(label, reply)

    def enable_pl_acquisition_on_start(self) -> None:
        try:
            self._set_pl_acquisition_enabled(True, "PL acquisition enabled on GUI start")
        except Exception as exc:
            self.log(
                f"PL acquisition enable skipped: no control reply from "
                f"{self.control_ip.text()}:{int(self.control_port.value())} ({exc})"
            )

    def disable_pl_acquisition_on_exit(self) -> None:
        try:
            self._set_pl_acquisition_enabled(False, "PL acquisition disabled on GUI exit")
        except Exception as exc:
            self.log(
                f"PL acquisition disable not confirmed: no control reply from "
                f"{self.control_ip.text()}:{int(self.control_port.value())} ({exc})"
            )

    def send_soft_trigger(self) -> None:
        try:
            if self.post_apply_drop_frames > 0 or self.discard_frames_until_s > 0.0:
                self.log("Post-apply frame discard canceled before manual soft trigger")
                self.discard_frames_until_s = 0.0
                self.post_apply_discard_count = 0
                self.resume_frame_processing_after_apply()
            if not self.save_enable.isChecked():
                self.log("Save frames is OFF; soft-triggered frame will not be written")
            old_trigger_cfg = self._trigger_cfg_value()
            old_control = send_control_command(
                self.control_ip.text(),
                int(self.control_port.value()),
                CTRL_OP_READ,
                ADC_REGISTERS["control"],
            ).value
            self._write_register("trigger_cfg", old_trigger_cfg | 0x2)
            self._write_register("control", old_control & ~CONTROL_SOFT_TRIGGER)
            time.sleep(SOFT_TRIGGER_LOW_SETTLE_S)
            self._write_register("control", old_control | CONTROL_SOFT_TRIGGER)
            time.sleep(SOFT_TRIGGER_HIGH_PULSE_S)
            self._write_register("control", old_control & ~CONTROL_SOFT_TRIGGER)
            reply = self._write_register("trigger_cfg", old_trigger_cfg)
            self._apply_reply_to_widgets(reply)
            pulse_ms = int(round(SOFT_TRIGGER_HIGH_PULSE_S * 1000))
            self._log_control_reply(f"Soft trigger pulse sent, control[1] held {pulse_ms} ms", reply)
        except Exception as exc:
            self.log(f"Soft trigger error: {exc}")

    def _write_register(self, name: str, value: int) -> ControlReply:
        return send_control_command(
            self.control_ip.text(),
            int(self.control_port.value()),
            CTRL_OP_WRITE,
            ADC_REGISTERS[name],
            value,
        )

    def _read_register(self, name: str) -> ControlReply:
        return send_control_command(
            self.control_ip.text(),
            int(self.control_port.value()),
            CTRL_OP_READ,
            ADC_REGISTERS[name],
        )

    def _apply_reply_to_widgets(self, reply: ControlReply) -> None:
        self.frame_words.setValue(int(reply.frame_size))
        self.pretrigger_words.setValue(int(reply.pretrigger_size))
        self.sample_decimation.setValue(max(1, int(reply.sample_decimation)))
        self._set_trigger_cfg_value(int(reply.trigger_cfg) & 0xF)
        self.self_threshold.setValue(int(reply.self_threshold) & 0xFFF)
        self._update_self_threshold_mv_from_code()
        self._set_channel_mask_value(int(reply.channel_mask) & 0xFF)
        self._set_output_cfg_value(int(reply.output_cfg) & 0xFFFFFFFF)
        self.baseline_shift.setValue(int(reply.baseline_shift))
        self._lock_plot_ranges()

    def _log_control_reply(self, prefix: str, reply: ControlReply) -> None:
        status = CTRL_STATUS_TEXT.get(reply.status, f"STATUS_{reply.status}")
        text = (
            f"{prefix}: {status}, "
            f"frame={reply.frame_size}, pre={reply.pretrigger_size}, "
            f"decim={reply.sample_decimation}, trig=0x{reply.trigger_cfg:X}, "
            f"threshold={reply.self_threshold}, mask=0x{reply.channel_mask:X}, "
            f"output=0x{reply.output_cfg:X}, baseline_shift={reply.baseline_shift}"
        )
        if int(reply.reg) == ADC_REGISTERS["status"]:
            text += f", {self._format_pl_status(int(reply.value))}"
        self.log(text)

    @staticmethod
    def _format_pl_status(value: int) -> str:
        value = int(value) & 0xFFFFFFFF
        return (
            f"pl_status=0x{value:08X} "
            f"(adc1_full={int((value & PL_STATUS_ADC1_FIFO_FULL) != 0)}, "
            f"adc2_full={int((value & PL_STATUS_ADC2_FIFO_FULL) != 0)}, "
            f"any_full={int((value & PL_STATUS_ANY_FIFO_FULL) != 0)}, "
            f"overrun={int((value & PL_STATUS_CAPTURE_OVERRUN) != 0)})"
        )

    @staticmethod
    def _unsigned16(value: int) -> int:
        return value & 0xFFFF

    @staticmethod
    def _signed16(value: int) -> int:
        value &= 0xFFFF
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _signed64(value: int) -> int:
        value &= 0xFFFFFFFFFFFFFFFF
        return value - 0x10000000000000000 if value & 0x8000000000000000 else value

    def _footer_channels_u16(self, word: int) -> dict[str, int]:
        return {
            "ADC1_A": self._unsigned16(word),
            "ADC1_B": self._unsigned16(word >> 16),
            "ADC2_A": self._unsigned16(word >> 32),
            "ADC2_B": self._unsigned16(word >> 48),
        }

    def _footer_channels_i16(self, word: int) -> dict[str, int]:
        return {
            "ADC1_A": self._signed16(word),
            "ADC1_B": self._signed16(word >> 16),
            "ADC2_A": self._signed16(word >> 32),
            "ADC2_B": self._signed16(word >> 48),
        }

    @staticmethod
    def _format_channel_values(values: dict[str, int]) -> str:
        return ", ".join(f"{name}={value}" for name, value in values.items())

    def _set_footer_value(self, field: str, value: str) -> None:
        label = self.footer_value_labels.get(field)
        if label is not None:
            label.setText(value)

    def _set_footer_channel_row(
        self,
        row: int,
        channel: str,
        baseline_mean: int | str,
        peak_value: int | str,
        peak_index: int | str,
        integral: int | str,
    ) -> None:
        values = [
            channel,
            str(baseline_mean),
            str(peak_value),
            str(peak_index),
            str(integral),
        ]

        for col, value in enumerate(values):
            self.footer_channel_table.setItem(
                row,
                col,
                QtWidgets.QTableWidgetItem(value),
            )

    def _clear_footer_channel_table(self) -> None:
        for row, channel in enumerate(["ADC1_A", "ADC1_B", "ADC2_A", "ADC2_B"]):
            self._set_footer_channel_row(row, channel, "-", "-", "-", "-")

    @staticmethod
    def _format_output_cfg_bits(output_cfg: int) -> str:
        enabled = []

        if output_cfg & 0x1:
            enabled.append("baseline corrected")
        if output_cfg & 0x2:
            enabled.append("footer")
        if output_cfg & 0x4:
            enabled.append("output mask")
        if output_cfg & OUTPUT_CFG_FOOTER_ONLY:
            enabled.append("footer only")
        if output_cfg & OUTPUT_CFG_RAW_EVERY_N:
            period = (output_cfg >> OUTPUT_CFG_RAW_PERIOD_SHIFT) & OUTPUT_CFG_RAW_PERIOD_MASK
            enabled.append(f"raw every {max(1, period)}")
        if not enabled:
            enabled.append("raw waveform")

        if output_cfg & OUTPUT_CFG_PEAK_SIGNED_MAX:
            enabled.append("peak signed max")
        else:
            enabled.append("peak abs max")

        return f"0x{output_cfg:02X} ({', '.join(enabled)})"

    def _update_footer_panel(self, footer_words: np.ndarray | None) -> None:
        if footer_words is None or footer_words.size < PL_FOOTER_WORDS:
            self._set_footer_value("Status", "Not present")
            self._set_footer_value("Magic", "-")
            self._set_footer_value("Format", "-")
            self._set_footer_value("Output cfg", "-")
            self._set_footer_value("Timestamp", "-")
            self._set_footer_value("Capture cfg", "-")
            self._set_footer_value("Raw words", "-")
            self._clear_footer_channel_table()
            return

        words = [int(word) for word in footer_words[:PL_FOOTER_WORDS]]
        info = words[1]
        version = (info >> 48) & 0xFFFF
        output_cfg = (info >> 32) & 0xFF
        status_flags = (info >> 24) & 0xFF
        baseline_shift = (info >> 16) & 0xFF
        footer_count = info & 0xFFFF
        baseline_mean = self._footer_channels_u16(words[2])
        peak_value = self._footer_channels_i16(words[3])
        raw_peak_index_adc1_a = words[4] & 0xFFFFFFFF
        raw_peak_index_adc1_b = (words[4] >> 32) & 0xFFFFFFFF
        raw_peak_index_adc2_a = words[5] & 0xFFFFFFFF
        raw_peak_index_adc2_b = (words[5] >> 32) & 0xFFFFFFFF
        integral = {
            "ADC1_A": self._signed64(words[6]),
            "ADC1_B": self._signed64(words[7]),
            "ADC2_A": self._signed64(words[8]),
            "ADC2_B": self._signed64(words[9]),
        }
        captured_frame_size = None
        captured_pre_samples = None
        captured_post_samples = None
        captured_decimation = None
        captured_trigger_cfg = None
        captured_self_threshold = None
        captured_channel_mask = None
        captured_output_cfg = None
        captured_config_seq = None
        captured_trig_time = None

        if len(words) >= 16:
            captured_frame_size = words[11] >> 32
            captured_pre_samples = words[11] & 0xFFFFFFFF
            captured_post_samples = words[12] >> 32
            captured_decimation = words[12] & 0xFFFFFFFF
            captured_trigger_cfg = words[13] >> 32
            captured_self_threshold = words[13] & 0xFFFFFFFF
            captured_channel_mask = words[14] >> 32
            captured_output_cfg = words[14] & 0xFFFFFFFF
            if version >= FOOTER_VERSION_CONFIG_SEQ_WORD10:
                captured_config_seq = words[10] & 0xFFFFFFFF
            elif version >= FOOTER_VERSION_TIMESTAMP:
                captured_config_seq = words[15] & 0xFFFFFFFF
            captured_trig_time = parse_trig_time(version, words[15])

        peak_index_offset = (
            1 if ((captured_trigger_cfg if captured_trigger_cfg is not None else self._trigger_cfg_value()) & TRIGGER_EXTERNAL) != 0 else 0
        )
        peak_index = {
            "ADC1_A": raw_peak_index_adc1_a + peak_index_offset,
            "ADC1_B": raw_peak_index_adc1_b + peak_index_offset,
            "ADC2_A": raw_peak_index_adc2_a + peak_index_offset,
            "ADC2_B": raw_peak_index_adc2_b + peak_index_offset,
        }

        event_word = words[10]
        event_frame_id = (event_word >> 32) & 0xFFFFFFFF
        ignored_trigger_count = (event_word >> 16) & 0xFFFF
        busy_trigger_count = event_word & 0xFFFF
        loss_word = words[16] if len(words) >= 17 else 0
        loss_ignored_count = (loss_word >> 48) & 0xFFFF
        loss_busy_count = (loss_word >> 32) & 0xFFFF
        stopped_trigger_count = (loss_word >> 16) & 0xFFFF
        fifo_trigger_count = loss_word & 0xFFFF
        if len(words) >= 17:
            ignored_trigger_count = loss_ignored_count
            busy_trigger_count = loss_busy_count

        adc1_fifo_full = (status_flags & FIFO_STATUS_ADC1_FULL) != 0
        adc2_fifo_full = (status_flags & FIFO_STATUS_ADC2_FULL) != 0
        any_fifo_full = adc1_fifo_full or adc2_fifo_full
        frame_invalid = (status_flags & FIFO_STATUS_FRAME_INVALID) != 0
        trigger_rejected_seen = (
            (status_flags & FIFO_STATUS_TRIGGER_REJECTED) != 0
        )

        if frame_invalid or any_fifo_full:
            affected = []
            if adc1_fifo_full:
                affected.append("ADC1")
            if adc2_fifo_full:
                affected.append("ADC2")
            affected_text = "/".join(affected) if affected else "unknown FIFO"
            self._set_footer_value(
                "Status",
                f"INVALID: FIFO full during frame ({affected_text}), status=0x{status_flags:02X}",
            )
            if self._last_decode_warning != f"fifo:{status_flags:02X}":
                self.log(
                    "WARNING: FIFO full occurred during frame; "
                    "samples may be dropped or channel alignment may be invalid"
                )
                self._last_decode_warning = f"fifo:{status_flags:02X}"
        else:
            status_text = f"VALID, status=0x{status_flags:02X}"
            if trigger_rejected_seen:
                status_text += ", trigger loss observed"
            self._set_footer_value("Status", status_text)
            self._set_footer_value(
                "Event",
                f"id={event_frame_id}, ignored={ignored_trigger_count}, busy={busy_trigger_count}, "
                f"stopped={stopped_trigger_count}, fifo={fifo_trigger_count} "
                f"(causes may overlap; bank busy is POST-only)",
            )

        self._set_footer_value("Magic", f"0x{words[0]:016X}")
        self._set_footer_value(
            "Format",
            f"version={version}, footer_words={footer_count}, baseline_shift={baseline_shift}",
        )
        self._set_footer_value("Output cfg", self._format_output_cfg_bits(output_cfg))

        if captured_trig_time is None:
            self._set_footer_value("Timestamp", "not available (footer version < 3)")
        else:
            modulus = timestamp_modulus_for_version(version)
            if self._last_trig_time is None:
                timestamp_text = "dt=- (first frame)"
            else:
                delta_ticks = timestamp_delta_ticks(
                    captured_trig_time, self._last_trig_time, modulus
                )
                timestamp_text = f"dt={format_timestamp_ticks(delta_ticks)}"
                if delta_ticks > 0:
                    rate_hz = 1.0 / (delta_ticks * TIMESTAMP_TICK_NS * 1.0e-9)
                    timestamp_text += f" ({rate_hz:.1f} Hz)"
            if version >= FOOTER_VERSION_TIMESTAMP64:
                timestamp_text += (
                    f", raw t={captured_trig_time} tk (64-bit, no practical wrap)"
                )
            else:
                timestamp_text += (
                    f", raw t={captured_trig_time} tk (32-bit, wraps 34.36 s)"
                )
            self._set_footer_value("Timestamp", timestamp_text)
            self._last_trig_time = captured_trig_time
        if captured_frame_size is None:
            self._set_footer_value("Capture cfg", "-")
        else:
            polarity = "falling" if (captured_trigger_cfg & SELF_TRIGGER_FALLING) else "rising"
            threshold_mv = self._threshold_code_to_mv(int(captured_self_threshold))
            seq_text = (
                "" if captured_config_seq is None else f"seq={captured_config_seq}, "
            )
            self._set_footer_value(
                "Capture cfg",
                f"{seq_text}frame={captured_frame_size}, "
                f"pre={captured_pre_samples}, post={captured_post_samples}, "
                f"decim={captured_decimation}, trig=0x{captured_trigger_cfg:X} "
                f"({polarity}), threshold={captured_self_threshold} ({threshold_mv:.3f} mV), "
                f"mask=0x{captured_channel_mask:X}, output=0x{captured_output_cfg:X}",
            )
        for row, channel in enumerate(["ADC1_A", "ADC1_B", "ADC2_A", "ADC2_B"]):
            self._set_footer_channel_row(
                row,
                channel,
                baseline_mean[channel],
                peak_value[channel],
                peak_index[channel],
                integral[channel],
            )
        self._set_footer_value(
            "Raw words",
            " ".join(f"{word:016X}" for word in words),
        )

    def choose_output_dir(self) -> None:
        selected = QtWidgets.QFileDialog.getExistingDirectory(self, "Select capture directory", self.output_dir.text())
        if selected:
            self.output_dir.setText(selected)

    def handle_frame(self, frame: Frame) -> None:
        if self.discard_frames_until_s > 0.0 and frame.received_at <= self.discard_frames_until_s:
            self.post_apply_discard_count += 1
            if self.post_apply_discard_count <= 3 or (self.post_apply_discard_count % 10) == 0:
                self.log(
                    f"Dropped post-apply frame {frame.frame_id} "
                    f"({self.post_apply_discard_count} discarded)"
                )
            return

        if self.ignore_received_frames:
            if self.post_apply_drop_frames > 0:
                self.post_apply_drop_frames -= 1
                self.log("Dropped one post-apply frame")
                if self.post_apply_drop_frames == 0:
                    self.resume_frame_processing_after_apply()
            return

        if self.active_shot_limit > 0 and self.shots_received >= self.active_shot_limit:
            if not self._shot_limit_stop_pending:
                self._shot_limit_stop_pending = True
                if self.receiver:
                    self.receiver.stop()
                QtCore.QTimer.singleShot(0, self.stop_udp_after_shot_limit)
            return

        now = time.time()
        self.last_frame_received_at = now
        self.record_frame_rate_sample(now)
        settings = self.settings()
        gui_output_cfg = int(settings["output_cfg"])
        waveform_words, footer_words = split_waveform_footer(frame.raw_words)
        actual_output_cfg = footer_output_cfg(footer_words)
        if actual_output_cfg is None:
            actual_output_cfg = gui_output_cfg
        captured_config = footer_capture_config(footer_words)

        frame_settings = dict(settings)
        frame_settings["output_cfg"] = int(actual_output_cfg)
        if captured_config is not None:
            frame_settings["channel_mask"] = int(captured_config["channel_mask"])
            frame_settings["baseline_shift"] = int(captured_config["baseline_shift"])

        pl_corrected = (int(actual_output_cfg) & 0x1) != 0
        save_enabled = self.save_enable.isChecked()
        self.saver.enabled = save_enabled
        self.saver.output_dir = Path(self.output_dir.text())
        self.saver.save_format = str(self.save_format.currentData())
        needs_decoded_channels = (
            self.active_plot_enabled or
            (save_enabled and self.saver.save_format in {"npz", "npz-compressed"})
        )
        if not needs_decoded_channels:
            channels_raw = tuple()
        elif pl_corrected:
            channels_raw = decode_corrected_channels(waveform_words)
        else:
            channels_raw = decode_channels(waveform_words)

        if save_enabled:
            save_queued = self.saver.save_later(frame, frame_settings, channels_raw)
            if self.shot_limit_uses_save_queue and save_queued:
                self.shots_received += 1
        elif now - self._last_save_disabled_log_at >= 2.0:
            self.log("Save frames is OFF; received frame not saved")
            self._last_save_disabled_log_at = now
        if not self.shot_limit_uses_save_queue:
            self.shots_received += 1

        output_mask = output_mask_from_channel_mask(frame_settings["channel_mask"])
        mask_enabled = (int(actual_output_cfg) & 0x4) != 0

        if self.active_plot_enabled:
            display_channels = self._channels_for_display(
                channels_raw,
                frame_settings,
                pl_corrected=pl_corrected,
            )
            y_ranges: list[tuple[float, float] | None] = []
            for idx, (curve, channel) in enumerate(zip(self.curves, display_channels)):
                channel_enabled = (not mask_enabled) or ((output_mask & (1 << idx)) != 0)
                curve.setVisible(channel_enabled)
                if channel_enabled:
                    x, y = downsample_for_plot(channel)
                    curve.setData(x, y)
                    y_ranges.append(self._auto_y_range(channel))
                else:
                    curve.setData([], [])
                    y_ranges.append(None)

            self._lock_plot_ranges(
                waveform_words.size,
                pl_corrected_override=pl_corrected,
                y_ranges=y_ranges,
            )
        self._update_footer_panel(footer_words)
        decode_info = {
            "gui_output_cfg": gui_output_cfg,
            "actual_output_cfg": int(actual_output_cfg),
            "decode_mode": "signed16 corrected" if pl_corrected else "raw12 packed",
            "display_mode": self.mode.currentText(),
            "total_words": int(frame.raw_words.size),
            "waveform_words": int(waveform_words.size),
            "footer_expected": True,
            "footer_present": footer_words is not None,
            "mask_enabled": mask_enabled,
            "output_mask": output_mask,
        }
        self._log_decode_warning(decode_info)
        self._update_stats(frame, int(waveform_words.size), footer_words, frame_settings, decode_info)
        self.update_frame_rate_stat(now)
        if (
            self.active_shot_limit > 0 and
            self.shots_received >= self.active_shot_limit and
            not self._shot_limit_stop_pending
        ):
            self._shot_limit_stop_pending = True
            if self.receiver:
                self.receiver.stop()
            QtCore.QTimer.singleShot(0, self.stop_udp_after_shot_limit)

    def _log_decode_warning(self, decode_info: dict) -> None:
        messages = []
        if decode_info["footer_expected"] and not decode_info["footer_present"]:
            messages.append("footer expected by GUI but not found in received frame")
        if decode_info["footer_present"] and decode_info["gui_output_cfg"] != decode_info["actual_output_cfg"]:
            messages.append(
                f"GUI output_cfg=0x{decode_info['gui_output_cfg']:02X}, "
                f"footer output_cfg=0x{decode_info['actual_output_cfg']:02X}"
            )

        warning = "; ".join(messages)
        if warning and warning != self._last_decode_warning:
            self.log(f"Frame decode warning: {warning}")
            self._last_decode_warning = warning

    def _update_stats(
        self,
        frame: Frame,
        waveform_word_count: int,
        footer_words: np.ndarray | None = None,
        settings: dict | None = None,
        decode_info: dict | None = None,
    ) -> None:
        self._set_stat("frame_id", str(frame.frame_id), 0)
        self._set_stat("received_at", datetime.fromtimestamp(frame.received_at).strftime("%H:%M:%S.%f")[:-3], 1)
        self._set_stat("packets", str(frame.packets), 2)
        self._set_stat("bytes", str(frame.bytes_received), 3)
        self._set_stat("dropped", str(frame.dropped_packets), 4)
        self._set_stat("waveform words", str(waveform_word_count), 5)
        self._set_stat("save", "ON" if self.save_enable.isChecked() else "OFF", 6)
        self._set_stat("footer metadata", "Present" if footer_words is not None else "Not present", 7)
        self._set_stat("incomplete", str(frame.incomplete_frames), 8)
        self._set_stat("plot", "ON" if self.active_plot_enabled else "OFF", 13)
        if footer_words is not None and footer_words.size >= 16:
            event_word = int(footer_words[10])
            ignored_trigger_count = (event_word >> 16) & 0xFFFF
            busy_trigger_count = event_word & 0xFFFF
            if footer_words.size >= 17:
                loss_word = int(footer_words[16])
                ignored_trigger_count = (loss_word >> 48) & 0xFFFF
                busy_trigger_count = (loss_word >> 32) & 0xFFFF
                stopped_trigger_count = (loss_word >> 16) & 0xFFFF
                fifo_trigger_count = loss_word & 0xFFFF
                self._set_stat(
                    "busy / ignored",
                    (
                        f"{busy_trigger_count} / {ignored_trigger_count} "
                        f"(stopped {stopped_trigger_count}, fifo {fifo_trigger_count}; overlap)"
                    ),
                    14,
                )
            else:
                self._set_stat("busy / ignored", f"{busy_trigger_count} / {ignored_trigger_count}", 14)
        else:
            self._set_stat("busy / ignored", "-", 14)
        self._set_stat(
            "shots",
            (
                f"{self.shots_received} / {self.active_shot_limit}"
                if self.active_shot_limit > 0 else
                f"{self.shots_received} / continuous"
            ),
            12,
        )

        if decode_info is not None:
            self._set_stat("PL decode", str(decode_info["decode_mode"]), 9)

    def _set_stat(self, key: str, value: str, row: int | None = None, right: bool = False) -> None:
        if row is None:
            row = 0
        table_rows = max(1, self.stats.rowCount())
        col = 2 if right or row >= table_rows else 0
        row = row % table_rows
        self.stats.setItem(row, col, QtWidgets.QTableWidgetItem(key))
        self.stats.setItem(row, col + 1, QtWidgets.QTableWidgetItem(value))

    def log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{stamp}] {message}")

    def closeEvent(self, event) -> None:
        self.stop_udp()
        self.disable_pl_acquisition_on_exit()
        self.saver.stop()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(description="Digitizer UDP frame receiver GUI")
    parser.add_argument("--bind-ip", default=DEFAULT_BIND_IP)
    parser.add_argument("--data-port", type=int, default=DEFAULT_DATA_PORT)
    parser.add_argument("--control-ip", default=DEFAULT_CONTROL_IP)
    parser.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    parser.add_argument("--frame-words", type=int, default=DEFAULT_FRAME_WORDS)
    parser.add_argument("--pretrigger-words", type=int, default=DEFAULT_PRETRIGGER_WORDS)
    parser.add_argument("--sample-decimation", type=int, default=1)
    parser.add_argument("--trigger-cfg", type=int, default=0x1)
    parser.add_argument("--self-threshold", type=int, default=1680)
    parser.add_argument("--channel-mask", type=int, default=0x1)
    parser.add_argument(
        "--output-cfg",
        type=int,
        default=OUTPUT_CFG_APPEND_FOOTER,
    )
    parser.add_argument("--baseline-shift", type=int, default=10)
    parser.add_argument("--mode", type=int, default=1)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--offset-mv", type=float, default=0.0)
    parser.add_argument("--fullscale-mv", type=float, default=2000.0)
    parser.add_argument("--display-fps", type=float, default=10.0)
    parser.add_argument("--output-dir", default=str(Path.cwd() / "captures"))
    parser.add_argument(
        "--save-format",
        choices=["raw-stream", "raw-bin", "npz", "npz-compressed"],
        default="raw-stream",
    )
    parser.add_argument("--shot-limit", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())

