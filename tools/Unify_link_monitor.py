import threading
import time
import tkinter as tk
from collections import defaultdict, deque
from dataclasses import dataclass
from tkinter import ttk

import importlib

import serial
from serial.tools import list_ports

import unify_link as ul

if not hasattr(ul, "UnifyLinkBase"):
    ul = importlib.import_module("unify_link.unify_link")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD_RATES = [
    "9600",
    "19200",
    "38400",
    "57600",
    "115200",
    "230400",
    "460800",
    "921600",
    "1000000",
    "1500000",
    "2000000",
]

FRAME_HEADER = int(ul.FRAME_HEADER)


@dataclass
class FrameInfo:
    comp_id: int
    data_id: int
    ts: float
    length: int


class StreamStats:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.rx_bytes_total = 0
        self.tx_bytes_total = 0
        self.rx_counts = defaultdict(int)
        self.tx_counts = defaultdict(int)
        self.rx_times: dict[tuple[int, int], deque[float]] = defaultdict(deque)
        self.tx_times: dict[tuple[int, int], deque[float]] = defaultdict(deque)

    def add_rx(self, byte_count: int, frames: list[FrameInfo]) -> None:
        with self._lock:
            self.rx_bytes_total += byte_count
            for info in frames:
                key = (info.comp_id, info.data_id)
                self.rx_counts[key] += 1
                self.rx_times[key].append(info.ts)

    def add_tx(self, byte_count: int, comp_id: int, data_id: int) -> None:
        ts = time.monotonic()
        with self._lock:
            self.tx_bytes_total += byte_count
            key = (comp_id, data_id)
            self.tx_counts[key] += 1
            self.tx_times[key].append(ts)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "rx_bytes_total": self.rx_bytes_total,
                "tx_bytes_total": self.tx_bytes_total,
                "rx_counts": dict(self.rx_counts),
                "tx_counts": dict(self.tx_counts),
                "rx_times": {k: deque(v) for k, v in self.rx_times.items()},
                "tx_times": {k: deque(v) for k, v in self.tx_times.items()},
            }


def parse_frames(buffer: bytearray) -> list[FrameInfo]:
    frames: list[FrameInfo] = []
    while True:
        idx = buffer.find(bytes([FRAME_HEADER]))
        if idx == -1:
            buffer.clear()
            break
        if idx > 0:
            del buffer[:idx]
        if len(buffer) < 8:
            break
        payload_len = int.from_bytes(buffer[4:6], "little") & 0x1FFF
        frame_len = 8 + payload_len
        if len(buffer) < frame_len:
            break
        comp_id = buffer[2]
        data_id = buffer[3]
        frames.append(
            FrameInfo(comp_id=comp_id, data_id=data_id, ts=time.monotonic(), length=payload_len)
        )
        del buffer[:frame_len]
    return frames


class UnifyLinkMonitor:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.base = ul.UnifyLinkBase()
        self.stats = StreamStats()
        self.ser: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        self._last_rate_time = time.monotonic()
        self._bytes_since_rate = 0
        self._current_rate_kb = 0.0
        self._last_rx_total = 0
        self._reader_error: str | None = None

        self._history_span = 120.0
        self._speed_span = 60.0
        self._history_times: deque[float] = deque()
        self._success_history: deque[int] = deque()
        self._comerr_history: deque[int] = deque()
        self._decode_history: deque[int] = deque()
        self._speed_times: deque[float] = deque()
        self._speed_history: deque[float] = deque()

        self.status_var = tk.StringVar(value="Disconnected")
        self.success_var = tk.StringVar(value="0")
        self.com_error_var = tk.StringVar(value="0")
        self.decode_error_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="0.00 KB/s")
        self.rx_bytes_var = tk.StringVar(value="0")

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="921600")
        self.comp_var = tk.StringVar(value="1")
        self.data_var = tk.StringVar(value="1")
        self.payload_var = tk.StringVar(value="")

        self._build_ui()
        self._register_all_ids()
        self._refresh_ports()
        self._schedule_update()

    def _build_ui(self) -> None:
        self.root.title("UnifyLink Monitor")
        self.root.geometry("1200x720")

        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=(10, 8))
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        top.columnconfigure(8, weight=1)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 10))
        ttk.Label(top, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_combo = ttk.Combobox(top, textvariable=self.baud_var, values=BAUD_RATES, width=10)
        self.baud_combo.grid(row=0, column=3, padx=(4, 10))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=4)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, padx=(10, 0))

        ttk.Label(top, textvariable=self.status_var, foreground="#2f3e4d").grid(
            row=0, column=8, sticky="e", padx=(10, 0)
        )

        stats = ttk.Frame(top)
        stats.grid(row=1, column=0, columnspan=9, sticky="ew", pady=(8, 0))

        ttk.Label(stats, text="Success:").pack(side=tk.LEFT)
        ttk.Label(stats, textvariable=self.success_var).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(stats, text="ComErr:").pack(side=tk.LEFT)
        ttk.Label(stats, textvariable=self.com_error_var).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(stats, text="DecodeErr:").pack(side=tk.LEFT)
        ttk.Label(stats, textvariable=self.decode_error_var).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(stats, text="Speed:").pack(side=tk.LEFT)
        ttk.Label(stats, textvariable=self.rate_var).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(stats, text="RX Bytes:").pack(side=tk.LEFT)
        ttk.Label(stats, textvariable=self.rx_bytes_var).pack(side=tk.LEFT, padx=(4, 12))

        left = ttk.Frame(self.root, padding=(10, 8))
        left.grid(row=1, column=0, sticky="nsew")
        left.rowconfigure(1, weight=1)
        left.rowconfigure(2, weight=0)
        left.columnconfigure(0, weight=1)

        send_box = ttk.LabelFrame(left, text="Send UL Message", padding=10)
        send_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        send_box.columnconfigure(6, weight=1)

        ttk.Label(send_box, text="Comp:").grid(row=0, column=0, sticky="w")
        ttk.Entry(send_box, textvariable=self.comp_var, width=6).grid(row=0, column=1, padx=(4, 10))
        ttk.Label(send_box, text="Data:").grid(row=0, column=2, sticky="w")
        ttk.Entry(send_box, textvariable=self.data_var, width=6).grid(row=0, column=3, padx=(4, 10))
        ttk.Label(send_box, text="Payload(hex):").grid(row=0, column=4, sticky="w")
        ttk.Entry(send_box, textvariable=self.payload_var).grid(row=0, column=5, sticky="ew", padx=(4, 10))
        ttk.Button(send_box, text="Send", command=self._send_message).grid(row=0, column=6)

        fig = Figure(figsize=(7.2, 5.4), dpi=100, constrained_layout=True)
        gs = fig.add_gridspec(2, 1, height_ratios=[3, 1])
        self.ax_main = fig.add_subplot(gs[0])
        self.ax_speed = fig.add_subplot(gs[1])

        self.ax_main.set_title("UnifyLink Stats")
        self.ax_main.set_xlabel("Time (s)")
        self.ax_main.set_ylabel("Count")
        self.main_success_line, = self.ax_main.plot([], [], color="#2ca02c", label="success")
        self.main_comerr_line, = self.ax_main.plot([], [], color="#d62728", label="com error")
        self.main_decode_line, = self.ax_main.plot([], [], color="#ff7f0e", label="decode error")
        self.ax_main.legend(loc="upper left")

        self.ax_speed.set_title("Speed (Last 60s)")
        self.ax_speed.set_xlabel("Time (s)")
        self.ax_speed.set_ylabel("KB/s")
        self.speed_line, = self.ax_speed.plot([], [], color="#1f77b4")

        self.canvas = FigureCanvasTkAgg(fig, master=left)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", pady=(0, 8))

        right = ttk.Frame(self.root, padding=(10, 8))
        right.grid(row=1, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        ttk.Label(right, text="Data Stream Stats", font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 6)
        )

        columns = ("comp", "data", "hz", "count")
        self.stats_view = ttk.Treeview(right, columns=columns, show="headings", height=18)
        self.stats_view.heading("comp", text="Comp")
        self.stats_view.heading("data", text="DataID")
        self.stats_view.heading("hz", text="Hz")
        self.stats_view.heading("count", text="Count")
        self.stats_view.column("comp", width=60, anchor="center")
        self.stats_view.column("data", width=70, anchor="center")
        self.stats_view.column("hz", width=70, anchor="center")
        self.stats_view.column("count", width=80, anchor="center")
        self.stats_view.grid(row=1, column=0, sticky="nsew")

        self.right_status = ttk.Label(right, text="TX/RX bytes are tracked separately.")
        self.right_status.grid(row=2, column=0, sticky="w", pady=(6, 0))

    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _register_all_ids(self) -> None:
        for comp_id in range(256):
            for data_id in range(256):
                self.base.register_any_payload(comp_id, data_id)

    def _toggle_connect(self) -> None:
        if self.ser:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        baud = self.baud_var.get().strip()
        if not port:
            self.status_var.set("No port selected")
            return
        try:
            baud_rate = int(baud)
        except ValueError:
            self.status_var.set("Invalid baud rate")
            return
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=0.05)
        except serial.SerialException as exc:
            self.status_var.set(f"Connect failed: {exc}")
            self.ser = None
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connect_btn.config(text="Disconnect")
        self.status_var.set(f"Connected: {port} @ {baud_rate}")

    def _disconnect(self) -> None:
        self.stop_event.set()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.connect_btn.config(text="Connect")
        self.status_var.set("Disconnected")

    def _reader_loop(self) -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            if not self.ser:
                break
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                self._reader_error = "Serial read error"
                break
            if not data:
                continue
            buffer.extend(data)
            frames = parse_frames(buffer)
            self.stats.add_rx(len(data), frames)
            self.base.rev_data_push(data)
            self.base.parse_data_task()

    def _send_message(self) -> None:
        if not self.ser:
            self.status_var.set("Not connected")
            return
        try:
            comp_id = int(self.comp_var.get(), 0)
            data_id = int(self.data_var.get(), 0)
        except ValueError:
            self.status_var.set("Invalid comp/data id")
            return
        payload_text = self.payload_var.get().strip()
        if not payload_text:
            payload = b""
        else:
            cleaned = (
                payload_text.replace("0x", "")
                .replace("0X", "")
                .replace(",", " ")
                .replace(";", " ")
            )
            try:
                payload = bytes.fromhex(cleaned)
            except ValueError:
                self.status_var.set("Payload hex format error")
                return
        built = self.base.build_send_data(comp_id, data_id, payload)
        if built == 0:
            self.status_var.set("Send buffer full or payload too large")
            return
        out = self.base.pop_send_buffer()
        if not out:
            self.status_var.set("Send buffer empty")
            return
        try:
            written = self.ser.write(out)
        except serial.SerialException as exc:
            self.status_var.set(f"Send failed: {exc}")
            return
        self.stats.add_tx(written, comp_id, data_id)
        self.status_var.set(f"Sent {written} bytes (comp={comp_id}, data={data_id})")

    def _schedule_update(self) -> None:
        self._update_ui()
        self.root.after(200, self._schedule_update)

    def _update_ui(self) -> None:
        success = int(self.base.success_count)
        com_err = int(self.base.com_error_count)
        decode_err = int(self.base.decode_error_count)

        now = time.monotonic()
        elapsed = now - self._last_rate_time
        snapshot = self.stats.snapshot()
        delta = snapshot["rx_bytes_total"] - self._last_rx_total
        if delta > 0:
            self._bytes_since_rate += delta
        self._last_rx_total = snapshot["rx_bytes_total"]

        if elapsed >= 0.5:
            rate = self._bytes_since_rate / elapsed
            self._current_rate_kb = rate / 1024.0
            self._bytes_since_rate = 0
            self._last_rate_time = now

        self.success_var.set(str(success))
        self.com_error_var.set(str(com_err))
        self.decode_error_var.set(str(decode_err))
        self.rate_var.set(f"{self._current_rate_kb:.2f} KB/s")
        self.rx_bytes_var.set(str(snapshot["rx_bytes_total"]))
        if self._reader_error:
            self.status_var.set(self._reader_error)
            self._reader_error = None

        self._history_times.append(now)
        self._success_history.append(success)
        self._comerr_history.append(com_err)
        self._decode_history.append(decode_err)

        while self._history_times and now - self._history_times[0] > self._history_span:
            self._history_times.popleft()
            self._success_history.popleft()
            self._comerr_history.popleft()
            self._decode_history.popleft()

        self._speed_times.append(now)
        self._speed_history.append(self._current_rate_kb)
        while self._speed_times and now - self._speed_times[0] > self._speed_span:
            self._speed_times.popleft()
            self._speed_history.popleft()

        t0 = self._history_times[0] if self._history_times else now
        times = [t - t0 for t in self._history_times]
        self.main_success_line.set_data(times, list(self._success_history))
        self.main_comerr_line.set_data(times, list(self._comerr_history))
        self.main_decode_line.set_data(times, list(self._decode_history))
        self.ax_main.relim()
        self.ax_main.autoscale_view()
        if times:
            self.ax_main.set_xlim(max(0, times[-1] - self._history_span), times[-1])

        t0_speed = self._speed_times[0] if self._speed_times else now
        speed_times = [t - t0_speed for t in self._speed_times]
        self.speed_line.set_data(speed_times, list(self._speed_history))
        self.ax_speed.relim()
        self.ax_speed.autoscale_view()
        if speed_times:
            self.ax_speed.set_xlim(max(0, speed_times[-1] - self._speed_span), speed_times[-1])

        self._update_stats_view(snapshot, now)

        self.canvas.draw_idle()

    def _update_stats_view(self, snapshot: dict, now: float) -> None:
        for item in self.stats_view.get_children():
            self.stats_view.delete(item)

        rows: list[tuple[int, int, float, int]] = []
        for key, count in snapshot["rx_counts"].items():
            times = snapshot["rx_times"].get(key, deque())
            while times and now - times[0] > 1.0:
                times.popleft()
            hz = float(len(times))
            rows.append((key[0], key[1], hz, count))

        rows.sort(key=lambda r: (-r[2], -r[3], r[0], r[1]))
        for comp_id, data_id, hz, count in rows[:40]:
            self.stats_view.insert("", tk.END, values=(comp_id, data_id, f"{hz:.1f}", count))

        tx_total = snapshot["tx_bytes_total"]
        rx_total = snapshot["rx_bytes_total"]
        self.right_status.config(text=f"RX bytes: {rx_total} | TX bytes: {tx_total}")


def main() -> None:
    root = tk.Tk()
    app = UnifyLinkMonitor(root)

    def on_close() -> None:
        app._disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
