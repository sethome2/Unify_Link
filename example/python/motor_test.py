import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

import importlib

import unify_link as ul

if not hasattr(ul, "UnifyLinkBase"):
    ul = importlib.import_module("unify_link.unify_link")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

PRINT_RX = True
MAX_PAYLOAD_HEX = 128


def choose_serial_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"[{idx}] {p.device} - {p.description}")

    while True:
        choice = input("Select port index: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        print("Invalid selection, try again.")


class MotorMonitorUI:
    def __init__(self, root: tk.Tk, base: ul.UnifyLinkBase, motors: ul.MotorLink) -> None:
        self.root = root
        self.base = base
        self.motors = motors

        self.set_history: list[float] = []
        self.position_history: list[float] = []
        self.speed_history: list[float] = []
        self.current_history: list[float] = []
        self.temperature_history: list[float] = []
        self.success_history: list[int] = []
        self.window_size = 200
        self.sample_index = 0

        self.status_var = tk.StringVar(value="Waiting for data...")
        self.set_var = tk.StringVar(value="0")
        self.success_var = tk.StringVar(value="0")
        self.com_error_var = tk.StringVar(value="0")
        self.decode_error_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="0.0 B/s")
        self.position_var = tk.StringVar(value="0")
        self.speed_var = tk.StringVar(value="0")
        self.current_var = tk.StringVar(value="0")
        self.temperature_var = tk.StringVar(value="0")
        self.error_code_var = tk.StringVar(value="0")

        self._last_rate_time = time.monotonic()
        self._bytes_since_rate = 0
        self._last_draw_time = time.monotonic()
        self._draw_interval = 0.1

        self._build_ui()

    def _build_ui(self) -> None:
        self.root.title("UnifyLink Motor Monitor")

        header = ttk.Frame(self.root, padding=10)
        header.pack(fill=tk.X)

        ttk.Label(header, text="Motor ID: 0").pack(side=tk.LEFT)
        ttk.Label(header, text="Set:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.set_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Success:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.success_var).pack(side=tk.LEFT)
        ttk.Label(header, text="ComErr:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.com_error_var).pack(side=tk.LEFT)
        ttk.Label(header, text="DecodeErr:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.decode_error_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Rate:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.rate_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Pos:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.position_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Speed:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.speed_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Current:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.current_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Temp:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.temperature_var).pack(side=tk.LEFT)
        ttk.Label(header, text="Err:").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Label(header, textvariable=self.error_code_var).pack(side=tk.LEFT)

        ttk.Label(self.root, textvariable=self.status_var, padding=10).pack(fill=tk.X)

        fig = Figure(figsize=(7, 4.5), dpi=100)
        self.ax_all = fig.add_subplot(111)
        self.ax_all.set_title("Motor Signals (ID 0)")
        self.ax_all.set_xlabel("Samples")
        self.ax_all.set_ylabel("Value")
        self.set_line, = self.ax_all.plot([], [], color="#1f77b4", alpha=0.7, label="set")
        self.position_line, = self.ax_all.plot([], [], color="#2ca02c", alpha=0.7, label="position")
        self.speed_line, = self.ax_all.plot([], [], color="#ff7f0e", alpha=0.7, label="speed")
        self.current_line, = self.ax_all.plot([], [], color="#d62728", alpha=0.7, label="current")
        self.temperature_line, = self.ax_all.plot([], [], color="#9467bd", alpha=0.7, label="temperature")
        self.ax_all.legend(loc="upper right")

        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.canvas = canvas

    def update_data(self, bytes_read: int) -> None:
        set_value = self.motors.motor_set[0].set
        basic = self.motors.motor_basic[0]
        position = basic.position
        speed = basic.speed
        current = basic.current
        temperature = basic.temperature
        success = self.base.success_count
        com_error = self.base.com_error_count
        decode_error = self.base.decode_error_count

        self.set_var.set(str(set_value))
        self.success_var.set(str(success))
        self.com_error_var.set(str(com_error))
        self.decode_error_var.set(str(decode_error))
        self.position_var.set(str(position))
        self.speed_var.set(str(speed))
        self.current_var.set(str(current))
        self.temperature_var.set(str(temperature))
        self.error_code_var.set(str(int(basic.error_code)))

        self._bytes_since_rate += bytes_read
        now = time.monotonic()
        elapsed = now - self._last_rate_time
        if elapsed >= 1.0:
            rate = self._bytes_since_rate / elapsed
            self.rate_var.set(f"{rate:.1f} B/s")
            self._bytes_since_rate = 0
            self._last_rate_time = now

        self.set_history.append(float(set_value))
        self.position_history.append(float(position))
        self.speed_history.append(float(speed))
        self.current_history.append(float(current))
        self.temperature_history.append(float(temperature))
        self.success_history.append(int(success))

        if len(self.set_history) > self.window_size:
            self.set_history = self.set_history[-self.window_size :]
            self.position_history = self.position_history[-self.window_size :]
            self.speed_history = self.speed_history[-self.window_size :]
            self.current_history = self.current_history[-self.window_size :]
            self.temperature_history = self.temperature_history[-self.window_size :]

        self.sample_index += 1
        start_index = max(0, self.sample_index - len(self.set_history) + 1)
        x = range(start_index, start_index + len(self.set_history))
        self.set_line.set_data(x, self.set_history)
        self.position_line.set_data(x, self.position_history)
        self.speed_line.set_data(x, self.speed_history)
        self.current_line.set_data(x, self.current_history)
        self.temperature_line.set_data(x, self.temperature_history)
        self.ax_all.relim()
        self.ax_all.autoscale_view()
        if len(self.set_history) >= 2:
            self.ax_all.set_xlim(start_index, start_index + self.window_size)

        now = time.monotonic()
        if now - self._last_draw_time >= self._draw_interval:
            self.canvas.draw_idle()
            self._last_draw_time = now

    def set_status(self, text: str) -> None:
        self.status_var.set(text)


def serial_reader(ser: serial.Serial, base: ul.UnifyLinkBase, stop_event: threading.Event,
                  ui_queue: queue.Queue) -> None:
    frame_buffer = bytearray()
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                if PRINT_RX:
                    frame_buffer.extend(data)
                    while True:
                        idx = frame_buffer.find(bytes([ul.FRAME_HEADER]))
                        if idx == -1:
                            frame_buffer.clear()
                            break
                        if idx > 0:
                            del frame_buffer[:idx]
                        if len(frame_buffer) < 8:
                            break
                        payload_len = int.from_bytes(frame_buffer[4:6], "little") & 0x1FFF
                        frame_len = 8 + payload_len
                        if len(frame_buffer) < frame_len:
                            break
                        frame = bytes(frame_buffer[:frame_len])
                        del frame_buffer[:frame_len]
                        comp_id = frame[2]
                        data_id = frame[3]
                        payload = frame[8:]
                        hex_payload = payload.hex()
                        if len(hex_payload) > MAX_PAYLOAD_HEX:
                            hex_payload = hex_payload[:MAX_PAYLOAD_HEX] + "..."
                        print(f"RX comp={comp_id} data={data_id} len={payload_len} payload={hex_payload}")
                base.rev_data_push(data)
                base.parse_data_task()
                try:
                    ui_queue.put_nowait(len(data))
                except queue.Full:
                    pass
        except serial.SerialException as exc:
            ui_queue.put(f"error:{exc}")
            break


def main() -> None:
    port = choose_serial_port()
    ser = serial.Serial(port, 921600, timeout=0.05)

    base = ul.UnifyLinkBase()
    motors = ul.MotorLink(base)
    _enc = ul.EncoderLink(base)

    root = tk.Tk()
    ui = MotorMonitorUI(root, base, motors)
    ui.set_status(f"Connected: {port} @ 921600")

    stop_event = threading.Event()
    ui_queue: queue.Queue = queue.Queue(maxsize=200)

    reader_thread = threading.Thread(
        target=serial_reader, args=(ser, base, stop_event, ui_queue), daemon=True
    )
    reader_thread.start()

    def poll_queue() -> None:
        total_bytes = 0
        try:
            while True:
                msg = ui_queue.get_nowait()
                if isinstance(msg, int):
                    total_bytes += msg
                elif isinstance(msg, str) and msg.startswith("error:"):
                    ui.set_status(msg)
        except queue.Empty:
            pass
        if total_bytes:
            ui.update_data(total_bytes)
        root.after(50, poll_queue)

    def on_close() -> None:
        stop_event.set()
        try:
            ser.close()
        except serial.SerialException:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    poll_queue()
    root.mainloop()


if __name__ == "__main__":
    main()
