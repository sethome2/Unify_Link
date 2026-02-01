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
        self.success_history: list[int] = []
        self.max_points = 200

        self.status_var = tk.StringVar(value="Waiting for data...")
        self.set_var = tk.StringVar(value="0")
        self.success_var = tk.StringVar(value="0")

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

        ttk.Label(self.root, textvariable=self.status_var, padding=10).pack(fill=tk.X)

        fig = Figure(figsize=(6, 3), dpi=100)
        self.ax_set = fig.add_subplot(111)
        self.ax_set.set_title("Motor Setpoint (ID 0)")
        self.ax_set.set_xlabel("Samples")
        self.ax_set.set_ylabel("Set")
        self.set_line, = self.ax_set.plot([], [], color="#1f77b4")

        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.canvas = canvas

    def update_data(self) -> None:
        set_value = self.motors.motor_set[0].set
        success = self.base.success_count

        self.set_var.set(str(set_value))
        self.success_var.set(str(success))

        self.set_history.append(float(set_value))
        self.success_history.append(int(success))

        if len(self.set_history) > self.max_points:
            self.set_history = self.set_history[-self.max_points :]

        self.set_line.set_data(range(len(self.set_history)), self.set_history)
        self.ax_set.relim()
        self.ax_set.autoscale_view()
        self.canvas.draw_idle()

    def set_status(self, text: str) -> None:
        self.status_var.set(text)


def serial_reader(ser: serial.Serial, base: ul.UnifyLinkBase, stop_event: threading.Event,
                  ui_queue: queue.Queue) -> None:
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                base.rev_data_push(data)
                base.parse_data_task()
                ui_queue.put("update")
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
    ui_queue: queue.Queue = queue.Queue()

    reader_thread = threading.Thread(
        target=serial_reader, args=(ser, base, stop_event, ui_queue), daemon=True
    )
    reader_thread.start()

    def poll_queue() -> None:
        try:
            while True:
                msg = ui_queue.get_nowait()
                if msg == "update":
                    ui.update_data()
                elif isinstance(msg, str) and msg.startswith("error:"):
                    ui.set_status(msg)
        except queue.Empty:
            pass
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
