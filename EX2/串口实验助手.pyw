import csv
import os
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


def import_pyserial():
    try:
        import serial
        from serial.tools import list_ports
        return serial, list_ports
    except ModuleNotFoundError:
        appdata = Path(os.environ.get("APPDATA", ""))
        candidates = sorted((appdata / "Python").glob("Python*/site-packages"), reverse=True)
        for candidate in candidates:
            sys.path.insert(0, str(candidate))
            try:
                import serial
                from serial.tools import list_ports
                return serial, list_ports
            except ModuleNotFoundError:
                continue
        raise


try:
    import tkinter as tk
    from tkinter import messagebox, ttk
    serial, list_ports = import_pyserial()
except Exception as exc:
    import tkinter.messagebox as messagebox
    messagebox.showerror("启动失败", f"无法加载串口组件：{exc}")
    raise


CSV_HEADER = [
    "record", "ms", "mode", "target_left", "target_right",
    "speed_left_mm_s", "speed_right_mm_s", "pwm_left", "pwm_right",
    "raw_left", "raw_right", "total_left", "total_right", "enable"
]


class SerialAssistant(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("B585 STM32F103RCT6 串口实验助手")
        self.geometry("1120x720")
        self.minsize(900, 600)

        self.serial_port = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.rx_queue = queue.Queue()
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        self.port_var = tk.StringVar(value="COM7")
        self.baud_var = tk.StringVar(value="115200")
        self.command_var = tk.StringVar(value="STATUS")
        self.status_var = tk.StringVar(value="未连接")
        self.log_path_var = tk.StringVar(value="尚未创建 CSV")

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._drain_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=13, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(5, 12))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="波特率").pack(side=tk.LEFT, padx=(18, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="连接", command=self.connect).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(top, text="断开", command=self.disconnect).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_var, foreground="#0b5").pack(side=tk.LEFT, padx=18)

        safety = ttk.LabelFrame(self, text="安全与基础操作", padding=8)
        safety.pack(fill=tk.X, padx=10, pady=(0, 8))
        for label, command in [
            ("STATUS", "STATUS"), ("ARM 解锁", "ARM"), ("STOP 急停", "STOP"),
            ("ZERO 清零", "ZERO"), ("ENC 计数", "ENC"),
            ("LOG 开", "LOG 1"), ("LOG 关", "LOG 0"), ("HELP", "HELP")
        ]:
            style = "Danger.TButton" if command == "STOP" else "TButton"
            ttk.Button(safety, text=label, style=style,
                       command=lambda value=command: self.send_command(value)).pack(side=tk.LEFT, padx=4)

        experiment = ttk.LabelFrame(self, text="实验命令", padding=8)
        experiment.pack(fill=tk.X, padx=10, pady=(0, 8))
        for label, command in [
            ("低速试转 800", "OPEN 800 800"),
            ("五档 SWEEP", "SWEEP"),
            ("PI 组1", "P1"), ("PI 组2", "P2"), ("PI 组3", "P3"),
            ("目标 200 mm/s", "SPEED 200 200"),
            ("CPR=60000（默认）", "CPR 60000"),
            ("CPR=62000（左轮实测）", "CPR 62000")
        ]:
            ttk.Button(experiment, text=label,
                       command=lambda value=command: self.send_command(value)).pack(side=tk.LEFT, padx=4)

        custom = ttk.Frame(self, padding=(10, 0, 10, 8))
        custom.pack(fill=tk.X)
        ttk.Label(custom, text="自定义命令").pack(side=tk.LEFT)
        entry = ttk.Entry(custom, textvariable=self.command_var)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        entry.bind("<Return>", lambda _event: self.send_command(self.command_var.get()))
        ttk.Button(custom, text="发送（自动 CRLF）",
                   command=lambda: self.send_command(self.command_var.get())).pack(side=tk.LEFT)

        log_frame = ttk.LabelFrame(self, text="串口接收", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))
        self.text = tk.Text(log_frame, wrap=tk.NONE, font=("Consolas", 10), bg="#101418", fg="#d7e0e7")
        yscroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.text.yview)
        xscroll = ttk.Scrollbar(log_frame, orient=tk.HORIZONTAL, command=self.text.xview)
        self.text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        bottom = ttk.Frame(self, padding=(10, 0, 10, 10))
        bottom.pack(fill=tk.X)
        ttk.Label(bottom, textvariable=self.log_path_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(bottom, text="清空窗口", command=lambda: self.text.delete("1.0", tk.END)).pack(side=tk.RIGHT)

        style = ttk.Style(self)
        style.configure("Danger.TButton", foreground="#b00020")

    def refresh_ports(self):
        ports = list(list_ports.comports())
        values = [port.device for port in ports]
        self.port_combo["values"] = values
        if "COM7" in values:
            self.port_var.set("COM7")
        elif values:
            self.port_var.set(values[0])
        else:
            self.port_var.set("")
        descriptions = ", ".join(f"{p.device}={p.description}" for p in ports) or "未检测到串口"
        self._append_text(f"[端口] {descriptions}\n")

    def connect(self):
        if self.serial_port and self.serial_port.is_open:
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("连接失败", "未检测到可用串口。请连接小车 Type-C 数据线后刷新。")
            return
        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.serial_port.dtr = False
            self.serial_port.rts = False
            self.stop_event.clear()
            self._open_csv()
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            self.status_var.set(f"已连接 {port} @ {baud}")
            self._append_text(f"[连接] {port} @ {baud}, 8N1\n")
            self.after(300, lambda: self.send_command("STATUS"))
        except Exception as exc:
            self.serial_port = None
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self):
        self.stop_event.set()
        if self.serial_port:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        self.status_var.set("未连接")
        self._append_text("[断开]\n")

    def _open_csv(self):
        log_dir = Path(__file__).resolve().parent / "实验数据"
        log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = log_dir / f"B585_COM7_{timestamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()
        self.log_path_var.set(f"CSV：{self.csv_path}")

    def send_command(self, command):
        command = command.strip()
        if not command:
            return
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("尚未连接", "请先连接串口。")
            return
        try:
            self.serial_port.write((command + "\r\n").encode("ascii"))
            self.serial_port.flush()
            self._append_text(f">>> {command}\n")
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))
            self.disconnect()

    def _reader_loop(self):
        pending = bytearray()
        while not self.stop_event.is_set() and self.serial_port and self.serial_port.is_open:
            try:
                data = self.serial_port.read(4096)
                if not data:
                    continue
                pending.extend(data)
                while b"\n" in pending:
                    raw, _, pending = pending.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("utf-8", errors="replace")
                    self.rx_queue.put(line)
            except Exception as exc:
                self.rx_queue.put(f"[接收错误] {exc}")
                break

    def _drain_rx_queue(self):
        processed = 0
        while processed < 500:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._append_text(line + "\n")
            if line.startswith("D,") and self.csv_writer:
                fields = line.split(",")
                if len(fields) == len(CSV_HEADER):
                    self.csv_writer.writerow(fields)
                    if int(fields[1]) % 500 == 0:
                        self.csv_file.flush()
            processed += 1
        self.after(50, self._drain_rx_queue)

    def _append_text(self, value):
        self.text.insert(tk.END, value)
        self.text.see(tk.END)

    def on_close(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\r\n")
                self.serial_port.flush()
                time.sleep(0.05)
            except Exception:
                pass
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    SerialAssistant().mainloop()
