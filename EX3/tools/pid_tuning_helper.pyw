import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox


DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200


COMMAND_SETS = {
    "安全停止": ["STOP", "STATUS"],
    "状态查询": ["STATUS"],
    "ARM使能": ["ARM", "STATUS"],
    "1_Kp12000": ["STOP", "PID 12000 0 0 0", "STATUS"],
    "2_Kp20000": ["STOP", "PID 20000 0 0 0", "STATUS"],
    "3_Kp27000": ["STOP", "PID 27000 0 0 0", "STATUS"],
    "4_Kd60": ["STOP", "PID 27000 60 0 0", "STATUS"],
    "5_Kd110": ["STOP", "PID 27000 110 0 0", "STATUS"],
    "6_VK200": ["STOP", "PID 27000 110 200 0", "STATUS"],
    "7_VK400": ["STOP", "PID 27000 110 400 0", "STATUS"],
    "8_Final_Ki2": ["STOP", "PID 27000 110 400 2", "STATUS"],
}


HOTKEYS = [
    ("F1", "安全停止"),
    ("F3", "1_Kp12000"),
    ("F4", "2_Kp20000"),
    ("F5", "3_Kp27000"),
    ("F6", "4_Kd60"),
    ("F7", "5_Kd110"),
    ("F8", "6_VK200"),
    ("F9", "7_VK400"),
    ("F10", "8_Final_Ki2"),
    ("F11", "ARM使能"),
    ("F12", "状态查询"),
]


NOTES = {
    "1_Kp12000": "只开角度环 P，Kp 偏小；观察车身软、轮子扶正力度不足。",
    "2_Kp20000": "只增大角度 Kp；支撑增强，但仍缺少角速度阻尼。",
    "3_Kp27000": "继续只增大角度 Kp；纠偏强，但可能过冲或抖动。",
    "4_Kd60": "固定 Kp=27000，只加入 Kd=60；观察抖动是否减小。",
    "5_Kd110": "固定 Kp=27000，只增大 Kd=110；角度环基本成型，但仍可能前后漂移。",
    "6_VK200": "固定角度环，只加入速度环 Kp=200；观察持续跑偏是否减轻。",
    "7_VK400": "固定角度环，只增大速度 Kp=400；观察站位是否更稳。",
    "8_Final_Ki2": "最终真实稳定参数：Balance 27000/110，Velocity 400/2；录制 30 秒站立和轻推回位。",
}


class TuningApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("实验三 平衡串口精简版调参助手")
        self.geometry("1160x740")
        self.serial = None
        self.alive = False
        self.rx_queue = queue.Queue()
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.status_var = tk.StringVar(value="未连接")
        self._build_ui()
        self._bind_keys()
        self.after(50, self._drain_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="连接", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="断开", command=self.disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.status_var, foreground="#0066aa").pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self, padding=(8, 0, 8, 8))
        body.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(left, text="快捷键").pack(anchor=tk.W)
        for key, name in HOTKEYS:
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=key, width=5).pack(side=tk.LEFT)
            ttk.Button(row, text=name, width=28,
                       command=lambda n=name: self.run_command_set(n)).pack(side=tk.LEFT)

        ttk.Separator(left).pack(fill=tk.X, pady=10)
        self.manual_var = tk.StringVar()
        ttk.Entry(left, textvariable=self.manual_var, width=34).pack(fill=tk.X, pady=3)
        ttk.Button(left, text="发送手动命令", command=self.send_manual).pack(fill=tk.X, pady=3)
        ttk.Button(left, text="清空窗口", command=lambda: self.log.delete("1.0", tk.END)).pack(fill=tk.X, pady=3)
        note = (
            "正式 8 组流程：F3-F10 每次只改一个变量或一个控制项。"
            "每组先写参数，再扶正小车按 F11 ARM；最后 F10 是当前真实稳定参数。"
        )
        ttk.Label(left, text=note, wraplength=340, foreground="#444").pack(anchor=tk.W, pady=10)

        self.log = tk.Text(right, wrap=tk.WORD, font=("Consolas", 11))
        self.log.pack(fill=tk.BOTH, expand=True)

    def _bind_keys(self):
        for key, name in HOTKEYS:
            self.bind(f"<{key}>", lambda _e, n=name: self.run_command_set(n))
        self.bind("<Escape>", lambda _e: self.run_command_set("安全停止"))

    def _append(self, text):
        self.log.insert(tk.END, text)
        self.log.see(tk.END)

    def connect(self):
        if self.serial is not None:
            return
        try:
            import serial
            self.serial = serial.Serial(self.port_var.get().strip(), int(self.baud_var.get().strip()), timeout=0.05)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.alive = True
            threading.Thread(target=self._reader_loop, daemon=True).start()
            self.status_var.set(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            self._append(f"\n[CONNECT] {self.port_var.get()} @ {self.baud_var.get()}\n")
        except Exception as exc:
            self.serial = None
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self):
        self.alive = False
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.status_var.set("未连接")
        self._append("\n[DISCONNECT]\n")

    def _reader_loop(self):
        while self.alive:
            try:
                data = self.serial.readline()
                if data:
                    text = self._clean_ascii(data)
                    if text.strip():
                        self.rx_queue.put(text)
            except Exception as exc:
                self.rx_queue.put(f"\n[RX_ERROR] {exc}\n")
                break

    def _drain_rx_queue(self):
        while True:
            try:
                text = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._append(text)
        self.after(50, self._drain_rx_queue)

    @staticmethod
    def _clean_ascii(data):
        chars = []
        for value in data:
            if value in (9, 10, 13) or 32 <= value <= 126:
                chars.append(chr(value))
        return "".join(chars)

    def send_line(self, line):
        self.connect()
        if self.serial is None:
            return
        payload = (line.strip() + "\r\n").encode("ascii", errors="ignore")
        try:
            self.serial.write(payload)
            self._append(f">>> {line.strip()}\n")
        except Exception as exc:
            self._append(f"[TX_ERROR] {exc}\n")

    def run_command_set(self, name):
        self._append(f"\n--- {name} ---\n")
        if name in NOTES:
            self._append(f"{NOTES[name]}\n")
        for command in COMMAND_SETS[name]:
            self.send_line(command)
            self.update_idletasks()
            time.sleep(0.08)

    def send_manual(self):
        text = self.manual_var.get().strip()
        if text:
            self.send_line(text)
            self.manual_var.set("")

    def _on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    TuningApp().mainloop()

