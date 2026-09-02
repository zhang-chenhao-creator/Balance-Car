# -*- coding: utf-8 -*-
"""
==============================================================================
 WHEELTEC 平衡小车 —— PC 控制终端（对应固件 USART3 蓝牙控制部分）
==============================================================================
 本程序在 PC 端模拟原版手机蓝牙 APP 的控制指令，通过串口（USB 转 TTL 接到
 小车 USART3 引脚，或 PC 蓝牙 SPP 虚拟串口）直接控制小车。

 固件参考：MiniBalance_HARDWARE/USART3/usart3.c  (HAL 库 Cube版, v5.7)
 串口参数：9600, 8, N, 1

 ── PC → 小车（单字节指令，与蓝牙 APP 完全一致）───────────────────────────
   0x41 'A'   前进        Flag_front = 1
   0x45 'E'   后退        Flag_back  = 1
   0x42~0x44 'B'/'C'/'D' 右转        Flag_Right = 1
   0x46~0x48 'F'/'G'/'H' 左转        Flag_Left  = 1
   0x5A 'Z'   刹车/停止   清除所有方向标志
   0x58 'X'   加速(+100)  Target_Velocity += 100  （每收到一帧 +100）
   0x59 'Y'   减速(-100)  Target_Velocity -= 100
   （固件解析：大于 0x0A 的未知字节一律视为刹车）

 ── PC → 小车（参数设置帧，供调参使用）────────────────────────────────────
   帧格式：0x7B | 参数编号 | '0' | 数值(ASCII十进制) | 0x7D
   参数编号：'0'=Target_Velocity '1'=Balance_Kp '2'=Balance_Kd
            '3'=Velocity_Kp '4'=Velocity_Ki '5'=Turn_Kp
            '6'=Turn_Kd '7'=Distance_KP '8'=Distance_KD
   ※ 注意：本固件数据解析循环为 for(j=i;j>=4;j--) Data+=(Receive[j-1]-48)*10^(i-j)，
     未收集帧尾，且起始位固定在 Receive[3]。经逐字节推演，帧中补一个 '0'
     （即 7B 编号 '0' 数值 7D）可使"数值"完整生效（任意位数，0~9999 均精确）。
     若按官方文档不带 '0' 发送，例：发送 025 实际生效 25、发送 25 实际生效 5。
   读取参数帧：0x7B 'P' 'P' 'P' 0x7D 使 Receive[3]==0x50 -> PID_Send=1，
     固件响应 {C...}$。※ 但读取后固件调参解析器会处于异常状态（参数设置帧
     不再生效），需重新上电小车恢复，故"读取"按钮请慎用（见 README）。

 ── 小车 → PC（printf 输出，ASCII 帧，以 '$' 结尾）────────────────────────
   {A<左轮速>:<右轮速>:<电压%>:<倾角°>}$      速度/电压/角度
   {B<Pitch>:<Roll>:<Yaw>}$                   姿态角
   {C<TV>:<BKP>:<BKD>:<VKP>:<VKI>:<TKP>:<TKD>:<DKP>:<DKD>}$  PID 参数

 运行环境：Windows Python 3.8+，仅标准库。
   · 若已安装 pyserial（pip install pyserial）→ 使用 pyserial 后端（跨平台）；
   · 否则自动使用 Win32 API(ctypes) 后端，无需安装任何第三方库。
==============================================================================
"""
import ctypes
import queue
import re
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, scrolledtext, ttk

# ============================================================================
# 1. 协议常量与编码/解析（纯函数，可在无界面环境下测试）
# ============================================================================

FRAME_HEADER = 0x7B          # 帧头
FRAME_TAIL   = 0x7D          # 帧尾
DEFAULT_BAUD = 9600          # 蓝牙模块/USART3 默认波特率

# 单字节遥控指令（与蓝牙 APP 一致）
CMD = {
    'forward': b'A',         # 0x41 前进
    'back':    b'E',         # 0x45 后退
    'left':    b'F',         # 0x46 左转（固件 F/G/H 均可，统一发 F）
    'right':   b'B',         # 0x42 右转（固件 B/C/D 均可，统一发 B）
    'brake':   b'Z',         # 0x5A 刹车
    'accel':   b'X',         # 0x58 加速 +100
    'decel':   b'Y',         # 0x59 减速 -100
}
CMD_NAME = {v: k for k, v in CMD.items()}

# 参数编号（ASCII '0'~'8'）
PARAMS = [
    ('Target_Velocity', '0'),
    ('Balance_Kp',      '1'),
    ('Balance_Kd',      '2'),
    ('Velocity_Kp',     '3'),
    ('Velocity_Ki',     '4'),
    ('Turn_Kp',         '5'),
    ('Turn_Kd',         '6'),
    ('Distance_KP',     '7'),
    ('Distance_KD',     '8'),
]
PARAM_IDX = {name: ord(idx) for name, idx in PARAMS}


def set_param_frame(name, value):
    """生成参数设置帧：7B 编号 '0' 数值 7D（数值任意位数，固件精确生效）"""
    value = int(value)
    if value < 0 or value > 9999:
        raise ValueError('参数值范围 0~9999')
    return bytes([FRAME_HEADER, PARAM_IDX[name], ord('0')]) + \
        str(value).encode('ascii') + bytes([FRAME_TAIL])


def read_params_frame():
    """生成读取参数帧：7B 'P' 'P' 'P' 7D → 固件 Receive[3]==0x50 → 响应 {C...}$"""
    return bytes([FRAME_HEADER, 0x50, 0x50, 0x50, FRAME_TAIL])


_STATUS_RE = re.compile(rb'\{([ABC])([\-0-9:]+)\}\$')


def parse_status_frame(chunk):
    """解析小车回传帧 {A..}$ / {B..}$ / {C..}$，返回 ('A', (..)) 或 None"""
    m = _STATUS_RE.match(chunk.strip())
    if not m:
        return None
    kind = m.group(1).decode('ascii')
    try:
        vals = tuple(int(x) for x in m.group(2).split(b':'))
    except ValueError:
        return None
    expect = {'A': 4, 'B': 3, 'C': 9}
    if len(vals) != expect[kind]:
        return None
    return kind, vals


def firmware_pid_parse(frame):
    """固件 usart3.c 中 PID 帧解析逻辑的镜像（用于协议自检，勿用于 GUI）"""
    recv = bytearray(50)
    i, flag = 0, 0
    data = 0.0
    for b in frame:
        if b == FRAME_HEADER:
            flag = 1
        elif b == FRAME_TAIL:
            flag = 2
        if flag == 1:
            recv[i] = b
            i += 1
        if flag == 2:
            if recv[3] == 0x50:
                return ('read',)
            if recv[1] != 0x23:
                j = i
                while j >= 4:
                    data += (recv[j - 1] - 48) * pow(10, i - j)
                    j -= 1
                return (recv[1], int(data))
    return None


# ============================================================================
# 2. 串口后端：优先 pyserial（跨平台），否则 Win32 ctypes（零依赖）
# ============================================================================

try:  # pyserial 可用时优先
    import serial as _serial
    from serial.tools import list_ports as _list_ports
    HAVE_PYSERIAL = True
except Exception:
    _serial = None
    HAVE_PYSERIAL = False


def list_serial_ports():
    """列出串口 [(端口名, 描述)]"""
    if HAVE_PYSERIAL:
        try:
            return [(p.device, p.description or '') for p in _list_ports.comports()]
        except Exception:
            pass
    # Win32 注册表方式（含友好名称：USB 转串口 / 蓝牙 SPP 均显示"XX (COMx)"）
    ports = {}
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM') as key:
            i = 0
            while True:
                try:
                    _, val, _ = winreg.EnumValue(key, i)
                except OSError:
                    break
                i += 1
                if str(val).upper().startswith('COM'):
                    ports[str(val)] = ''
    except Exception:
        pass

    # 从 PnP 枚举补充分端口友好名称（USB 串口、蓝牙 SPP 虚拟串口）
    def _enum_walk(base, depth):
        try:
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as key:
                i = 0
                while True:
                    try:
                        sub = winreg.EnumKey(key, i)
                    except OSError:
                        break
                    i += 1
                    path = base + '\\' + sub
                    try:
                        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path) as k2:
                            fn = ''
                            try:
                                fn, _ = winreg.QueryValueEx(k2, 'FriendlyName')
                            except OSError:
                                pass
                            pn = ''
                            try:
                                with winreg.OpenKey(k2, 'Device Parameters') as kp:
                                    pn, _ = winreg.QueryValueEx(kp, 'PortName')
                            except OSError:
                                pass
                            if pn and str(pn).upper().startswith('COM') \
                                    and str(pn) in ports and fn:
                                ports[str(pn)] = str(fn)
                    except OSError:
                        pass
                    if depth > 1:
                        _enum_walk(path, depth - 1)
        except OSError:
            pass

    for top in (r'SYSTEM\CurrentControlSet\Enum\USB',
                r'SYSTEM\CurrentControlSet\Enum\BTHENUM',
                r'SYSTEM\CurrentControlSet\Enum\FTDIBUS',
                r'SYSTEM\CurrentControlSet\Enum\ACPI'):
        _enum_walk(top, 3)
    return [(p, ports[p]) for p in sorted(ports)]


class _PyserialPort:
    """pyserial 后端"""
    backend = 'pyserial'

    def __init__(self, name, baud):
        self._ser = _serial.Serial(port=name, baudrate=baud, bytesize=8,
                                   parity='N', stopbits=1, timeout=0.05)

    def write(self, data):
        self._ser.write(data)

    def read(self):
        return self._ser.read(256)

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


class _Win32Port:
    """Win32 串口 API (ctypes) 后端 —— 无需安装任何第三方库"""
    backend = 'win32-ctypes'

    GENERIC_READ, GENERIC_WRITE = 0x80000000, 0x40000000
    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL = 3, 0x80
    INVALID_HANDLE = ctypes.c_void_p(-1).value

    class _DCB(ctypes.Structure):
        _fields_ = [('DCBlength', ctypes.c_uint32),
                    ('BaudRate', ctypes.c_uint32),
                    ('fBits', ctypes.c_uint32),
                    ('wReserved', ctypes.c_uint16),
                    ('XonLim', ctypes.c_uint16),
                    ('XoffLim', ctypes.c_uint16),
                    ('ByteSize', ctypes.c_uint8),
                    ('Parity', ctypes.c_uint8),
                    ('StopBits', ctypes.c_uint8),
                    ('XonChar', ctypes.c_char),
                    ('XoffChar', ctypes.c_char),
                    ('ErrorChar', ctypes.c_char),
                    ('EofChar', ctypes.c_char),
                    ('EvtChar', ctypes.c_char),
                    ('wReserved1', ctypes.c_uint16)]

    class _Timeouts(ctypes.Structure):
        _fields_ = [('ReadIntervalTimeout', ctypes.c_uint32),
                    ('ReadTotalTimeoutMultiplier', ctypes.c_uint32),
                    ('ReadTotalTimeoutConstant', ctypes.c_uint32),
                    ('WriteTotalTimeoutMultiplier', ctypes.c_uint32),
                    ('WriteTotalTimeoutConstant', ctypes.c_uint32)]

    def __init__(self, name, baud):
        k32 = self._k32()
        handle = k32.CreateFileW(
            r'\\.\%s' % name, self.GENERIC_READ | self.GENERIC_WRITE, 0, None,
            self.OPEN_EXISTING, self.FILE_ATTRIBUTE_NORMAL, None)
        if not handle or handle == self.INVALID_HANDLE:
            raise OSError('无法打开串口 %s (error %d)' % (name, k32.GetLastError()))
        self._handle = handle
        try:
            dcb = self._DCB()
            if not k32.GetCommState(handle, ctypes.byref(dcb)):
                raise OSError('GetCommState 失败 (error %d)' % k32.GetLastError())
            dcb.DCBlength = ctypes.sizeof(self._DCB)
            dcb.BaudRate = baud
            dcb.ByteSize = 8
            dcb.Parity = 0      # NOPARITY
            dcb.StopBits = 0    # ONESTOPBIT
            # 清除流控位，保证 8N1 无流控；fBinary 置位
            dcb.fBits &= ~((1 << 2) | (1 << 3) | (1 << 8) | (1 << 9))
            dcb.fBits |= 1
            if not k32.SetCommState(handle, ctypes.byref(dcb)):
                raise OSError('SetCommState 失败 (error %d)' % k32.GetLastError())
            to = self._Timeouts()
            to.ReadIntervalTimeout = 0xFFFFFFFF  # 立即返回
            to.ReadTotalTimeoutMultiplier = 0
            to.ReadTotalTimeoutConstant = 0
            to.WriteTotalTimeoutMultiplier = 0
            to.WriteTotalTimeoutConstant = 0
            if not k32.SetCommTimeouts(handle, ctypes.byref(to)):
                raise OSError('SetCommTimeouts 失败 (error %d)' % k32.GetLastError())
        except Exception:
            k32.CloseHandle(handle)
            raise
        self._k32 = k32

    @staticmethod
    def _k32():
        try:
            k32 = ctypes.windll.kernel32
        except Exception:
            raise RuntimeError('当前系统不支持 Win32 串口 API，请安装 pyserial')
        # 必须显式声明原型：否则 64 位进程下 HANDLE 会被截断成 32 位，导致句柄无效
        k32.CreateFileW.restype = ctypes.c_void_p
        k32.CreateFileW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32,
                                    ctypes.c_uint32, ctypes.c_void_p,
                                    ctypes.c_uint32, ctypes.c_uint32,
                                    ctypes.c_void_p]
        k32.CloseHandle.argtypes = [ctypes.c_void_p]
        k32.CloseHandle.restype = ctypes.c_int
        k32.GetLastError.restype = ctypes.c_uint32
        for fn in ('GetCommState', 'SetCommState'):
            getattr(k32, fn).restype = ctypes.c_int
        k32.GetCommState.argtypes = [ctypes.c_void_p,
                                     ctypes.POINTER(_Win32Port._DCB)]
        k32.SetCommState.argtypes = [ctypes.c_void_p,
                                     ctypes.POINTER(_Win32Port._DCB)]
        k32.SetCommTimeouts.argtypes = [ctypes.c_void_p,
                                        ctypes.POINTER(_Win32Port._Timeouts)]
        k32.SetCommTimeouts.restype = ctypes.c_int
        k32.ReadFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                 ctypes.c_uint32,
                                 ctypes.POINTER(ctypes.c_uint32),
                                 ctypes.c_void_p]
        k32.ReadFile.restype = ctypes.c_int
        k32.WriteFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                  ctypes.c_uint32,
                                  ctypes.POINTER(ctypes.c_uint32),
                                  ctypes.c_void_p]
        k32.WriteFile.restype = ctypes.c_int
        return k32

    def write(self, data):
        buf = ctypes.create_string_buffer(bytes(data))
        written = ctypes.c_uint32(0)
        ok = self._k32.WriteFile(self._handle, buf, len(data),
                                 ctypes.byref(written), None)
        if not ok:
            raise OSError('串口写入失败 (error %d)' % self._k32.GetLastError())
        return written.value

    def read(self):
        buf = ctypes.create_string_buffer(256)
        got = ctypes.c_uint32(0)
        ok = self._k32.ReadFile(self._handle, buf, 256, ctypes.byref(got), None)
        if not ok:
            raise OSError('串口读取失败 (error %d)' % self._k32.GetLastError())
        return buf.raw[:got.value]

    def close(self):
        try:
            self._k32.CloseHandle(self._handle)
        except Exception:
            pass


def open_port(name, baud):
    """打开串口，返回后端对象或抛出异常"""
    if HAVE_PYSERIAL:
        return _PyserialPort(name, baud)
    if sys.platform == 'win32':
        return _Win32Port(name, baud)
    raise RuntimeError('未安装 pyserial 且非 Windows 系统，无法打开串口')


# ============================================================================
# 3. 图形界面
# ============================================================================

FONT    = ('Microsoft YaHei UI', 10)
FONT_B  = ('Microsoft YaHei UI', 10, 'bold')
FONT_BT = ('Microsoft YaHei UI', 12, 'bold')   # 方向盘按钮字体
FONT_BIG = ('Microsoft YaHei UI', 14, 'bold')
FONT_M  = ('Consolas', 9)

HOLD_KEYS = {  # 按住持续发送的按键 -> 指令
    'w': 'forward', 'up': 'forward',
    's': 'back', 'down': 'back',
    'a': 'left', 'left': 'left',
    'd': 'right', 'right': 'right',
    'space': 'brake', 'z': 'brake',
}
MOMENTARY_KEYS = {  # 按一下发送一次的按键
    'x': 'accel', 'y': 'decel',
}
HOLD_REPEAT_MS = 50   # 按住时指令重复周期（如蓝牙 APP 持续下发的效果）
POLL_RX_MS = 25       # 接收队列轮询周期
PLOT_MS = 100         # 波形刷新周期


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('WHEELTEC 平衡小车 —— PC 控制终端（蓝牙协议版 / 9600 8N1）')
        self.geometry('1180x800')
        self.minsize(1020, 720)

        self._port = None            # 打开的串口后端
        self._rxq = queue.Queue()    # 接收线程 -> 界面
        self._rxbuf = b''            # 半帧缓冲区
        self._reader = None
        self._quit_reader = False
        self._held = []              # 当前按住的（指令名）列表，最近按下的在末尾
        self._status = {k: None for k in ('A', 'B', 'C')}
        self._plots = {'angle': deque(maxlen=320), 'pitch': deque(maxlen=320)}
        self._log_keep = 400

        self._build_ui()
        self._bind_keys()
        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self.after(POLL_RX_MS, self._poll_rx)
        self.after(PLOT_MS, self._redraw_plot)
        self.after(HOLD_REPEAT_MS, self._repeat_tick)

    # ------------------------------------------------------------------ 界面
    def _build_ui(self):
        pad = {'padx': 6, 'pady': 4}

        # ---------- 顶部：连接区 ----------
        top = ttk.Frame(self)
        top.pack(side='top', fill='x', **pad)
        ttk.Label(top, text='串口:', font=FONT).pack(side='left')
        self._port_cb = ttk.Combobox(top, width=12, font=FONT, state='readonly')
        self._port_cb.pack(side='left')
        self._port_desc = tk.Label(top, text='', fg='#888888',
                                   font=('Microsoft YaHei UI', 9), anchor='w')
        self._port_desc.pack(side='left', padx=(8, 0))
        self._port_cb.bind('<<ComboboxSelected>>', lambda e: self._show_port_desc())
        ttk.Button(top, text='刷新', width=6, command=self._refresh_ports, takefocus=0).pack(side='left')
        ttk.Label(top, text='波特率:', font=FONT).pack(side='left', padx=(12, 0))
        self._baud_cb = ttk.Combobox(top, width=7, font=FONT, state='readonly',
                                     values=('9600', '115200', '230400'))
        self._baud_cb.set(str(DEFAULT_BAUD))
        self._baud_cb.pack(side='left')
        self._conn_btn = ttk.Button(top, text='连接串口', width=10, command=self._toggle_conn, takefocus=0)
        self._conn_btn.pack(side='left', padx=(12, 0))
        self._conn_lbl = tk.Label(top, text='● 未连接', fg='#c62828', font=FONT_B)
        self._conn_lbl.pack(side='left', padx=(10, 0))
        self._backend_lbl = tk.Label(top, text='后端: ' + ('pyserial' if HAVE_PYSERIAL else 'Win32 API (零依赖)'),
                                     fg='#888888', font=('Microsoft YaHei UI', 9))
        self._backend_lbl.pack(side='right')
        self._refresh_ports()

        # ---------- 主体三栏 ----------
        body = ttk.Frame(self)
        body.pack(side='top', fill='both', expand=True, **pad)

        # ===== 左栏：遥控面板 =====
        left = tk.LabelFrame(body, text=' 遥控（同蓝牙 APP 指令） ', font=FONT_B)
        left.pack(side='left', fill='y', padx=(0, 8))
        bt = {'takefocus': 0, 'width': 8, 'font': FONT_BT, 'relief': 'raised'}

        def make_btn(parent, text, cmd, color=None, row=None, col=None):
            b = tk.Button(parent, text=text, command=lambda c=cmd: self._press_hold(c), **bt)
            if color:
                b.configure(bg=color, activebackground=color)
            b.bind('<ButtonPress-1>', lambda e, c=cmd: self._press_hold(c))
            b.bind('<ButtonRelease-1>', lambda e: self._release_hold(cmd))
            b.bind('<Leave>', lambda e: self._release_hold(cmd))
            b.grid(row=row, column=col, padx=4, pady=4, sticky='nsew')
            return b

        # 方向盘
        make_btn(left, '▲ 前进\nW / ↑  (A)', 'forward', row=0, col=1)
        make_btn(left, '◀ 左转\nA / ←  (F)', 'left', row=1, col=0)
        make_btn(left, '■ 刹车\n空格 / Z', 'brake', '#ffcdd2', row=1, col=1)
        make_btn(left, '右转 ▶\nD / →  (B)', 'right', row=1, col=2)
        make_btn(left, '▼ 后退\nS / ↓  (E)', 'back', row=2, col=1)
        # 加减速 + 急停
        make_btn(left, '加速 +100\nX', 'accel', '#c8e6c9', row=3, col=0)
        make_btn(left, '减速 -100\nY', 'decel', '#fff9c4', row=3, col=1)
        tk.Button(left, text='紧急停车', font=FONT_B, fg='white', bg='#d32f2f',
                  activebackground='#b71c1c', activeforeground='white', width=8,
                  relief='raised', takefocus=0,
                  command=lambda: self._send_burst('brake', 3)).grid(row=3, column=2, padx=4, pady=4)
        tk.Label(left, text='按住方向键/按钮持续控制，松开自动刹车 Z\n'
                            'X/Y：每次 +100/-100（Target_Velocity）',
                 font=('Microsoft YaHei UI', 9), fg='#555555',
                 justify='left').grid(row=4, column=0, columnspan=3, padx=6, pady=(10, 2))

        # ===== 中栏：状态 + 波形 =====
        mid = tk.LabelFrame(body, text=' 小车状态回传（{A}/{B}/$ 帧） ', font=FONT_B)
        mid.pack(side='left', fill='both', expand=True, padx=(0, 8))

        st = ttk.Frame(mid)
        st.pack(fill='x', padx=8, pady=6)
        self._volt_bar = ttk.Progressbar(st, length=120, maximum=100)
        self._volt_bar.grid(row=0, column=1, padx=(6, 10), pady=2)
        self._lbl_volt = tk.Label(st, text='电压 --%', font=FONT)
        self._lbl_volt.grid(row=0, column=0, sticky='w')
        self._lbl_angle = tk.Label(st, text='倾角 --°', font=FONT)
        self._lbl_angle.grid(row=0, column=2, padx=(10, 0))
        self._lbl_speed = tk.Label(st, text='左轮 --  右轮 --', font=FONT)
        self._lbl_speed.grid(row=1, column=0, columnspan=2, sticky='w', pady=(4, 0))
        self._lbl_pose = tk.Label(st, text='Pitch --  Roll --  Yaw --', font=FONT)
        self._lbl_pose.grid(row=1, column=2, padx=(10, 0), pady=(4, 0))
        self._lbl_goal = tk.Label(st, text='目标速度 Target_Velocity: --（本机估算）', font=FONT, fg='#1565C0')
        self._lbl_goal.grid(row=2, column=0, columnspan=3, sticky='w', pady=(4, 0))
        self._lbl_raw = tk.Label(st, text='最近帧: --', font=FONT_M, fg='#666666', anchor='w')
        self._lbl_raw.grid(row=3, column=0, columnspan=3, sticky='we', pady=(4, 0))

        plotf = ttk.Frame(mid)
        plotf.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        self._canvas = tk.Canvas(plotf, height=180, bg='white', highlightthickness=1,
                                 highlightbackground='#cccccc')
        self._canvas.pack(fill='both', expand=True)
        c = self._canvas
        c.create_text(8, 6, anchor='nw', fill='#1565C0', font=('Microsoft YaHei UI', 9),
                      text='■ 倾角 Angle_Balance (°)')
        c.create_text(8, 22, anchor='nw', fill='#EF6C00', font=('Microsoft YaHei UI', 9),
                      text='■ 俯仰 Pitch (°)')

        # ===== 右栏：参数设置 =====
        right = tk.LabelFrame(body, text=' 参数设置（调参 / 与 APP 相同） ', font=FONT_B)
        right.pack(side='left', fill='y')
        self._pid_vars = {}
        self._pid_cur = {}
        rp = ttk.Frame(right)
        rp.pack(fill='x', padx=8, pady=6)
        for i, (name, idx) in enumerate(PARAMS):
            ttk.Label(rp, text=name, font=FONT).grid(row=i, column=0, sticky='w', pady=2)
            v = tk.StringVar(value='')
            self._pid_vars[name] = v
            e = ttk.Entry(rp, textvariable=v, width=9, font=FONT)
            e.grid(row=i, column=1, padx=(6, 4), pady=2)
            ttk.Button(rp, text='设置', width=5, takefocus=0,
                       command=lambda n=name: self._set_param(n)).grid(row=i, column=2, pady=2)
            cur = tk.Label(rp, text='--', font=('Microsoft YaHei UI', 9), fg='#666666', width=10)
            cur.grid(row=i, column=3, sticky='w', padx=(6, 0))
            self._pid_cur[name] = cur
        ttk.Button(right, text='读取全部参数（慎用）', width=20, takefocus=0,
                   command=self._read_params).pack(padx=8, pady=(6, 2))
        tk.Label(right, text='固件备注：设置帧按\n"7B 编号 0 数值 7D" 发送，\n'
                            '数值 0~9999 精确生效；\n'
                            '“读取”后调参解析会失效，\n需重新上电恢复。',
                 font=('Microsoft YaHei UI', 9), fg='#b26a00',
                 justify='left', anchor='w').pack(padx=8, pady=(0, 8))

        # ---------- 底部：日志 ----------
        bottom = tk.LabelFrame(self, text=' 通信日志 ', font=FONT_B)
        bottom.pack(side='bottom', fill='x', padx=6, pady=(4, 8))
        self._log = scrolledtext.ScrolledText(bottom, height=7, font=FONT_M,
                                              state='disabled', bg='#fafafa')
        self._log.pack(fill='x', padx=6, pady=4)

    def _refresh_ports(self):
        ports = list_serial_ports()
        self._port_list = ports
        self._port_cb['values'] = [p[0] for p in ports] or ['（无串口）']
        if ports:
            self._port_cb.set(ports[0][0])
        else:
            self._port_cb.set('')
        self._show_port_desc()

    def _show_port_desc(self):
        name = self._port_cb.get()
        desc = ''
        for p, d in getattr(self, '_port_list', []):
            if p == name:
                desc = d
                break
        if not desc:
            desc = '蓝牙/串口模块配对后会在“设备管理器-端口”里出现新 COM 口'
        self._port_desc.config(text=desc[:46])

    # ------------------------------------------------------------------ 键鼠
    def _bind_keys(self):
        self.bind('<KeyPress>', self._on_key_press)
        self.bind('<KeyRelease>', self._on_key_release)

    def _on_key_press(self, e):
        k = e.keysym.lower()
        if not self._port:
            return
        if k in HOLD_KEYS:
            cmd = HOLD_KEYS[k]
            if cmd not in self._held:
                self._held.append(cmd)
                self._send_cmd(cmd)     # 立即响应，不等重复周期
        elif k in MOMENTARY_KEYS:
            self._send_cmd(MOMENTARY_KEYS[k], log=True)

    def _on_key_release(self, e):
        k = e.keysym.lower()
        if k in HOLD_KEYS:
            self._release_hold(HOLD_KEYS[k])

    def _press_hold(self, cmd):
        if cmd in ('accel', 'decel'):      # 加减速只在按下瞬间发送一次
            if self._port:
                self._send_cmd(cmd, log=True)
            return
        if cmd not in self._held:
            self._held.append(cmd)
            if self._port:
                self._send_cmd(cmd)     # 立即响应，不等重复周期

    def _release_hold(self, cmd):
        if cmd in self._held:
            self._held.remove(cmd)
        if not self._port:
            self._held.clear()
            return
        if self._held:                   # 还有按键按住：立刻切到最近按下的指令
            self._send_cmd(self._held[-1])
        else:                            # 全部松开：刹车
            self._send_cmd('brake')

    def _repeat_tick(self):
        if self._held and self._port:
            self._send_cmd(self._held[-1])
        self.after(HOLD_REPEAT_MS, self._repeat_tick)

    # ------------------------------------------------------------------ 发送
    def _send_cmd(self, name, log=False):
        data = CMD[name]
        try:
            self._port.write(data)
        except Exception as ex:
            self._log_line('!! 发送失败: %s' % ex)
            return
        if log:
            desc = {'forward': '前进', 'back': '后退', 'left': '左转', 'right': '右转',
                    'brake': '刹车', 'accel': '加速 +100', 'decel': '减速 -100'}[name]
            self._log_line('TX  %s  "%s"   %s' % (data.hex().upper(), data.decode('ascii'), desc))

    def _send_burst(self, name, n):
        if not self._port:
            return
        data = CMD[name]
        for _ in range(n):
            self._port.write(data)
            time.sleep(0.02)
        self._log_line('TX  %s x%d  紧急停车' % (data.hex().upper(), n))

    def _set_param(self, name):
        if not self._port:
            self._log_line('!! 未连接串口，参数未发送')
            return
        txt = self._pid_vars[name].get().strip()
        try:
            frame = set_param_frame(name, int(txt))
        except ValueError:
            self._log_line('!! 参数格式错误: %s = %r（要求 0~9999 的整数）' % (name, txt))
            return
        try:
            self._port.write(frame)
        except Exception as ex:
            self._log_line('!! 发送失败: %s' % ex)
            return
        self._log_line('TX  %s  设置 %s = %s' % (frame.hex(' ').upper(), name, txt))

    def _read_params(self):
        if not self._port:
            self._log_line('!! 未连接串口')
            return
        frame = read_params_frame()
        self._port.write(frame)
        self._log_line('TX  %s  读取参数（固件将回传 {C...}$；'
                       '此后调参设置帧会失效，需重新上电恢复）' % frame.hex(' ').upper())

    # ------------------------------------------------------------------ 连接
    def _toggle_conn(self):
        if self._port:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        name = self._port_cb.get()
        if not name or name.startswith('（'):
            self._log_line('!! 未选择串口')
            return
        try:
            port = open_port(name, int(self._baud_cb.get()))
        except Exception as ex:
            messagebox.showerror('连接失败', '%s\n\n请检查 USB 转 TTL 是否插好、'
                                          '驱动是否安装、端口是否被占用。' % ex)
            self._log_line('!! 连接失败: %s' % ex)
            return
        self._port = port
        self._rxbuf = b''
        self._quit_reader = False
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self._conn_btn.config(text='断开连接')
        self._conn_lbl.config(text='● 已连接  9600 8N1', fg='#2e7d32')
        try:
            port.write(CMD['brake'])     # 连接后先刹车，清除小车残留标志
        except Exception:
            pass
        self._log_line('== 已连接 %s @ %d baud (%s) ==' % (name, int(self._baud_cb.get()), port.backend))

    def _disconnect(self):
        self._quit_reader = True
        port, self._port = self._port, None
        self._held.clear()
        try:
            port.close()
        except Exception:
            pass
        self._conn_btn.config(text='连接串口')
        self._conn_lbl.config(text='● 未连接', fg='#c62828')
        self._log_line('== 已断开 ==')

    def _reader_loop(self):
        while not self._quit_reader:
            try:
                data = self._port.read()
            except Exception as ex:
                if not self._quit_reader:
                    self._rxq.put(('err', str(ex)))
                break
            if not data:
                continue
            self._rxbuf += data
            while b'$' in self._rxbuf:
                chunk, self._rxbuf = self._rxbuf.split(b'$', 1)
                if chunk:
                    self._rxq.put(('chunk', chunk + b'$'))

    # ------------------------------------------------------------------ 接收
    def _poll_rx(self):
        try:
            while True:
                kind, payload = self._rxq.get_nowait()
                if kind == 'err':
                    self._log_line('!! 串口错误: %s' % payload)
                    if self._port:
                        self._disconnect()
                    break
                self._on_frame(payload)
        except queue.Empty:
            pass
        self.after(POLL_RX_MS, self._poll_rx)

    def _on_frame(self, chunk):
        parsed = parse_status_frame(chunk)
        self._lbl_raw.config(text='最近帧: %s' % chunk.decode('ascii', 'replace').strip())
        if parsed is None:
            return
        kind, vals = parsed
        self._status[kind] = vals
        if kind == 'A':
            l, r, volt, ang = vals
            self._lbl_volt.config(text='电压 %d%%' % volt)
            self._volt_bar['value'] = max(0, min(100, volt))
            self._lbl_angle.config(text='倾角 %d°' % ang)
            self._lbl_speed.config(text='左轮 %d   右轮 %d' % (l, r))
            self._plots['angle'].append(ang)
        elif kind == 'B':
            p, rl, yw = vals
            self._lbl_pose.config(text='Pitch %d  Roll %d  Yaw %d' % (p, rl, yw))
            self._plots['pitch'].append(p)
        elif kind == 'C':
            names = [n for n, _ in PARAMS]
            for n, v in zip(names, vals):
                self._pid_cur[n].config(text='当前 %d' % v)
                if not self._pid_vars[n].get():
                    self._pid_vars[n].set(str(v))

    # ------------------------------------------------------------------ 波形
    def _redraw_plot(self):
        c = self._canvas
        c.delete('line')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or h < 50:
            self.after(PLOT_MS, self._redraw_plot)
            return
        m_l, m_r, m_t, m_b = 34, 8, 30, 18
        pw, ph = w - m_l - m_r, h - m_t - m_b

        # 网格：-30..30°
        for deg in (-30, -20, -10, 0, 10, 20, 30):
            y = m_t + (30 - deg) / 60.0 * ph
            c.create_line(m_l, y, w - m_r, y, fill='#eeeeee', tags='line')
            c.create_text(m_l - 6, y, anchor='e', font=FONT_M, fill='#999999',
                          text=str(deg), tags='line')
        c.create_line(m_l, m_t + ph / 2, w - m_r, m_t + ph / 2, fill='#bbbbbb', tags='line')

        def draw(series, color):
            pts = list(series)
            if len(pts) < 2:
                return
            n = len(pts)
            coords = []
            for i, v in enumerate(pts):
                x = m_l + i / (n - 1) * pw
                y = m_t + (30 - max(-30, min(30, v))) / 60.0 * ph
                coords += [x, y]
            c.create_line(*coords, fill=color, width=2, tags='line')

        draw(self._plots['angle'], '#1565C0')
        draw(self._plots['pitch'], '#EF6C00')
        self.after(PLOT_MS, self._redraw_plot)

    # ------------------------------------------------------------------ 日志
    def _log_line(self, text):
        ts = time.strftime('%H:%M:%S')
        self._log.configure(state='normal')
        self._log.insert('end', '[%s] %s\n' % (ts, text))
        # 截断过长日志
        n = int(self._log.index('end-1c').split('.')[0])
        if n > self._log_keep:
            self._log.delete('1.0', '%d.0' % (n - self._log_keep))
        self._log.see('end')
        self._log.configure(state='disabled')

    def _on_close(self):
        if self._port:
            self._disconnect()
        self.destroy()


def main():
    app = App()
    app.mainloop()


if __name__ == '__main__':
    main()