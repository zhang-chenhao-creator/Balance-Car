# -*- coding: utf-8 -*-
"""
协议自检脚本：把 car_controller 中生成的每一帧，喂给"固件 usart3.c v5.7 解析
逻辑的镜像"（firmware_pid_parse），验证控制端编码与固件解析完全一致。

运行：python protocol_test.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from car_controller import (PARAMS, CMD, set_param_frame, read_params_frame,
                            parse_status_frame, firmware_pid_parse)


def test_control_bytes():
    """单字节指令必须是固件 usart3.c 中列出的值"""
    expect = {'forward': 0x41, 'back': 0x45, 'left': 0x46, 'right': 0x42,
              'brake': 0x5A, 'accel': 0x58, 'decel': 0x59}
    for name, val in expect.items():
        got = CMD[name][0]
        assert got == val, '指令 %s 应为 0x%02X，实际 0x%02X' % (name, val, got)
        print('  单字节指令 OK: %-7s 0x%02X "%s"' % (name, got, chr(got)))


def test_set_param_exact():
    """设置帧 7B 编号 '0' 数值 7D —— 固件解析结果必须等于设定值"""
    values = [0, 1, 5, 7, 9, 10, 25, 42, 99, 100, 123, 255, 500, 999, 1000, 1234, 9999]
    for name, idx in PARAMS:
        for v in values:
            frame = set_param_frame(name, v)
            r = firmware_pid_parse(frame)
            assert r is not None, '%s=%d 固件无响应' % (name, v)
            assert r == (ord(idx), v), \
                '固件解析 %s=%d 得到 %s（帧 %s）' % (name, v, r, frame.hex(' '))
    print('  参数设置帧 OK: 16 个取值 × 9 个参数全部与固件解析一致')


def test_set_param_official_doc_difference():
    """按官方文档(不带前导0)发送时固件的实际行为，验证 README 备注"""
    # 官方格式 7B 编号 数值 7D：发送 25 -> 固件只取最后一位 -> 5
    frame = bytes([0x7B, 0x31]) + b'25' + bytes([0x7D])
    r = firmware_pid_parse(frame)
    assert r == (ord('1'), 5), r
    # 发送 100 -> "00" -> 0
    frame = bytes([0x7B, 0x31]) + b'100' + bytes([0x7D])
    r = firmware_pid_parse(frame)
    assert r == (ord('1'), 0), r
    # 单数字 -> 0
    frame = bytes([0x7B, 0x31]) + b'5' + bytes([0x7D])
    r = firmware_pid_parse(frame)
    assert r == (ord('1'), 0), r
    print('  官方格式对照 OK: 25->5、100->0、5->0（固件取末位，验证前导 0 的必要性）')


def test_read_frame():
    f = read_params_frame()
    assert f == bytes([0x7B, 0x50, 0x50, 0x50, 0x7D]), f.hex()
    assert firmware_pid_parse(f) == ('read',)
    print('  读取参数帧 OK: 7B 50 50 50 7D -> 固件 PID_Send=1')


def test_parse_status():
    cases = [
        (b'{A12:34:78:-5}$', ('A', (12, 34, 78, -5))),
        (b'{A0:0:0:0}$',     ('A', (0, 0, 0, 0))),
        (b'{B-1:2:3}$',      ('B', (-1, 2, 3))),
        (b'{C1:2:3:4:5:6:7:8:9}$', ('C', (1, 2, 3, 4, 5, 6, 7, 8, 9))),
        (b'{C100:155:25:300:3:54:12:80:20}$', ('C', (100, 155, 25, 300, 3, 54, 12, 80, 20))),
    ]
    for raw, exp in cases:
        assert parse_status_frame(raw) == exp, raw
    # 坏帧
    for bad in (b'garbage', b'{A1:2}$', b'{D1:2:3:4}$', b'', b'$'):
        assert parse_status_frame(bad) is None
    print('  状态帧解析 OK: {A}/{B}/{C} 帧与坏帧过滤')


def test_stress_frames():
    """模拟串口字节流拆分：任意切分点下解析结果不变"""
    stream = b'{A12:34:78:-5}${B-1:2:3}${C1:2:3:4:5:6:7:8:9}$XZZZZ'
    frames = []
    buf = b''
    for b in stream:  # 逐字节喂入
        buf += bytes([b])
        while b'$' in buf:
            chunk, buf = buf.split(b'$', 1)
            if chunk:
                frames.append(parse_status_frame(chunk + b'$'))
    assert frames[0] == ('A', (12, 34, 78, -5))
    assert frames[1] == ('B', (-1, 2, 3))
    assert frames[2] == ('C', (1, 2, 3, 4, 5, 6, 7, 8, 9))
    print('  字节流拆分 OK: 不完整帧/多帧粘连均可正确切分')


if __name__ == '__main__':
    print('=== WHEELTEC 平衡小车 PC 控制终端：协议自检 ===')
    test_control_bytes()
    test_set_param_exact()
    test_set_param_official_doc_difference()
    test_read_frame()
    test_parse_status()
    test_stress_frames()
    print('=== 全部通过 ===')