import argparse
import csv
import math
import statistics
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm, mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "实验数据"
DEFAULT_REPORT_DIR = ROOT.parent / "_local" / "实验报告" / "EX2"
DEFAULT_OUTPUT_PDF = DEFAULT_REPORT_DIR / "MPU6050姿态解算实验报告.pdf"

FILES = {
    "STATIC": DATA_DIR / "MPU6050_STATIC_20260707_095424.csv",
    "TILT": DATA_DIR / "MPU6050_TILT_20260707_095549.csv",
    "SHAKE": DATA_DIR / "MPU6050_SHAKE_20260707_095651.csv",
}

PLOTS = {
    "STATIC": DATA_DIR / "MPU6050_STATIC_20260707_095424.png",
    "TILT": DATA_DIR / "MPU6050_TILT_20260707_095549.png",
    "SHAKE": DATA_DIR / "MPU6050_SHAKE_20260707_095651.png",
}

DEFAULT_BOARD_IMAGE = (
    ROOT.parent / "_local" / "参考资料" / "课程与硬件资料" / "主控介绍图.png"
)


def parse_args():
    parser = argparse.ArgumentParser(description="生成 MPU6050 姿态解算实验报告")
    parser.add_argument("--student-name", default="待填写", help="报告封面姓名")
    parser.add_argument("--student-id", default="待填写", help="报告封面学号")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PDF,
        help="输出 PDF；默认写入仅本地 _local/实验报告/EX2",
    )
    parser.add_argument(
        "--board-image",
        type=Path,
        default=DEFAULT_BOARD_IMAGE,
        help="可选主控图片路径",
    )
    return parser.parse_args()


def register_fonts():
    pdfmetrics.registerFont(TTFont("SimHei", r"C:\Windows\Fonts\simhei.ttf"))
    pdfmetrics.registerFont(TTFont("Deng", r"C:\Windows\Fonts\Deng.ttf"))


def read_rows(path):
    with path.open(newline="", encoding="utf-8-sig") as f:
        return [row for row in csv.DictReader(f) if row.get("record") == "A"]


def f(row, key):
    return float(row[key])


def i(row, key):
    return int(row[key])


def fmt(value, digits=3):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def stat_values(rows, key):
    vals = [f(r, key) for r in rows if r.get(key) not in ("", None)]
    return {
        "mean": statistics.mean(vals),
        "std": statistics.pstdev(vals),
        "min": min(vals),
        "max": max(vals),
    }


def summarize(rows):
    out = {
        "samples": len(rows),
        "ok": sum(1 for r in rows if r["status"] == "OK"),
        "wait": sum(1 for r in rows if r["status"] == "DMP_WAIT"),
        "dmp_hz": stat_values(rows, "dmp_hz"),
        "sat": sum(
            1
            for r in rows
            if any(abs(i(r, key)) >= 32767 for key in ["ax_raw", "ay_raw", "az_raw"])
        ),
    }
    for key in ["pitch_acc", "pitch_dmp", "pitch_comp", "pitch_kalman"]:
        out[key] = stat_values(rows, key)
    for key in ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]:
        out[key] = stat_values(rows, key)
    return out


def make_styles():
    styles = getSampleStyleSheet()
    styles.add(
        ParagraphStyle(
            name="CNTitle",
            fontName="SimHei",
            fontSize=22,
            leading=30,
            alignment=TA_CENTER,
            spaceAfter=16,
        )
    )
    styles.add(
        ParagraphStyle(
            name="CNSubTitle",
            fontName="SimHei",
            fontSize=15,
            leading=22,
            alignment=TA_CENTER,
            spaceAfter=10,
        )
    )
    styles.add(
        ParagraphStyle(
            name="CNHeading1",
            fontName="SimHei",
            fontSize=15,
            leading=21,
            spaceBefore=10,
            spaceAfter=6,
        )
    )
    styles.add(
        ParagraphStyle(
            name="CNHeading2",
            fontName="SimHei",
            fontSize=12,
            leading=18,
            spaceBefore=8,
            spaceAfter=4,
        )
    )
    styles.add(
        ParagraphStyle(
            name="CNBody",
            fontName="Deng",
            fontSize=10.2,
            leading=16,
            alignment=TA_LEFT,
            spaceAfter=5,
        )
    )
    styles.add(
        ParagraphStyle(
            name="CNNote",
            fontName="Deng",
            fontSize=8.8,
            leading=13,
            textColor=colors.HexColor("#444444"),
            spaceAfter=4,
        )
    )
    styles.add(
        ParagraphStyle(
            name="TableText",
            fontName="Deng",
            fontSize=8,
            leading=10,
            alignment=TA_CENTER,
        )
    )
    styles.add(
        ParagraphStyle(
            name="TableHead",
            fontName="SimHei",
            fontSize=8,
            leading=10,
            alignment=TA_CENTER,
        )
    )
    return styles


def p(text, style):
    return Paragraph(text, style)


def table(data, widths=None, font_size=8, header=True):
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    style = [
        ("FONTNAME", (0, 0), (-1, -1), "Deng"),
        ("FONTSIZE", (0, 0), (-1, -1), font_size),
        ("LEADING", (0, 0), (-1, -1), font_size + 2),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#B8C0CC")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (0, 0), (-1, -1), "CENTER"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    if header:
        style += [
            ("FONTNAME", (0, 0), (-1, 0), "SimHei"),
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#EAF1F8")),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.HexColor("#1F2D3D")),
        ]
    t.setStyle(TableStyle(style))
    return t


def image(path, width):
    img = Image(str(path))
    ratio = img.imageHeight / float(img.imageWidth)
    img.drawWidth = width
    img.drawHeight = width * ratio
    return img


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Deng", 8)
    canvas.setFillColor(colors.HexColor("#666666"))
    canvas.drawString(18 * mm, 10 * mm, "MPU6050姿态解算实验报告")
    canvas.drawRightString(192 * mm, 10 * mm, f"第 {doc.page} 页")
    canvas.restoreState()


def build(student_name, student_id, output_pdf, board_image):
    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    register_fonts()
    styles = make_styles()
    rows = {name: read_rows(path) for name, path in FILES.items()}
    sums = {name: summarize(value) for name, value in rows.items()}

    doc = SimpleDocTemplate(
        str(output_pdf),
        pagesize=A4,
        rightMargin=18 * mm,
        leftMargin=18 * mm,
        topMargin=16 * mm,
        bottomMargin=16 * mm,
        title="MPU6050姿态解算实验报告",
        author=student_name,
    )

    story = []
    story.append(Spacer(1, 18 * mm))
    story.append(p("杭州电子科技大学", styles["CNSubTitle"]))
    story.append(p("课程设计报告", styles["CNTitle"]))
    cover = [
        ["课程名称", "具身智能实践综合设计I"],
        ["设计名称", "MPU6050传感器通信与姿态解算实验"],
        ["指导教师", "曾毓"],
        ["小组学生姓名", student_name],
        ["小组学生学号", student_id],
        ["学生班级", "具身智能班"],
        ["学生专业", "具身智能"],
        ["实践日期", "2026年7月7日"],
    ]
    story.append(Spacer(1, 8 * mm))
    story.append(table(cover, [45 * mm, 105 * mm], font_size=11, header=False))
    story.append(Spacer(1, 18 * mm))
    story.append(
        p(
            "本报告基于 WHEELTEC B585/C10B STM32F103RCT6 小车平台，完成 MPU6050 原始数据读取、DMP姿态解算、互补滤波和一维卡尔曼滤波对比，并给出静止、倾斜、快速晃动三种工况下的采集曲线与统计分析。",
            styles["CNBody"],
        )
    )
    story.append(PageBreak())

    story.append(p("1 实验目的与任务完成情况", styles["CNHeading1"]))
    story.append(
        p(
            "实验目标是完成 MPU6050 与 STM32 的通信，读取原始加速度与陀螺仪数据并完成单位换算；调用 DMP 输出 Pitch 姿态；在同一固件中实现互补滤波和一维卡尔曼滤波，并通过上位机记录三类状态下的曲线进行对比。",
            styles["CNBody"],
        )
    )
    task_table = [
        ["要求", "完成情况"],
        ["原始数据读取", "软件 I2C 读取 ax/ay/az/gx/gy/gz，输出 raw、g、deg/s 三种形式。"],
        ["DMP姿态解算", "DMP 初始化成功，WHO_AM_I=0x68，串口输出 dmp_hz，实测约 11-13 Hz。"],
        ["滤波方法对比", "实现 DMP、互补滤波、卡尔曼滤波，并保存 STATIC/TILT/SHAKE 曲线。"],
        ["方向验证", "首次测试发现前后倾主要体现在 ay_g，已将 Pitch 统一修正为绕 MPU6050 X 轴；报告中按前倾为负、后仰为正约定说明。"],
        ["烧录与安全", "先备份 256 KiB Flash，再写入并回读校验；固件未初始化电机 PWM，避免误动作。"],
    ]
    story.append(table(task_table, [42 * mm, 122 * mm], font_size=8.4))

    story.append(p("2 硬件连接与系统结构", styles["CNHeading1"]))
    story.append(
        p(
            "主控采用 WHEELTEC B585/C10B 板载 STM32F103RCT6。USART1 通过 CH9102 映射到电脑 COM7，便于烧录与 CSV 数据采集。MPU6050 模块接在软件 I2C 总线上，INT 引脚用于保留 DMP/FIFO 中断能力。",
            styles["CNBody"],
        )
    )
    conn = [
        ["STM32F103RCT6", "MPU6050/外设", "说明"],
        ["PB14", "SCL", "软件 I2C 时钟线"],
        ["PB15", "SDA", "软件 I2C 数据线"],
        ["PB9", "INT", "MPU6050 中断输入，保留 DMP/FIFO 状态"],
        ["3.3V/5V", "VCC", "按模块丝印供电，本实验使用板载传感器模块"],
        ["GND", "GND", "共地"],
        ["PA9/PA10", "CH9102/COM7", "USART1 调试与数据输出，115200-8-N-1"],
    ]
    story.append(table(conn, [35 * mm, 38 * mm, 91 * mm], font_size=8.2))
    story.append(p("图1为小车主控与 MPU6050 模块位置示意。", styles["CNNote"]))
    if board_image.exists():
        story.append(image(board_image, 150 * mm))

    story.append(p("3 软件设计与数据格式", styles["CNHeading1"]))
    story.append(
        p(
            "固件基于 HAL 工程改造，初始化系统时钟、USART1、软件 I2C 和 MPU6050。主循环按照设定周期读取原始数据、计算加速度角、更新互补滤波与卡尔曼滤波，并尝试读取 DMP FIFO 数据。串口命令包括 HELP、STATUS、CAL、LOG、MODE、STATE、RATE，便于同一固件切换记录状态。",
            styles["CNBody"],
        )
    )
    story.append(
        p(
            "CSV 字段为：A, ms, state, mode, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, pitch_acc, pitch_dmp, pitch_comp, pitch_kalman, dmp_hz, dt_ms, status。",
            styles["CNNote"],
        )
    )
    story.append(
        p(
            "单位换算采用 MPU6050 常用量程：加速度 raw/16384 得到 g，陀螺仪 raw/16.4 得到 deg/s。若需要换算为 m/s^2，可再乘以 9.80665。",
            styles["CNBody"],
        )
    )

    story.append(p("4 静止原始数据与噪声统计", styles["CNHeading1"]))
    story.append(
        p(
            "静止状态采集 1030 组数据。下表列出前 10 组原始数据和单位换算值，用于证明原始读取和转换流程正确。",
            styles["CNBody"],
        )
    )
    raw_head = [
        [
            "序号",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "ax_g",
            "ay_g",
            "az_g",
            "gx_dps",
            "gy_dps",
            "gz_dps",
        ]
    ]
    for idx, r in enumerate(rows["STATIC"][:10], start=1):
        raw_head.append(
            [
                str(idx),
                r["ax_raw"],
                r["ay_raw"],
                r["az_raw"],
                r["gx_raw"],
                r["gy_raw"],
                r["gz_raw"],
                fmt(f(r, "ax_g"), 3),
                fmt(f(r, "ay_g"), 3),
                fmt(f(r, "az_g"), 3),
                fmt(f(r, "gx_dps"), 3),
                fmt(f(r, "gy_dps"), 3),
                fmt(f(r, "gz_dps"), 3),
            ]
        )
    story.append(table(raw_head, [10 * mm] + [12 * mm] * 6 + [14 * mm] * 6, font_size=6.4))

    static_stats = sums["STATIC"]
    noise_table = [["量", "均值", "标准差", "最小值", "最大值"]]
    for key, label in [
        ("ax_g", "ax_g"),
        ("ay_g", "ay_g"),
        ("az_g", "az_g"),
        ("gx_dps", "gx_dps"),
        ("gy_dps", "gy_dps"),
        ("gz_dps", "gz_dps"),
        ("pitch_acc", "pitch_acc"),
        ("pitch_comp", "pitch_comp"),
        ("pitch_kalman", "pitch_kalman"),
        ("pitch_dmp", "pitch_dmp"),
    ]:
        st = static_stats[key]
        noise_table.append([label, fmt(st["mean"]), fmt(st["std"]), fmt(st["min"]), fmt(st["max"])])
    story.append(Spacer(1, 5 * mm))
    story.append(table(noise_table, [34 * mm, 32 * mm, 32 * mm, 32 * mm, 32 * mm], font_size=7.8))
    story.append(
        p(
            "静止状态下互补滤波和卡尔曼滤波的 Pitch 标准差约为 0.38 deg，明显小于快速晃动状态；DMP 输出更平滑，但本次集成中的 DMP 更新频率约 12 Hz，低于主循环 50 Hz。",
            styles["CNBody"],
        )
    )
    story.append(image(PLOTS["STATIC"], 165 * mm))

    story.append(PageBreak())
    story.append(p("5 姿态解算方法与融合思想", styles["CNHeading1"]))
    story.append(p("5.1 加速度角", styles["CNHeading2"]))
    story.append(
        p(
            "静态或低速运动时，加速度计主要测量重力方向，因此可由重力分量计算倾角。本实验前后倾主要对应 MPU6050 的 ay 变化，故 Pitch 加速度角采用 atan2(ay_g, sqrt(ax_g^2 + az_g^2))。加速度法没有积分漂移，但快速晃动时线加速度会混入重力估计，噪声和突变较明显。",
            styles["CNBody"],
        )
    )
    story.append(p("5.2 互补滤波", styles["CNHeading2"]))
    story.append(
        p(
            "互补滤波将陀螺仪短时响应和加速度计长期稳定性结合：pitch = alpha * (pitch + gyro_x * dt) + (1 - alpha) * pitch_acc。本实验 alpha 取 0.98。它计算量小、响应快，适合 STM32F103 这类资源有限的平台。",
            styles["CNBody"],
        )
    )
    story.append(p("5.3 一维卡尔曼滤波", styles["CNHeading2"]))
    story.append(
        p(
            "一维卡尔曼滤波把 Pitch 角和陀螺仪零偏作为状态，通过预测和观测校正估计角度。它比互补滤波多维护协方差矩阵，对噪声模型更敏感，但能同时修正陀螺仪偏置，理论上长期漂移更小。",
            styles["CNBody"],
        )
    )
    story.append(p("5.4 DMP", styles["CNHeading2"]))
    story.append(
        p(
            "DMP 是 MPU6050 内部数字运动处理器，可在芯片内部完成姿态融合并通过 FIFO 输出欧拉角/四元数。本实验调用公开 DMP 驱动完成初始化和读取。实测 DMP 数据稳定、噪声小，但输出频率约 11-13 Hz，动态幅值较互补和卡尔曼更小，体现出更强平滑和更慢动态响应。",
            styles["CNBody"],
        )
    )

    story.append(p("6 三种状态曲线与方法对比", styles["CNHeading1"]))
    cmp_rows = [["状态", "样本数", "DMP均频(Hz)", "acc范围", "comp范围", "kalman范围", "加速度满量程行"]]
    for key in ["STATIC", "TILT", "SHAKE"]:
        s = sums[key]
        cmp_rows.append(
            [
                key,
                str(s["samples"]),
                fmt(s["dmp_hz"]["mean"], 2),
                f"{fmt(s['pitch_acc']['min'], 1)}~{fmt(s['pitch_acc']['max'], 1)}",
                f"{fmt(s['pitch_comp']['min'], 1)}~{fmt(s['pitch_comp']['max'], 1)}",
                f"{fmt(s['pitch_kalman']['min'], 1)}~{fmt(s['pitch_kalman']['max'], 1)}",
                str(s["sat"]),
            ]
        )
    story.append(table(cmp_rows, [22 * mm, 20 * mm, 24 * mm, 28 * mm, 28 * mm, 30 * mm, 26 * mm], font_size=7.4))
    story.append(
        p(
            "倾斜状态下，Pitch 覆盖约 -55 deg 到 +58 deg，说明前后倾轴向修正有效。互补滤波与卡尔曼滤波基本跟随加速度角，但曲线更平滑；DMP 曲线变化方向一致，但幅值更小、更新更慢。",
            styles["CNBody"],
        )
    )
    story.append(image(PLOTS["TILT"], 165 * mm))

    story.append(PageBreak())
    story.append(p("7 快速晃动与误差分析", styles["CNHeading1"]))
    story.append(image(PLOTS["SHAKE"], 165 * mm))
    story.append(
        p(
            "快速晃动状态下，加速度计不仅测量重力，还包含明显线加速度。统计中 pitch_acc 标准差达到 52.387 deg，并出现 120 行加速度满量程，说明单独依赖加速度角会在强扰动下严重失真。互补滤波和卡尔曼滤波由于引入陀螺仪积分，Pitch 标准差分别降至 4.190 deg 和 4.324 deg，抗瞬时噪声能力更好。",
            styles["CNBody"],
        )
    )
    story.append(
        p(
            "从响应延迟看，互补滤波计算最直接，响应紧跟陀螺仪；卡尔曼滤波与互补滤波趋势接近，但由于估计偏置和协方差，短时变化略更平滑。DMP 输出最平滑，但在本次固件中 DMP FIFO 有间歇等待，dmp_hz 约 12 Hz，因此快速动作中的响应较慢、幅值较小。",
            styles["CNBody"],
        )
    )

    story.append(p("8 结论", styles["CNHeading1"]))
    conclusions = [
        ["结论点", "说明"],
        ["原始读取正确", "WHO_AM_I=0x68，三轴原始值与单位换算正常，静止时 az 接近 1g 量级。"],
        ["方向与轴向", "经首次实测修正后，前后倾对应 ay/gx，统一按前倾为负、后仰为正的实验约定输出。"],
        ["噪声", "静止下互补/卡尔曼 Pitch 标准差约 0.38 deg，满足课堂实验记录要求。"],
        ["响应", "倾斜曲线中互补和卡尔曼能跟随 -55~+58 deg 的动作，DMP更平滑但响应慢。"],
        ["漂移", "短时实验未观察到明显单调漂移；卡尔曼滤波通过估计陀螺仪偏置，在长时间记录中更有利于抑制漂移。"],
    ]
    story.append(table(conclusions, [35 * mm, 129 * mm], font_size=8.2))
    story.append(
        p(
            "综合来看，互补滤波适合嵌入式实时控制，卡尔曼滤波适合需要偏置估计和更严格噪声建模的场景，DMP适合快速获得平滑姿态但需要注意驱动配置、FIFO读取频率和动态响应。",
            styles["CNBody"],
        )
    )

    story.append(p("附录：构建与烧录记录", styles["CNHeading1"]))
    record = [
        ["项目", "结果"],
        ["Keil 编译", "0 Error(s), 0 Warning(s); Code=47798, RO-data=1002, RW-data=3136, ZI-data=2992"],
        ["固件 SHA256", "8BA3731C4D1786473556A2300DF706FF551BF481FE44B1F3F152A20AAB348789"],
        ["烧录", "COM7 Bootloader 写入，mass_erase=ok, write=ok, verification=ok"],
        ["备份", "烧录前完整读取 256 KiB Flash，备份文件保存在 backup 文件夹"],
        ["安全", "固件未初始化电机 PWM，不输出电机控制信号"],
    ]
    story.append(table(record, [35 * mm, 129 * mm], font_size=7.8))

    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(output_pdf)


if __name__ == "__main__":
    args = parse_args()
    build(args.student_name, args.student_id, args.output, args.board_image)
