import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Plot MPU6050 attitude curves from captured CSV.")
    parser.add_argument("csv_file", help="path to MPU6050_*.csv")
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    rows = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("record") == "A":
                rows.append(row)

    if not rows:
        raise SystemExit("no A records found")

    t0 = float(rows[0]["ms"])
    t = [(float(row["ms"]) - t0) / 1000.0 for row in rows]
    pitch_acc = [float(row["pitch_acc"]) for row in rows]
    pitch_dmp = [float(row["pitch_dmp"]) for row in rows]
    pitch_comp = [float(row["pitch_comp"]) for row in rows]
    pitch_kalman = [float(row["pitch_kalman"]) for row in rows]

    import matplotlib.pyplot as plt

    plt.figure(figsize=(11, 6), dpi=140)
    plt.plot(t, pitch_dmp, label="DMP Pitch", linewidth=1.2)
    plt.plot(t, pitch_comp, label="Complementary", linewidth=1.0)
    plt.plot(t, pitch_kalman, label="Kalman", linewidth=1.0)
    plt.plot(t, pitch_acc, label="Accel only", linewidth=0.8, alpha=0.55)
    plt.xlabel("Time (s)")
    plt.ylabel("Pitch (deg)")
    plt.title(f"MPU6050 attitude comparison - {csv_path.stem}")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()

    out_path = csv_path.with_suffix(".png")
    plt.savefig(out_path)
    print(out_path)


if __name__ == "__main__":
    main()
