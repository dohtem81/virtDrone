import argparse
import os
import re

import matplotlib.pyplot as plt
import pandas as pd


VALUE = r"([-+]?[0-9]*\.?[0-9]+)"

PATTERN = re.compile(
    rf"T:\s*{VALUE}s"
    rf".*?S/P Alt:\s*{VALUE}\s*/\s*{VALUE}m"
    rf".*?S/P RPM:\s*{VALUE}\s*/\s*{VALUE}"
    rf".*?S/P SOC:\s*{VALUE}\s*/\s*{VALUE}%"
    rf".*?S/P V:\s*{VALUE}\s*/\s*{VALUE}V"
    rf".*?S/P T:\s*{VALUE}\s*/\s*{VALUE}C"
    rf".*?Curr:\s*{VALUE}A"
    rf".*?Batt:\s*{VALUE}mAh"
    rf".*?TgtAlt:\s*{VALUE}m"
    rf".*?RefRPM:\s*{VALUE}"
    rf".*?TgtErr:\s*{VALUE}m"
    rf".*?P:\s*{VALUE}"
    rf".*?I:\s*{VALUE}"
    rf".*?D:\s*{VALUE}",
    re.IGNORECASE,
)


def parse_log(log_path: str) -> pd.DataFrame:
    rows = []
    with open(log_path, "r", encoding="utf-8") as log_file:
        for line in log_file:
            match = PATTERN.search(line)
            if not match:
                continue
            values = [float(item) for item in match.groups()]
            rows.append(values)

    columns = [
        "time_s",
        "alt_sensed_m",
        "alt_true_m",
        "rpm_sensed",
        "rpm_true",
        "soc_sensed",
        "soc_true",
        "voltage_sensed_v",
        "voltage_true_v",
        "temp_sensed_c",
        "temp_true_c",
        "current_a",
        "battery_mah",
        "target_alt_m",
        "ref_rpm",
        "target_err_m",
        "p_term",
        "i_term",
        "d_term",
    ]
    return pd.DataFrame(rows, columns=columns)


def build_full_dashboard(df: pd.DataFrame, output_path: str) -> None:
    fig, axs = plt.subplots(6, 1, figsize=(16, 20), sharex=True)

    fig.suptitle("virtDrone Flight Dashboard", fontsize=16, fontweight="bold")
    fig.text(0.02, 0.94, "Sensors + Altitude", fontsize=12, fontweight="bold")
    fig.text(0.02, 0.40, "Altitude Controller State", fontsize=12, fontweight="bold")

    axs[0].plot(df["time_s"], df["alt_sensed_m"], label="Altitude sensed")
    axs[0].plot(df["time_s"], df["alt_true_m"], label="Altitude true")
    axs[0].plot(df["time_s"], df["target_alt_m"], "--", label="Target altitude")
    axs[0].set_ylabel("Altitude [m]")
    axs[0].legend(loc="best")
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(df["time_s"], df["rpm_sensed"], label="RPM sensed")
    axs[1].plot(df["time_s"], df["rpm_true"], label="RPM true")
    axs[1].plot(df["time_s"], df["ref_rpm"], "--", label="RPM reference")
    axs[1].set_ylabel("RPM")
    axs[1].legend(loc="best")
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(df["time_s"], df["voltage_sensed_v"], label="Voltage sensed")
    axs[2].plot(df["time_s"], df["voltage_true_v"], label="Voltage true")
    axs[2].plot(df["time_s"], df["current_a"], label="Current [A]")
    axs[2].set_ylabel("V / A")
    axs[2].legend(loc="best")
    axs[2].grid(True, alpha=0.3)

    axs[3].plot(df["time_s"], df["soc_sensed"], label="SOC sensed [%]")
    axs[3].plot(df["time_s"], df["soc_true"], label="SOC true [%]")
    axs[3].plot(df["time_s"], df["battery_mah"], label="Battery [mAh]")
    axs[3].plot(df["time_s"], df["temp_sensed_c"], label="Temp sensed [C]")
    axs[3].plot(df["time_s"], df["temp_true_c"], label="Temp true [C]")
    axs[3].set_ylabel("Energy / Temp")
    axs[3].legend(loc="best", ncol=3)
    axs[3].grid(True, alpha=0.3)

    axs[4].plot(df["time_s"], df["target_err_m"], label="Target error [m]")
    axs[4].set_ylabel("Error [m]")
    axs[4].legend(loc="best")
    axs[4].grid(True, alpha=0.3)

    axs[5].plot(df["time_s"], df["p_term"], label="P")
    axs[5].plot(df["time_s"], df["i_term"], label="I")
    axs[5].plot(df["time_s"], df["d_term"], label="D")
    axs[5].set_ylabel("Controller terms")
    axs[5].set_xlabel("Time [s]")
    axs[5].legend(loc="best")
    axs[5].grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig.savefig(output_path, dpi=180)


def build_minimal_dashboard(df: pd.DataFrame, output_path: str) -> None:
    fig, axs = plt.subplots(2, 1, figsize=(16, 10), sharex=True)

    fig.suptitle("virtDrone Minimal Flight Dashboard", fontsize=16, fontweight="bold")
    fig.text(0.02, 0.92, "Sensors + Altitude", fontsize=12, fontweight="bold")
    fig.text(0.02, 0.47, "Altitude Controller State", fontsize=12, fontweight="bold")

    axs[0].plot(df["time_s"], df["alt_sensed_m"], label="Altitude sensed")
    axs[0].plot(df["time_s"], df["alt_true_m"], label="Altitude true")
    axs[0].plot(df["time_s"], df["target_alt_m"], "--", label="Target altitude")
    axs[0].set_ylabel("Altitude [m]")
    axs[0].legend(loc="best")
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(df["time_s"], df["target_err_m"], label="Target error [m]")
    axs[1].plot(df["time_s"], df["p_term"], label="P")
    axs[1].plot(df["time_s"], df["i_term"], label="I")
    axs[1].plot(df["time_s"], df["d_term"], label="D")
    axs[1].set_ylabel("Controller")
    axs[1].set_xlabel("Time [s]")
    axs[1].legend(loc="best")
    axs[1].grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig.savefig(output_path, dpi=180)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate flight dashboard from simulator log")
    parser.add_argument("--input", default="test.txt", help="Path to simulator log file")
    parser.add_argument(
        "--output",
        default="docs/tutorials/charts/flight_dashboard.png",
        help="Output PNG path",
    )
    parser.add_argument(
        "--mode",
        choices=["full", "minimal"],
        default="full",
        help="Chart mode: full (all flight params) or minimal (sensors+altitude and controller state)",
    )
    args = parser.parse_args()

    dataframe = parse_log(args.input)
    if dataframe.empty:
        raise ValueError(f"No telemetry rows parsed from {args.input}")

    if args.mode == "minimal":
        build_minimal_dashboard(dataframe, args.output)
    else:
        build_full_dashboard(dataframe, args.output)


if __name__ == "__main__":
    main()