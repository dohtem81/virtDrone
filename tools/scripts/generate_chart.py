import argparse
import os
import re
from pathlib import Path

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
    rf".*?D:\s*{VALUE}"
    rf"(?:.*?PosENU:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)m"
    rf".*?VelENU:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)m/s"
    rf".*?YPR:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)rad)?"
    rf"(?:.*?S/P GPSPos:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)\s*/\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)"
    rf".*?S/P GPSVel:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)\s*/\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)m/s)?"
    rf"(?:.*?WTotAcc:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)m/s2"
    rf".*?WSteady:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)"
    rf".*?WGust:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\)"
    rf".*?WTurb:\s*\(\s*{VALUE}\s*,\s*{VALUE}\s*,\s*{VALUE}\s*\))?",
    re.IGNORECASE,
)


def parse_log(log_path: str) -> pd.DataFrame:
    rows = []
    log_lines = _read_log_lines(log_path)
    for line in log_lines:
        match = PATTERN.search(line)
        if not match:
            continue
        values = [float(item) if item is not None else float("nan") for item in match.groups()]
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
        "pos_e_m",
        "pos_n_m",
        "pos_u_m",
        "vel_e_mps",
        "vel_n_mps",
        "vel_u_mps",
        "yaw_rad",
        "pitch_rad",
        "roll_rad",
        "gps_lat_sensed_deg",
        "gps_lon_sensed_deg",
        "gps_alt_sensed_m",
        "gps_lat_true_deg",
        "gps_lon_true_deg",
        "gps_alt_true_m",
        "gps_vel_n_sensed_mps",
        "gps_vel_e_sensed_mps",
        "gps_vel_d_sensed_mps",
        "gps_vel_n_true_mps",
        "gps_vel_e_true_mps",
        "gps_vel_d_true_mps",
        "weather_total_acc_e_mps2",
        "weather_total_acc_n_mps2",
        "weather_total_acc_u_mps2",
        "weather_steady_acc_e_mps2",
        "weather_steady_acc_n_mps2",
        "weather_steady_acc_u_mps2",
        "weather_gust_acc_e_mps2",
        "weather_gust_acc_n_mps2",
        "weather_gust_acc_u_mps2",
        "weather_turb_acc_e_mps2",
        "weather_turb_acc_n_mps2",
        "weather_turb_acc_u_mps2",
    ]
    return pd.DataFrame(rows, columns=columns)


def _read_log_lines(log_path: str) -> list[str]:
    raw_bytes = Path(log_path).read_bytes()
    for encoding in ("utf-8", "utf-8-sig", "utf-16", "utf-16-le", "utf-16-be"):
        try:
            return raw_bytes.decode(encoding).splitlines()
        except UnicodeDecodeError:
            continue

    return raw_bytes.decode("utf-8", errors="replace").splitlines()


def build_full_dashboard(df: pd.DataFrame, output_path: str) -> None:
    fig, axs = plt.subplots(9, 1, figsize=(16, 29), sharex=True)

    fig.suptitle("virtDrone Flight Dashboard", fontsize=16, fontweight="bold")
    fig.text(0.02, 0.95, "Sensors + Altitude", fontsize=12, fontweight="bold")
    fig.text(0.02, 0.49, "Altitude Controller + GPS State", fontsize=12, fontweight="bold")

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

    energy_ax = axs[3]
    temp_ax = energy_ax.twinx()
    energy_ax.plot(df["time_s"], df["soc_sensed"], label="SOC sensed [%]", color="tab:blue")
    energy_ax.plot(df["time_s"], df["soc_true"], label="SOC true [%]", color="tab:cyan")
    energy_ax.plot(df["time_s"], df["battery_mah"], label="Battery [mAh]", color="tab:green")
    temp_ax.plot(df["time_s"], df["temp_sensed_c"], label="Temp sensed [C]", color="tab:red")
    temp_ax.plot(df["time_s"], df["temp_true_c"], label="Temp true [C]", color="tab:orange")
    energy_ax.set_ylabel("Energy [% / mAh]")
    temp_ax.set_ylabel("Temperature [°C]")
    energy_ax.grid(True, alpha=0.3)
    energy_handles, energy_labels = energy_ax.get_legend_handles_labels()
    temp_handles, temp_labels = temp_ax.get_legend_handles_labels()
    energy_ax.legend(energy_handles + temp_handles, energy_labels + temp_labels, loc="best", ncol=3)

    axs[4].plot(df["time_s"], df["target_err_m"], label="Target error [m]")
    axs[4].set_ylabel("Error [m]")
    axs[4].legend(loc="best")
    axs[4].grid(True, alpha=0.3)

    axs[5].plot(df["time_s"], df["p_term"], label="P")
    axs[5].plot(df["time_s"], df["i_term"], label="I")
    axs[5].plot(df["time_s"], df["d_term"], label="D")
    axs[5].set_ylabel("Controller terms")
    axs[5].legend(loc="best")
    axs[5].grid(True, alpha=0.3)

    if not df["gps_alt_sensed_m"].isna().all():
        axs[6].plot(df["time_s"], df["gps_alt_sensed_m"], label="GPS alt sensed")
        axs[6].plot(df["time_s"], df["gps_alt_true_m"], label="GPS alt true")
        axs[6].plot(df["time_s"], df["gps_lat_sensed_deg"], label="GPS lat sensed")
        axs[6].plot(df["time_s"], df["gps_lat_true_deg"], label="GPS lat true")
        axs[6].plot(df["time_s"], df["gps_lon_sensed_deg"], label="GPS lon sensed")
        axs[6].plot(df["time_s"], df["gps_lon_true_deg"], label="GPS lon true")
        axs[6].legend(loc="best", ncol=3)
    axs[6].set_ylabel("GPS pos")
    axs[6].grid(True, alpha=0.3)

    if not df["gps_vel_n_sensed_mps"].isna().all():
        axs[7].plot(df["time_s"], df["gps_vel_n_sensed_mps"], label="GPS Vn sensed")
        axs[7].plot(df["time_s"], df["gps_vel_n_true_mps"], label="GPS Vn true")
        axs[7].plot(df["time_s"], df["gps_vel_e_sensed_mps"], label="GPS Ve sensed")
        axs[7].plot(df["time_s"], df["gps_vel_e_true_mps"], label="GPS Ve true")
        axs[7].plot(df["time_s"], df["gps_vel_d_sensed_mps"], label="GPS Vd sensed")
        axs[7].plot(df["time_s"], df["gps_vel_d_true_mps"], label="GPS Vd true")
        axs[7].legend(loc="best", ncol=3)
    axs[7].set_ylabel("GPS vel [m/s]")

    if not df["weather_total_acc_e_mps2"].isna().all():
        axs[8].plot(df["time_s"], df["weather_total_acc_e_mps2"], label="W total E", color="tab:blue")
        axs[8].plot(df["time_s"], df["weather_total_acc_n_mps2"], label="W total N", color="tab:orange")
        axs[8].plot(df["time_s"], df["weather_total_acc_u_mps2"], label="W total U", color="tab:green")

        axs[8].plot(df["time_s"], df["weather_steady_acc_e_mps2"], "--", label="W steady E", color="tab:blue")
        axs[8].plot(df["time_s"], df["weather_steady_acc_n_mps2"], "--", label="W steady N", color="tab:orange")
        axs[8].plot(df["time_s"], df["weather_steady_acc_u_mps2"], "--", label="W steady U", color="tab:green")

        axs[8].plot(df["time_s"], df["weather_gust_acc_e_mps2"], ":", label="W gust E", color="tab:blue")
        axs[8].plot(df["time_s"], df["weather_gust_acc_n_mps2"], ":", label="W gust N", color="tab:orange")
        axs[8].plot(df["time_s"], df["weather_gust_acc_u_mps2"], ":", label="W gust U", color="tab:green")

        axs[8].plot(df["time_s"], df["weather_turb_acc_e_mps2"], "-.", label="W turb E", color="tab:blue")
        axs[8].plot(df["time_s"], df["weather_turb_acc_n_mps2"], "-.", label="W turb N", color="tab:orange")
        axs[8].plot(df["time_s"], df["weather_turb_acc_u_mps2"], "-.", label="W turb U", color="tab:green")

        axs[8].legend(loc="best", ncol=3)
    axs[8].set_ylabel("Weather [m/s²]")
    axs[8].grid(True, alpha=0.3)

    axs[8].set_xlabel("Time [s]")
    axs[7].grid(True, alpha=0.3)

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