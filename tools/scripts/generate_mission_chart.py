import argparse
import re
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


STATUS_TO_CODE = {
    "IDLE": 0,
    "RUNNING": 1,
    "PAUSED": 2,
    "COMPLETED": 3,
    "ABORTED": 4,
    "FAILED": 5,
}

CODE_TO_STATUS = {value: key for key, value in STATUS_TO_CODE.items()}


def load_telemetry(telemetry_csv: str) -> pd.DataFrame:
    df = pd.read_csv(telemetry_csv)
    required_columns = [
        "sim_elapsed_s",
        "position_enu_x_m",
        "position_enu_y_m",
        "position_enu_z_m",
        "target_altitude_m",
        "yaw_rad",
        "pitch_rad",
        "roll_rad",
        "desired_motor_rpm_0",
        "desired_motor_rpm_1",
        "desired_motor_rpm_2",
        "desired_motor_rpm_3",
    ]
    missing = [column for column in required_columns if column not in df.columns]
    if missing:
        raise ValueError(f"Missing required telemetry columns: {missing}")

    return df.sort_values("sim_elapsed_s").reset_index(drop=True)


def load_events(events_log: str) -> pd.DataFrame:
    rows = []
    for raw_line in Path(events_log).read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(",", 2)
        if len(parts) != 3:
            continue
        local_timestamp, sim_elapsed, message = parts
        try:
            sim_elapsed_s = float(sim_elapsed)
        except ValueError:
            continue
        rows.append(
            {
                "local_timestamp": local_timestamp,
                "sim_elapsed_s": sim_elapsed_s,
                "message": message,
            }
        )

    return pd.DataFrame(rows).sort_values("sim_elapsed_s").reset_index(drop=True)


def parse_mission_targets(
    mission_yaml: str | None,
) -> Tuple[
    Dict[int, float],
    Dict[int, float],
    Dict[int, float],
    Dict[int, float],
    Dict[int, float],
    Dict[int, float],
    float,
    float,
    float,
    float,
    float,
    float,
]:
    if mission_yaml is None:
        return {}, {}, {}, {}, {}, {}, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan

    mission_doc = yaml.safe_load(Path(mission_yaml).read_text(encoding="utf-8"))
    mission = mission_doc.get("mission", {})
    initial = mission.get("initial_conditions", {})
    initial_pos = initial.get("position_enu_m", {})

    initial_x = float(initial_pos.get("x", np.nan))
    initial_y = float(initial_pos.get("y", np.nan))
    initial_z = float(initial.get("altitude_m", np.nan))
    initial_yaw = float(initial.get("yaw_rad", np.nan))
    initial_pitch = 0.0
    initial_roll = 0.0

    x_targets: Dict[int, float] = {}
    y_targets: Dict[int, float] = {}
    z_targets: Dict[int, float] = {}
    yaw_targets: Dict[int, float] = {}
    pitch_targets: Dict[int, float] = {}
    roll_targets: Dict[int, float] = {}

    for step in mission.get("steps", []):
        step_id = int(step.get("step_id", -1))
        if step_id < 0:
            continue
        action = str(step.get("action", "")).strip().lower()

        if action == "go_to_position":
            target = step.get("target_position_enu_m", {})
            x_targets[step_id] = float(target.get("x", np.nan))
            y_targets[step_id] = float(target.get("y", np.nan))
            z_targets[step_id] = float(step.get("target_altitude_m", np.nan))
        elif action in {"hover", "change_altitude"}:
            z_targets[step_id] = float(step.get("target_altitude_m", np.nan))
            if "yaw_rad" in step:
                yaw_targets[step_id] = float(step.get("yaw_rad", np.nan))
        elif action == "land":
            z_targets[step_id] = 0.0
        elif action == "set_attitude":
            if "target_yaw_rad" in step:
                yaw_targets[step_id] = float(step.get("target_yaw_rad", np.nan))
            if "target_pitch_rad" in step:
                pitch_targets[step_id] = float(step.get("target_pitch_rad", np.nan))
            if "target_roll_rad" in step:
                roll_targets[step_id] = float(step.get("target_roll_rad", np.nan))
        elif action == "rotate_yaw":
            if "target_yaw_rad" in step:
                yaw_targets[step_id] = float(step.get("target_yaw_rad", np.nan))

    return (
        x_targets,
        y_targets,
        z_targets,
        yaw_targets,
        pitch_targets,
        roll_targets,
        initial_x,
        initial_y,
        initial_z,
        initial_yaw,
        initial_pitch,
        initial_roll,
    )


def build_step_and_status_series(
    telemetry_time_s: np.ndarray,
    events_df: pd.DataFrame,
    x_targets: Dict[int, float],
    y_targets: Dict[int, float],
    z_targets: Dict[int, float],
    yaw_targets: Dict[int, float],
    pitch_targets: Dict[int, float],
    roll_targets: Dict[int, float],
    initial_x: float,
    initial_y: float,
    initial_z: float,
    initial_yaw: float,
    initial_pitch: float,
    initial_roll: float,
    z_fallback: np.ndarray,
) -> Tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    step_series = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)
    status_series = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)

    x_ref = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)
    y_ref = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)
    z_ref = np.array(z_fallback, dtype=float)
    yaw_ref = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)
    pitch_ref = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)
    roll_ref = np.full_like(telemetry_time_s, fill_value=np.nan, dtype=float)

    current_step = np.nan
    current_status = STATUS_TO_CODE["IDLE"]

    current_x_ref = initial_x
    current_y_ref = initial_y
    current_z_ref = initial_z
    current_yaw_ref = initial_yaw
    current_pitch_ref = initial_pitch
    current_roll_ref = initial_roll

    step_regex = re.compile(r"MISSION_STEP\s+step_id=(\-?\d+)")
    status_regex = re.compile(r"MISSION_STATUS\s+status=([A-Z_]+)")

    event_index = 0
    total_events = len(events_df)

    for i, time_s in enumerate(telemetry_time_s):
        while event_index < total_events and events_df.iloc[event_index]["sim_elapsed_s"] <= time_s + 1e-9:
            message = str(events_df.iloc[event_index]["message"])

            step_match = step_regex.search(message)
            if step_match:
                current_step = float(int(step_match.group(1)))
                step_id = int(current_step)
                if step_id in x_targets:
                    current_x_ref = x_targets[step_id]
                if step_id in y_targets:
                    current_y_ref = y_targets[step_id]
                if step_id in z_targets:
                    current_z_ref = z_targets[step_id]
                if step_id in yaw_targets:
                    current_yaw_ref = yaw_targets[step_id]
                if step_id in pitch_targets:
                    current_pitch_ref = pitch_targets[step_id]
                if step_id in roll_targets:
                    current_roll_ref = roll_targets[step_id]

            status_match = status_regex.search(message)
            if status_match:
                status_name = status_match.group(1)
                current_status = STATUS_TO_CODE.get(status_name, np.nan)

            event_index += 1

        step_series[i] = current_step
        status_series[i] = current_status
        x_ref[i] = current_x_ref
        y_ref[i] = current_y_ref
        if not np.isnan(current_z_ref):
            z_ref[i] = current_z_ref
        yaw_ref[i] = current_yaw_ref
        pitch_ref[i] = current_pitch_ref
        roll_ref[i] = current_roll_ref

    return step_series, status_series, x_ref, y_ref, z_ref, yaw_ref, pitch_ref, roll_ref


def plot_mission_chart(
    telemetry_df: pd.DataFrame,
    step_series: np.ndarray,
    status_series: np.ndarray,
    x_ref: np.ndarray,
    y_ref: np.ndarray,
    z_ref: np.ndarray,
    yaw_ref: np.ndarray,
    pitch_ref: np.ndarray,
    roll_ref: np.ndarray,
    output_path: str,
) -> None:
    time_s = telemetry_df["sim_elapsed_s"].to_numpy()
    x = telemetry_df["position_enu_x_m"].to_numpy()
    y = telemetry_df["position_enu_y_m"].to_numpy()
    z = telemetry_df["position_enu_z_m"].to_numpy()
    yaw = telemetry_df["yaw_rad"].to_numpy()
    pitch = telemetry_df["pitch_rad"].to_numpy()
    roll = telemetry_df["roll_rad"].to_numpy()
    motor0 = telemetry_df["desired_motor_rpm_0"].to_numpy()
    motor1 = telemetry_df["desired_motor_rpm_1"].to_numpy()
    motor2 = telemetry_df["desired_motor_rpm_2"].to_numpy()
    motor3 = telemetry_df["desired_motor_rpm_3"].to_numpy()

    fig, axs = plt.subplots(9, 1, figsize=(16, 25), sharex=True)

    axs[0].plot(time_s, x, label="X actual", color="tab:blue")
    axs[0].plot(time_s, x_ref, "--", label="X ref", color="tab:blue", alpha=0.8)
    axs[0].set_ylabel("X [m]")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="best")

    axs[1].plot(time_s, y, label="Y actual", color="tab:orange")
    axs[1].plot(time_s, y_ref, "--", label="Y ref", color="tab:orange", alpha=0.8)
    axs[1].set_ylabel("Y [m]")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="best")

    axs[2].plot(time_s, z, label="Z actual", color="tab:green")
    axs[2].plot(time_s, z_ref, "--", label="Z ref", color="tab:green", alpha=0.8)
    axs[2].set_ylabel("Z [m]")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc="best")

    axs[3].plot(time_s, yaw, label="Yaw actual", color="tab:purple")
    axs[3].plot(time_s, yaw_ref, "--", label="Yaw ref", color="tab:purple", alpha=0.8)
    axs[3].set_ylabel("Yaw [rad]")
    axs[3].grid(True, alpha=0.3)
    axs[3].legend(loc="best")

    axs[4].plot(time_s, pitch, label="Pitch actual", color="tab:brown")
    axs[4].plot(time_s, pitch_ref, "--", label="Pitch ref", color="tab:brown", alpha=0.8)
    axs[4].set_ylabel("Pitch [rad]")
    axs[4].grid(True, alpha=0.3)
    axs[4].legend(loc="best")

    axs[5].plot(time_s, roll, label="Roll actual", color="tab:cyan")
    axs[5].plot(time_s, roll_ref, "--", label="Roll ref", color="tab:cyan", alpha=0.8)
    axs[5].set_ylabel("Roll [rad]")
    axs[5].grid(True, alpha=0.3)
    axs[5].legend(loc="best")

    axs[6].step(time_s, step_series, where="post", label="Mission step_id", color="tab:olive")
    axs[6].set_ylabel("Step ID")
    axs[6].grid(True, alpha=0.3)
    axs[6].legend(loc="best")

    axs[7].step(time_s, status_series, where="post", label="Mission status", color="tab:red")
    axs[7].set_yticks(list(CODE_TO_STATUS.keys()))
    axs[7].set_yticklabels([CODE_TO_STATUS[code] for code in CODE_TO_STATUS.keys()])
    axs[7].set_ylabel("Status")
    axs[7].grid(True, alpha=0.3)
    axs[7].legend(loc="best")

    axs[8].plot(time_s, motor0, label="M0 ref", color="tab:blue")
    axs[8].plot(time_s, motor1, label="M1 ref", color="tab:orange")
    axs[8].plot(time_s, motor2, label="M2 ref", color="tab:green")
    axs[8].plot(time_s, motor3, label="M3 ref", color="tab:red")
    axs[8].set_ylabel("Motor RPM")
    axs[8].set_xlabel("Simulation time [s]")
    axs[8].grid(True, alpha=0.3)
    axs[8].legend(loc="best")

    fig.suptitle("Mission Tracking: XYZ/YPR vs References + Sequence Status + Motor RPM", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate mission chart (XYZ/YPR actual vs references + mission status/step)"
    )
    parser.add_argument("--telemetry", default="docs/tutorials/simulation_telemetry.csv", help="Path to telemetry CSV")
    parser.add_argument("--events", default="docs/tutorials/simulation_events.log", help="Path to simulation events log")
    parser.add_argument("--mission", default=None, help="Optional mission YAML path for XY/Z references")
    parser.add_argument(
        "--output",
        default="docs/tutorials/charts/mission_xyz_status.png",
        help="Output PNG path",
    )
    args = parser.parse_args()

    telemetry_df = load_telemetry(args.telemetry)
    events_df = load_events(args.events)

    (
        x_targets,
        y_targets,
        z_targets,
        yaw_targets,
        pitch_targets,
        roll_targets,
        initial_x,
        initial_y,
        initial_z,
        initial_yaw,
        initial_pitch,
        initial_roll,
    ) = parse_mission_targets(args.mission)

    step_series, status_series, x_ref, y_ref, z_ref, yaw_ref, pitch_ref, roll_ref = build_step_and_status_series(
        telemetry_df["sim_elapsed_s"].to_numpy(),
        events_df,
        x_targets,
        y_targets,
        z_targets,
        yaw_targets,
        pitch_targets,
        roll_targets,
        initial_x,
        initial_y,
        initial_z,
        initial_yaw,
        initial_pitch,
        initial_roll,
        telemetry_df["target_altitude_m"].to_numpy(),
    )

    plot_mission_chart(
        telemetry_df,
        step_series,
        status_series,
        x_ref,
        y_ref,
        z_ref,
        yaw_ref,
        pitch_ref,
        roll_ref,
        args.output,
    )


if __name__ == "__main__":
    main()
