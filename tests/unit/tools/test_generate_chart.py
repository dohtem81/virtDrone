import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = REPO_ROOT / "tools" / "scripts" / "generate_chart.py"
SPEC = importlib.util.spec_from_file_location("generate_chart", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _line_minimal() -> str:
    return (
        "T: 1.00s"
        " | S/P Alt: 1.00 / 2.00m"
        " | S/P RPM: 3.00 / 4.00"
        " | S/P SOC: 5.00 / 6.00%"
        " | S/P V: 7.00 / 8.00V"
        " | S/P T: 9.00 / 10.00C"
        " | Curr: 11.00A"
        " | Batt: 12.00mAh"
        " | TgtAlt: 13.00m"
        " | RefRPM: 14.00"
        " | TgtErr: 15.00m"
        " | P: 16.00"
        " | I: 17.00"
        " | D: 18.00"
    )


def _line_full() -> str:
    return (
        _line_minimal()
        + " | PosENU: ( 19.00, 20.00, 21.00)m"
        + " | VelENU: ( 22.00, 23.00, 24.00)m/s"
        + " | YPR: ( 25.00, 26.00, 27.00)rad"
        + " | S/P GPSPos: ( 52.10, 21.20, 101.00) / ( 52.11, 21.21, 102.00)"
        + " | S/P GPSVel: ( 1.10, 2.20, 3.30) / ( 1.00, 2.00, 3.00)m/s"
        + " | WTotAcc: ( 0.10, 0.20, 0.30)m/s2"
        + " | WSteady: ( 0.40, 0.50, 0.60)"
        + " | WGust: ( 0.70, 0.80, 0.90)"
        + " | WTurb: ( 1.10, 1.20, 1.30)"
    )


def test_parse_log_parses_minimal_line(tmp_path: Path) -> None:
    log_path = tmp_path / "log.txt"
    log_path.write_text(_line_minimal() + "\n", encoding="utf-8")

    df = MODULE.parse_log(str(log_path))

    assert len(df) == 1
    assert pytest.approx(df.iloc[0]["time_s"], rel=1e-9) == 1.0
    assert pytest.approx(df.iloc[0]["d_term"], rel=1e-9) == 18.0
    assert df.iloc[0]["pos_e_m"] != df.iloc[0]["pos_e_m"]  # NaN
    assert df.iloc[0]["gps_lat_sensed_deg"] != df.iloc[0]["gps_lat_sensed_deg"]  # NaN


def test_parse_log_parses_extended_enu_and_gps_blocks(tmp_path: Path) -> None:
    log_path = tmp_path / "log.txt"
    log_path.write_text(_line_full() + "\n", encoding="utf-8")

    df = MODULE.parse_log(str(log_path))

    assert len(df) == 1
    assert pytest.approx(df.iloc[0]["pos_e_m"], rel=1e-9) == 19.0
    assert pytest.approx(df.iloc[0]["vel_u_mps"], rel=1e-9) == 24.0
    assert pytest.approx(df.iloc[0]["yaw_rad"], rel=1e-9) == 25.0
    assert pytest.approx(df.iloc[0]["gps_lat_sensed_deg"], rel=1e-9) == 52.10
    assert pytest.approx(df.iloc[0]["gps_lon_true_deg"], rel=1e-9) == 21.21
    assert pytest.approx(df.iloc[0]["gps_vel_d_true_mps"], rel=1e-9) == 3.0
    assert pytest.approx(df.iloc[0]["weather_total_acc_e_mps2"], rel=1e-9) == 0.10
    assert pytest.approx(df.iloc[0]["weather_steady_acc_n_mps2"], rel=1e-9) == 0.50
    assert pytest.approx(df.iloc[0]["weather_gust_acc_u_mps2"], rel=1e-9) == 0.90
    assert pytest.approx(df.iloc[0]["weather_turb_acc_u_mps2"], rel=1e-9) == 1.30


def test_parse_log_skips_non_matching_lines(tmp_path: Path) -> None:
    log_path = tmp_path / "log.txt"
    log_path.write_text("not telemetry\n" + _line_minimal() + "\n", encoding="utf-8")

    df = MODULE.parse_log(str(log_path))

    assert len(df) == 1


def test_parse_log_reads_utf16_input(tmp_path: Path) -> None:
    log_path = tmp_path / "log_utf16.txt"
    log_path.write_text(_line_minimal() + "\n", encoding="utf-16")

    df = MODULE.parse_log(str(log_path))

    assert len(df) == 1
    assert pytest.approx(df.iloc[0]["alt_true_m"], rel=1e-9) == 2.0
