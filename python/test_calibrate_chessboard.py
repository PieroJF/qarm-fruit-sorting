"""
Unit + integration tests for the D1 chessboard-calibration subsystem.

Run with:
    py -3.13 python/test_calibrate_chessboard.py
Exit code is the number of failed sections (0 = all pass).
"""
import os
import sys
import json
import tempfile
import traceback
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

_RESULTS = []


def _section(name, fn):
    try:
        fn()
        _RESULTS.append((name, True, None))
        print(f"  [PASS] {name}")
    except Exception as ex:
        _RESULTS.append((name, False, ex))
        print(f"  [FAIL] {name}: {ex}")
        traceback.print_exc()


# ========================================================================
# A1. session_cal IO
# ========================================================================
def test_session_cal_roundtrip():
    from session_cal import SessionCal

    cal = SessionCal(
        timestamp="2026-04-22T14:30:00",
        chess_origin_in_base_m=np.array([0.30, 0.10, 0.00]),
        h_pixel_to_chess_mm=np.eye(3),
        survey_pose_joints_rad=np.array([0.1, -0.2, 0.3, 0.0]),
        chess_pattern={"cols": 7, "rows": 5, "square_mm": 30,
                       "inner_cols": 6, "inner_rows": 4},
        d415_intrinsics={"fx": 912.6, "fy": 911.3,
                          "cx": 635.4, "cy": 343.0},
        homography_reproj_rms_px=0.45,
        camera_height_above_table_m=0.60,
        image_size=(1280, 720),
    )

    with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                      delete=False) as f:
        path = f.name
    try:
        cal.save(path)
        roundtrip = SessionCal.load(path)
        assert np.allclose(roundtrip.chess_origin_in_base_m,
                            cal.chess_origin_in_base_m)
        assert np.allclose(roundtrip.h_pixel_to_chess_mm,
                            cal.h_pixel_to_chess_mm)
        assert roundtrip.timestamp == cal.timestamp
        assert roundtrip.chess_pattern["square_mm"] == 30
        assert roundtrip.image_size == (1280, 720)
    finally:
        os.unlink(path)


def test_session_cal_missing_file_raises():
    from session_cal import SessionCal
    try:
        SessionCal.load("/no/such/path.json")
        assert False, "should have raised"
    except FileNotFoundError:
        pass


if __name__ == "__main__":
    _section("A1 session_cal roundtrip", test_session_cal_roundtrip)
    _section("A1 session_cal missing file", test_session_cal_missing_file_raises)
    fails = sum(1 for _, ok, _ in _RESULTS if not ok)
    print(f"\n{len(_RESULTS)} test(s), {fails} failed")
    sys.exit(fails)
