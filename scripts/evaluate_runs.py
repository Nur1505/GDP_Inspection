#!/usr/bin/env python3
import os
import math
import argparse
import pandas as pd

def safe_stats(series):
    s = pd.to_numeric(series, errors="coerce").dropna()
    if len(s) == 0:
        return {
            "count": 0,
            "mean": float("nan"),
            "std": float("nan"),
            "median": float("nan"),
            "min": float("nan"),
            "max": float("nan")
        }
    return {
        "count": int(len(s)),
        "mean": float(s.mean()),
        "std": float(s.std(ddof=1)) if len(s) > 1 else 0.0,
        "median": float(s.median()),
        "min": float(s.min()),
        "max": float(s.max())
    }

def print_metric(name, stats, unit=""):
    print("\n%s" % name)
    print("  count  : %d" % stats["count"])
    print("  mean   : %.4f %s" % (stats["mean"], unit))
    print("  std    : %.4f %s" % (stats["std"], unit))
    print("  median : %.4f %s" % (stats["median"], unit))
    print("  min    : %.4f %s" % (stats["min"], unit))
    print("  max    : %.4f %s" % (stats["max"], unit))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results_dir", required=True)
    args = parser.parse_args()

    uav1_csv = os.path.join(args.results_dir, "run_summary.csv")
    uav2_csv = os.path.join(args.results_dir, "run_summary_uav2.csv")

    if not os.path.exists(uav1_csv):
        print("Missing:", uav1_csv)
        return
    if not os.path.exists(uav2_csv):
        print("Missing:", uav2_csv)
        return

    df1 = pd.read_csv(uav1_csv)
    df2 = pd.read_csv(uav2_csv)

    print("\n==============================")
    print("UAV1 / MAPPING / FOI RESULTS")
    print("==============================")

    print_metric("UAV1 RMS position error", safe_stats(df1["uav1_rms_position_error_m"]), "m")
    print_metric("UAV1 mean position error", safe_stats(df1["uav1_mean_position_error_m"]), "m")
    print_metric("UAV1 max position error", safe_stats(df1["uav1_max_position_error_m"]), "m")
    print_metric("UAV1 final drift", safe_stats(df1["uav1_final_drift_m"]), "m")
    print_metric("Map completeness", safe_stats(df1["map_completeness_percent"]), "%")
    print_metric("FOI position error", safe_stats(df1["foi_position_error_m"]), "m")

    foi_success = pd.to_numeric(df1["foi_detection_success"], errors="coerce").fillna(0)
    print("\nFOI detection success rate: %.2f%%" % (100.0 * foi_success.mean()))

    print("\n==============================")
    print("UAV2 INSPECTION RESULTS")
    print("==============================")

    print_metric("UAV2 hover error to commanded FOI", safe_stats(df2["uav2_hover_error_to_commanded_foi_m"]), "m")
    print_metric("UAV2 hover error to GT FOI", safe_stats(df2["uav2_hover_error_to_gt_foi_m"]), "m")
    print_metric("UAV2 final home error", safe_stats(df2["uav2_final_home_error_m"]), "m")
    print_metric("UAV2 mission time", safe_stats(df2["uav2_mission_time_s"]), "s")

    uav2_success = pd.to_numeric(df2["uav2_success"], errors="coerce").fillna(0)
    print("\nUAV2 mission success rate: %.2f%%" % (100.0 * uav2_success.mean()))

    merged_file = os.path.join(args.results_dir, "merged_results.xlsx")
    with pd.ExcelWriter(merged_file) as writer:
        df1.to_excel(writer, sheet_name="uav1_mapping_foi", index=False)
        df2.to_excel(writer, sheet_name="uav2_inspection", index=False)

    print("\nSaved merged Excel file to:", merged_file)

if __name__ == "__main__":
    main()