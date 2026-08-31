#!/usr/bin/env python3
"""
Plot the mouse-scale main log on one shared timeline: weight, CO2, ambient
temp/humidity, and the thermal hotspot - so you can see how they line up
across a run.

Reads a DATA####.CSV (iso,unixtime,millis,raw,tared,grams,co2_ppm,temp_c,
rh_pct,ir_min,ir_max,ir_mean,ir_hot). The IR *summary* columns live in this
file already; use view_thermal.py on the paired THRM file for full 8x8 frames.

Usage (run from the repo root):
    python src/scalee/view_run.py TESTDATA/DATA0007.CSV
    python src/scalee/view_run.py TESTDATA/DATA0007.CSV --step 5 --save run.png
    python src/scalee/view_run.py TESTDATA/DATA0007.CSV --avg-sec 30   # smooth via 30s bin averages

    Or as a module (with src/ on PYTHONPATH):
    python -m scalee.view_run TESTDATA/DATA0007.CSV

Needs: numpy, pandas, matplotlib
"""
import argparse
import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description="Aligned timeline of weight, CO2, temp/humidity, thermal hotspot.")
    p.add_argument("csv", help="Path to a DATA####.CSV file")
    p.add_argument("--step", type=int, default=1, help="plot every Nth row (speed up long runs)")
    p.add_argument("--avg-sec", type=float, default=None,
                    help="average all traces into N-second bins instead of plotting raw samples "
                         "(applied after --step, if given)")
    p.add_argument("--save", default=None, help="save the figure instead of showing it")
    args = p.parse_args()

    try:
        df = pd.read_csv(args.csv)
    except FileNotFoundError:
        sys.exit(f"File not found: {args.csv}")

    if args.step > 1:
        df = df.iloc[:: args.step].reset_index(drop=True)
    if len(df) == 0:
        sys.exit("No rows in file.")

    # High-resolution wall-clock axis: anchor to the first row's unixtime, then advance by
    # the millis delta. millis is monotonic at 10 Hz; unixtime alone is only 1 Hz, so 10
    # rows would otherwise stack on the same instant.
    if {"unixtime", "millis"}.issubset(df.columns):
        t0 = pd.to_datetime(df["unixtime"].iloc[0], unit="s")
        t = t0 + pd.to_timedelta(df["millis"] - df["millis"].iloc[0], unit="ms")
    else:
        t = pd.to_datetime(df["iso"])          # fallback: 1 s resolution

    # Weight: prefer calibrated grams; fall back to raw counts if never calibrated.
    if "grams" in df and df["grams"].notna().any():
        weight, wlabel = df["grams"], "weight (g)"
    else:
        weight, wlabel = df["tared"], "weight (raw counts)"

    # Optionally collapse every trace into fixed-width time bins (mean per bin) - trades
    # sample-level detail for a readable trend on long, noisy runs.
    if args.avg_sec:
        if args.avg_sec <= 0:
            sys.exit("--avg-sec must be positive.")
        cols = {"weight": weight}
        for c in ("co2_ppm", "temp_c", "rh_pct", "ir_max", "ir_hot"):
            if c in df:
                cols[c] = df[c]
        binned = pd.DataFrame(cols)
        binned.index = pd.DatetimeIndex(t)
        binned = binned.resample(pd.Timedelta(seconds=args.avg_sec)).mean()
        if len(binned) == 0:
            sys.exit("No data after averaging.")
        t = pd.Series(binned.index.values)
        weight = binned["weight"]
        df = binned

    # --- Console sanity summary ---
    dur = (t.iloc[-1] - t.iloc[0]).total_seconds()
    print(f"rows: {len(df)}   duration: {dur/60:.1f} min")
    print(f"weight: {weight.min():.2f} .. {weight.max():.2f}  ({wlabel})")
    if df.get("co2_ppm") is not None and df["co2_ppm"].notna().any():
        print(f"CO2:    {df['co2_ppm'].min():.0f} .. {df['co2_ppm'].max():.0f} ppm")
    if df.get("ir_max") is not None and df["ir_max"].notna().any():
        print(f"IR max: {df['ir_max'].min():.1f} .. {df['ir_max'].max():.1f} C")
    if df.get("temp_c") is not None and df["temp_c"].notna().any():
        print(f"Temp:   {df['temp_c'].min():.1f} .. {df['temp_c'].max():.1f} C")
    if df.get("rh_pct") is not None and df["rh_pct"].notna().any():
        print(f"RH:     {df['rh_pct'].min():.1f} .. {df['rh_pct'].max():.1f} %")

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    # --- Weight ---
    ax[0].plot(t, weight, lw=0.8, color="tab:blue")
    ax[0].set_ylabel(wlabel)
    ax[0].grid(alpha=0.3)

    # --- CO2 ---
    if "co2_ppm" in df and df["co2_ppm"].notna().any():
        ax[1].plot(t, df["co2_ppm"], lw=1.0, color="tab:green")
        ax[1].set_ylabel("CO2 (ppm)")
    else:
        ax[1].text(0.5, 0.5, "no CO2 data", ha="center", va="center", transform=ax[1].transAxes)
    ax[1].grid(alpha=0.3)

    # --- Ambient temp (left axis) + humidity (right axis) ---
    if "temp_c" in df and df["temp_c"].notna().any():
        ax[2].plot(t, df["temp_c"], lw=1.0, color="tab:orange")
        ax[2].set_ylabel("temp (C)", color="tab:orange")
        ax[2].tick_params(axis="y", labelcolor="tab:orange")
        if "rh_pct" in df and df["rh_pct"].notna().any():
            axh = ax[2].twinx()
            axh.plot(t, df["rh_pct"], lw=0.6, color="tab:cyan", alpha=0.7)
            axh.set_ylabel("RH (%)", color="tab:cyan")
            axh.tick_params(axis="y", labelcolor="tab:cyan")
    else:
        ax[2].text(0.5, 0.5, "no temp/RH data", ha="center", va="center", transform=ax[2].transAxes)
    ax[2].grid(alpha=0.3)

    # --- Thermal: hotspot temperature (left axis) + hotspot pixel position (right axis) ---
    if "ir_max" in df and df["ir_max"].notna().any():
        ax[3].plot(t, df["ir_max"], lw=1.0, color="tab:red")
        ax[3].set_ylabel("IR max (C)", color="tab:red")
        ax[3].tick_params(axis="y", labelcolor="tab:red")
        # Hotspot position jumps randomly when nothing warm is present and settles on the
        # animal when it is - so this trace itself reads as "present / where".
        axp = ax[3].twinx()
        axp.plot(t, df["ir_hot"], lw=0.6, color="tab:purple", alpha=0.7)
        axp.set_ylabel("hotspot pixel (0-63)", color="tab:purple")
        axp.set_ylim(-1, 64)
        axp.tick_params(axis="y", labelcolor="tab:purple")
    else:
        ax[3].text(0.5, 0.5, "no IR data", ha="center", va="center", transform=ax[3].transAxes)
    ax[3].grid(alpha=0.3)
    ax[3].set_xlabel("time")

    fig.suptitle(f"{args.csv}   ({len(df)} rows, {dur/60:.1f} min)")
    fig.autofmt_xdate()
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()