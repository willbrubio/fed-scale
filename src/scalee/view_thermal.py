#!/usr/bin/env python3
"""
Visualize AMG8833 thermal frames logged by the mouse-scale rig.

Reads a THRM####.CSV file (columns: iso,unixtime,millis,px0..px63) and plays the
8x8 frames back as an animated thermal image. Quickest way to confirm the IR
sensor is actually capturing heat/motion - wave a hand and watch the blob track.

Usage (run from the repo root):
    python src/scalee/view_thermal.py TESTDATA/THRM0007.CSV
    python src/scalee/view_thermal.py TESTDATA/THRM0007.CSV --interval 120 --cmap jet
    python src/scalee/view_thermal.py TESTDATA/THRM0007.CSV --rot 1 --flip h      # fix orientation
    python src/scalee/view_thermal.py TESTDATA/THRM0007.CSV --save thermal.gif    # write instead of show

    Or as a module (with src/ on PYTHONPATH):
    python -m scalee.view_thermal TESTDATA/THRM0007.CSV

Needs: numpy, pandas, matplotlib  (pillow for --save *.gif, ffmpeg for *.mp4)
"""
import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def main():
    p = argparse.ArgumentParser(description="Play back AMG8833 8x8 thermal frames.")
    p.add_argument("csv", help="Path to a THRM####.CSV file")
    p.add_argument("--interval", type=int, default=200, help="ms between frames (playback speed)")
    p.add_argument("--cmap", default="inferno", help="matplotlib colormap (try inferno, jet, magma)")
    # Orientation is mounting-dependent; adjust these if the image looks rotated/mirrored.
    p.add_argument("--rot", type=int, default=0, choices=[0, 1, 2, 3], help="rotate 90*rot deg")
    p.add_argument("--flip", default="none", choices=["none", "h", "v"], help="mirror h or v")
    p.add_argument("--step", type=int, default=1, help="show every Nth frame (speed up long runs)")
    p.add_argument("--save", default=None, help="save animation to .gif/.mp4 instead of showing")
    args = p.parse_args()

    # --- Load ---
    try:
        df = pd.read_csv(args.csv)
    except FileNotFoundError:
        sys.exit(f"File not found: {args.csv}")

    # Pull the 64 pixel columns in order; fail clearly if this isn't a THRM file.
    px_cols = [f"px{i}" for i in range(64)]
    missing = [c for c in px_cols if c not in df.columns]
    if missing:
        sys.exit(f"Missing pixel columns (is this a THRM file?): {missing[:3]} ...")

    # Each row -> one 8x8 frame. px index i maps to row=i//8, col=i%8 (matches firmware).
    frames = df[px_cols].to_numpy(dtype=float).reshape(-1, 8, 8)

    # Optional orientation fixes.
    if args.rot:
        frames = np.rot90(frames, k=args.rot, axes=(1, 2))
    if args.flip == "h":
        frames = frames[:, :, ::-1]
    elif args.flip == "v":
        frames = frames[:, ::-1, :]

    # Subsample for long recordings.
    frames = frames[:: args.step]
    n = len(frames)
    if n == 0:
        sys.exit("No frames to show.")

    # Timestamps for the title (fall back to an index if the column is absent).
    labels = df["iso"].astype(str).tolist()[:: args.step] if "iso" in df.columns \
        else [str(i) for i in range(n)]

    # Fixed color scale across the whole run so a warm blob stays visually comparable.
    vmin, vmax = float(np.nanmin(frames)), float(np.nanmax(frames))

    # --- Console sanity stats (useful even without watching) ---
    per_frame_max = frames.reshape(n, -1).max(axis=1)
    print(f"frames:        {n}")
    print(f"temp range:    {vmin:.1f} - {vmax:.1f} C")
    print(f"per-frame max: mean {per_frame_max.mean():.1f} C, peak {per_frame_max.max():.1f} C")
    # A near-flat scene (small spread) usually means nothing warm passed the sensor.
    if vmax - vmin < 1.0:
        print("note: very little temperature variation - is anything warm in view?")

    # --- Animated view ---
    fig, ax = plt.subplots(figsize=(5, 5.5))
    # bicubic interpolation smooths the coarse 8x8 into a readable thermal image.
    im = ax.imshow(frames[0], cmap=args.cmap, vmin=vmin, vmax=vmax, interpolation="bicubic")
    fig.colorbar(im, ax=ax, label="deg C", fraction=0.046, pad=0.04)
    ax.set_xticks([]); ax.set_yticks([])
    title = ax.set_title("")

    def update(i):
        im.set_data(frames[i])
        title.set_text(f"frame {i + 1}/{n}   {labels[i]}")
        return im, title

    anim = FuncAnimation(fig, update, frames=n, interval=args.interval, blit=False)

    if args.save:
        anim.save(args.save, dpi=100)   # .gif needs pillow; .mp4 needs ffmpeg
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()