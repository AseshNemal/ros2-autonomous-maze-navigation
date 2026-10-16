#!/usr/bin/env python3
"""Compose the report figures from real, committed files only (Pillow):
  docs/figures/fig_maze.png  Figure 1 (assignment PDF) | our ground-truth occupancy map
  docs/figures/fig_slam.png  ground-truth map | SLAM Toolbox map of the last recorded VM run
"""
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"
MAPS = ROOT / "ros2_ws/src/slam_mapping/maps"


def panel(img, size, caption, smooth=False):
    img = img.convert("L").resize(size, Image.LANCZOS if smooth else Image.NEAREST).convert("RGB")
    out = Image.new("RGB", (size[0], size[1] + 28), "white")
    out.paste(img, (0, 28))
    ImageDraw.Draw(out).text((6, 8), caption, fill="black")
    return out


def row(panels, path):
    w = sum(p.width for p in panels) + 12 * (len(panels) - 1)
    canvas = Image.new("RGB", (w, max(p.height for p in panels)), "white")
    x = 0
    for p in panels:
        canvas.paste(p, (x, 0))
        x += p.width + 12
    canvas.save(path)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    gt = Image.open(MAPS / "maze_groundtruth.pgm")
    slam = Image.open(MAPS / "maze_slam.pgm")
    fig1 = Image.open(ROOT / "docs/figure1_from_assignment.png")
    side = 300
    row([panel(fig1, (side, round(side * fig1.height / fig1.width)), "Figure 1 (assignment PDF)", smooth=True),
         panel(gt, (side, side), "Our ground-truth map (from maze_layout.yaml)")], OUT / "fig_maze.png")
    row([panel(gt, (side, side), "Ground truth"),
         panel(slam, (side, round(side * slam.height / slam.width)),
               "SLAM Toolbox map, last VM run (FAIL)")], OUT / "fig_slam.png")
    print("wrote", OUT / "fig_maze.png", OUT / "fig_slam.png")


if __name__ == "__main__":
    main()
