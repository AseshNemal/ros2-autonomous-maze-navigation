#!/usr/bin/env python3
"""Check maze_layout.yaml against Figure 1 of the assignment, by measuring the figure's line widths.

docs/figure1_from_assignment.png is Figure 1 extracted unchanged from docs/Assignment_02.pdf
(`pdfimages -f 3 -l 3 -png docs/Assignment_02.pdf fig`, the 480x493 image).  Every interior cell
boundary is sampled at three points; a boundary is a WALL when the dark run through it is >= 3 px
(thick line), otherwise open (thin 1-2 px line).  The measured wall set is compared with
`vertical_walls` / `horizontal_walls` in the layout.  Needs Pillow; exits 0 only on an exact match.
"""
import sys
from pathlib import Path

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "docs" / "figure1_from_assignment.png"
LAYOUT = ROOT / "ros2_ws/src/maze_simulation/config/maze_layout.yaml"
N = 6                      # Figure 1 is a 6 x 6 grid of 0.5 m cells
THICK_PX = 3


def measure(path=FIG):
    im = Image.open(path).convert("L")
    w, h = im.size
    dark = lambda x, y: im.getpixel((x, y)) < 128           # noqa: E731
    xs = [3.5 + (w - 7) / N * k for k in range(N + 1)]       # grid-line centres (outer frame ~4 px)
    ys = [2.5 + (h - 5) / N * k for k in range(N + 1)]
    vert, hor = set(), set()
    for k in range(1, N):
        for r in range(N):
            runs = []
            for t in (0.35, 0.5, 0.65):
                y, x = int(ys[r] + (ys[r + 1] - ys[r]) * t), int(round(xs[k]))
                runs.append(sum(dark(x + d, y) for d in range(-5, 6)))
            if min(runs) >= THICK_PX:
                vert.add((k, r))
        for c in range(N):
            runs = []
            for t in (0.35, 0.5, 0.65):
                x, y = int(xs[c] + (xs[c + 1] - xs[c]) * t), int(round(ys[k]))
                runs.append(sum(dark(x, y + d) for d in range(-5, 6)))
            if min(runs) >= THICK_PX:
                hor.add((k, c))
    return vert, hor


def main():
    layout = yaml.safe_load(LAYOUT.read_text())
    vert, hor = measure()
    lv = {tuple(v) for v in layout["vertical_walls"]}
    lh = {tuple(v) for v in layout["horizontal_walls"]}
    ok = (vert == lv and hor == lh and layout["rows"] == N and layout["cols"] == N
          and layout["cell_size_m"] == 0.5)
    print(f"figure walls: {len(vert)} vertical + {len(hor)} horizontal interior; "
          f"layout: {len(lv)} + {len(lh)}")
    print("only in figure:", sorted(vert - lv), sorted(hor - lh))
    print("only in layout:", sorted(lv - vert), sorted(lh - hor))
    print("FIGURE1 MATCH" if ok else "FIGURE1 MISMATCH")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
