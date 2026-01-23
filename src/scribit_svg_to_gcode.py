#!/usr/bin/env python3
"""
svg2scribit_gcode.py

Convert stroked SVG paths into Scribit-compatible G-code:
- drawing.gcode: draws the SVG paths (pen down on strokes)
- bbox_dots.gcode: marks the mapped drawing bounding box corners as dots

Notes / assumptions:
- SVG coordinate system is y-down; Scribit "wall XY" here is also treated as y-down.
- Wall origin is top-left; X to the right; Y downward.
- Scribit kinematics are expressed as delta lengths of left/right cords (L/R).
- G-code output uses:
  - G21  : mm units
  - G91  : incremental moves
  - M17  : enable motors
  - G101 : pen down (we use it 3x for reliable latch)
  - G4 S : dwell in seconds (for dots)

Requires:
  pip install svgpathtools
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from svgpathtools import Path, svg2paths2


# Scribit default distance between nails
D_MM_DEFAULT = 1860

# Pen slot Z degrees 
PEN_SLOTS_Z: Dict[int, int] = {1: 89, 2: 161, 3: 233, 4: 305}

# Proven reliable Z reference after homing (G77)
Z_AFTER_G77 = -56.0


@dataclass
class CarouselState:
    """Tracks the commanded carousel Z (degrees) so we can force CCW-only pen changes."""
    z: Optional[float] = None


def ccw_only_target(current_z: Optional[float], slot_z: float) -> float:
    """Return an absolute Z target that moves CCW-only by adding +360 as needed."""
    if current_z is None:
        # Best effort (should be avoided by homing once at the top of the file)
        return float(slot_z)
    target = float(slot_z)
    while target < current_z:
        target += 360.0
    return target


# ----------------------------
# Small utilities
# ----------------------------

def clamp_int(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def clamp_float(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def parse_color(s: Optional[str]) -> Optional[str]:
    """Normalize common SVG color formats. Returns None for 'none'/'transparent'/missing."""
    if not s:
        return None
    s = s.strip().lower()
    if s in ("none", "transparent"):
        return None
    if s.startswith("#"):
        if len(s) == 4:  # #rgb -> #rrggbb
            r, g, b = s[1], s[2], s[3]
            return f"#{r}{r}{g}{g}{b}{b}"
        if len(s) == 7:
            return s
        return s  # unknown length, keep as-is
    m = re.match(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", s)
    if m:
        r, g, b = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        r = clamp_int(r, 0, 255)
        g = clamp_int(g, 0, 255)
        b = clamp_int(b, 0, 255)
        return f"#{r:02x}{g:02x}{b:02x}"
    return s


# ----------------------------
# Scribit geometry & G-code
# ----------------------------

def xy_to_lr(x_mm: float, y_mm: float, D_mm: float) -> Tuple[float, float]:
    """Convert wall XY (mm) to left/right cord lengths (mm)."""
    L = math.hypot(x_mm, y_mm)
    R = math.hypot(D_mm - x_mm, y_mm)
    return L, R

def gcode_header() -> List[str]:
    return ["G21", "G91", "M17"]

def gcode_home_carousel(st: CarouselState) -> List[str]:
    """Home carousel (G77) and set a known Z reference (G92 Z-56)."""
    st.z = Z_AFTER_G77
    return [
        "G21",
        "G90",
        "M17",
        "G77",
        f"G92 Z{Z_AFTER_G77:g}",
        "G91",
    ]

def gcode_pen_select_ccw(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Select a pen slot, forcing CCW-only carousel motion via +360 wrap."""
    if pen not in PEN_SLOTS_Z:
        raise ValueError(f"pen must be 1..4, got {pen}")
    slot = float(PEN_SLOTS_Z[pen])
    target = ccw_only_target(st.z, slot)
    st.z = target
    # Temporarily absolute for Z, then back to incremental (same style as original).
    return ["G90", f"G1 Z{target:.3f} F{f_z}", "G91"]

def gcode_pen_down() -> List[str]:
    # Reliable latch: 30 degrees isn't enough; do G101 three times (~90 degrees).
    return ["G101", "G101", "G101"]

def gcode_pen_up(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Pen-up is implemented by returning to the slot Z (CCW-only)."""
    return gcode_pen_select_ccw(pen, f_z, st)

def gcode_dwell(seconds: float) -> List[str]:
    s = max(0.0, seconds)
    return [f"G4 S{s:.3f}"]

def wall_xy_to_lr_delta_g1(
    cur_xy: Tuple[float, float],
    next_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
) -> Tuple[str, Tuple[float, float]]:
    """
    Emit a single incremental G1 move in (dL, -dR) space, based on wall XY change.
    Returns (gcode_line, updated_xy).
    """
    x0, y0 = cur_xy
    x1, y1 = next_xy
    L0, R0 = xy_to_lr(x0, y0, D_mm)
    L1, R1 = xy_to_lr(x1, y1, D_mm)
    dL = L1 - L0
    dR = R1 - R0
    return (f"G1 X{dL:.3f} Y{-dR:.3f} F{feed}", (x1, y1))

def move_xy_segmented(
    cur_xy: Tuple[float, float],
    target_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
    max_step_mm: float,
) -> Tuple[List[str], Tuple[float, float]]:
    """
    Move from cur_xy to target_xy by splitting the straight-line wall XY path into
    <= max_step_mm segments (helps avoid huge single deltas).
    """
    x0, y0 = cur_xy
    x1, y1 = target_xy
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return ([], cur_xy)
    if max_step_mm <= 0:
        # fallback: single move
        line, new_xy = wall_xy_to_lr_delta_g1(cur_xy, target_xy, D_mm, feed)
        return ([line], new_xy)

    n = max(1, int(math.ceil(dist / max_step_mm)))
    lines: List[str] = []
    xy = cur_xy
    for i in range(1, n + 1):
        t = i / n
        mid = (x0 + dx * t, y0 + dy * t)
        line, xy = wall_xy_to_lr_delta_g1(xy, mid, D_mm, feed)
        lines.append(line)
    return (lines, xy)


# ----------------------------
# SVG sampling & mapping
# ----------------------------

def sample_path_uniform_t(path: Path, n: int) -> List[complex]:
    """Uniform-in-t sampling: returns n+1 points including endpoint."""
    n = max(1, n)
    return [path.point(i / n) for i in range(n + 1)]

@dataclass(frozen=True)
class SvgToWallMapper:
    u_center: float
    v_center: float
    scale: float
    wall_cx: float
    wall_cy: float

    def map_uv(self, u: float, v: float) -> Tuple[float, float]:
        # Preserve y-down
        x = (u - self.u_center) * self.scale + self.wall_cx
        y = (v - self.v_center) * self.scale + self.wall_cy
        return (x, y)


# ----------------------------
# Main conversion pipeline
# ----------------------------

@dataclass(frozen=True)
class Args:
    svg: str
    D_mm: float
    fit_frac: float
    step_mm: float
    travel_step_mm: float
    f_travel: int
    f_draw: int
    f_z: int
    dot_dwell_s: float
    bbox_pen: int
    default_pen: int
    home_carousel: bool
    out_bbox: str
    out_draw: str

def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=(
            "Convert an SVG into Scribit G-code.\n"
            "Outputs:\n"
            "  - bbox_dots.gcode : dots at the mapped bounding box corners\n"
            "  - drawing.gcode   : the actual drawing (stroked paths)\n"
        ),
    )
    p.add_argument("svg", help="Input SVG file path")

    p.add_argument("--D_mm", type=float, default=D_MM_DEFAULT,
                   help="Distance between nails in mm (Scribit D)")

    p.add_argument("--fit_frac", type=float, default=0.70,
                   help="Scale drawing to fit within fit_frac * D in both width/height")

    p.add_argument("--step_mm", type=float, default=1.0,
                   help="Pen-down step size along curves in wall mm (smaller = smoother, larger = faster)")

    p.add_argument("--travel_step_mm", type=float, default=5.0,
                   help="Pen-up max step size in wall mm when repositioning between paths")

    p.add_argument("--f_travel", type=int, default=600, help="Feed rate for pen-up travel moves")
    p.add_argument("--f_draw", type=int, default=300, help="Feed rate for pen-down drawing moves")
    p.add_argument("--f_z", type=int, default=600, help="Feed rate for Z moves (pen select / pen up)")

    p.add_argument("--dot_dwell_s", type=float, default=0.20,
                   help="Dwell time (seconds) when making bbox corner dots")

    p.add_argument("--bbox_pen", type=int, default=1,
                   help="Pen slot (1..4) used for bbox dots")
    p.add_argument("--default_pen", type=int, default=1,
                   help="Fallback pen slot (1..4) if pen mapping overflows")

    p.add_argument("--no_home_carousel", action="store_true",
                   help="Do NOT emit G77 + G92 Z-56 at file start (not recommended)")

    p.add_argument("--out_bbox", default="bbox_dots.gcode",
                   help="Output filename for bbox dots G-code")
    p.add_argument("--out_draw", default="drawing.gcode",
                   help="Output filename for drawing G-code")

    return p

def load_drawable_paths(svg_path: str, default_pen: int) -> Tuple[List[Tuple[Path, int]], Dict[str, int]]:
    """
    Returns:
      drawable: list of (Path, pen_id)
      color_to_pen: mapping used
    Rules:
      - Use stroke color if present and not 'none'
      - If stroke missing AND fill missing/none => assume black stroke
      - If fill is set (visible) but stroke missing => skip (likely a filled shape)
      - Map first seen colors to pens 1..4; overflow uses default_pen
    """
    paths, attrs, _svg_attrs = svg2paths2(svg_path)

    color_to_pen: Dict[str, int] = {}
    next_pen = 1
    drawable: List[Tuple[Path, int]] = []

    for p, a in zip(paths, attrs):
        stroke = parse_color(a.get("stroke"))

        if stroke is None:
            fill = parse_color(a.get("fill"))
            if fill is None:
                stroke = "#000000"
            else:
                continue

        if stroke not in color_to_pen and next_pen <= 4:
            color_to_pen[stroke] = next_pen
            next_pen += 1

        pen = color_to_pen.get(stroke, default_pen)
        drawable.append((p, pen))

    return drawable, color_to_pen

def compute_svg_bbox(drawable: Iterable[Tuple[Path, int]]) -> Tuple[float, float, float, float]:
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")
    any_path = False
    for p, _ in drawable:
        any_path = True
        bx0, bx1, by0, by1 = p.bbox()
        xmin = min(xmin, bx0)
        xmax = max(xmax, bx1)
        ymin = min(ymin, by0)
        ymax = max(ymax, by1)
    if not any_path:
        raise SystemExit("No drawable paths.")
    return xmin, xmax, ymin, ymax

def write_lines(path: str, lines: List[str]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def main() -> None:
    ap = build_argparser()
    ns = ap.parse_args()
    ns.home_carousel = (not ns.no_home_carousel)

    d = vars(ns)
    d.pop("no_home_carousel", None)
    args = Args(**d)

    # Basic validation
    args_D = float(args.D_mm)
    if args_D <= 0:
        raise SystemExit("--D_mm must be > 0")
    if args.bbox_pen not in PEN_SLOTS_Z:
        raise SystemExit("--bbox_pen must be 1..4")
    if args.default_pen not in PEN_SLOTS_Z:
        raise SystemExit("--default_pen must be 1..4")
    if not (0.0 < args.fit_frac <= 1.5):
        # allow >1 a bit for experimentation, but block nonsense
        raise SystemExit("--fit_frac must be in (0, 1.5]")

    # Wall "center" target
    wall_cx = args_D / 2.0
    wall_cy = args_D / 2.0

    drawable, color_to_pen = load_drawable_paths(args.svg, args.default_pen)
    if not drawable:
        raise SystemExit("No drawable stroked paths found (stroke != none).")

    xmin, xmax, ymin, ymax = compute_svg_bbox(drawable)
    svg_w = xmax - xmin
    svg_h = ymax - ymin
    if svg_w <= 0 or svg_h <= 0:
        raise SystemExit("Degenerate SVG bbox (zero width/height).")

    u_center = (xmin + xmax) / 2.0
    v_center = (ymin + ymax) / 2.0
    target = args.fit_frac * args_D
    scale = min(target / svg_w, target / svg_h)

    mapper = SvgToWallMapper(
        u_center=u_center,
        v_center=v_center,
        scale=scale,
        wall_cx=wall_cx,
        wall_cy=wall_cy,
    )

    # ---------- (1) bbox dots ----------
    bbox_corners_wall = [
        mapper.map_uv(xmin, ymin),
        mapper.map_uv(xmax, ymin),
        mapper.map_uv(xmax, ymax),
        mapper.map_uv(xmin, ymax),
    ]

    g_bbox: List[str] = []
    g_bbox += gcode_header()
    st_bbox = CarouselState()
    if args.home_carousel:
        g_bbox += gcode_home_carousel(st_bbox)

    cur_xy = (wall_cx, wall_cy)
    pen = args.bbox_pen

    g_bbox += gcode_pen_select_ccw(pen, args.f_z, st_bbox)
    for xy in bbox_corners_wall:
        lines, cur_xy = move_xy_segmented(
            cur_xy, xy, args_D, args.f_travel, max_step_mm=args.travel_step_mm
        )
        g_bbox += lines
        g_bbox += gcode_pen_down()
        g_bbox += gcode_dwell(args.dot_dwell_s)
        g_bbox += gcode_pen_up(pen, args.f_z, st_bbox)

    write_lines(args.out_bbox, g_bbox)

    # ---------- (2) drawing ----------
    g_draw: List[str] = []
    g_draw += gcode_header()
    st_draw = CarouselState()
    if args.home_carousel:
        g_draw += gcode_home_carousel(st_draw)
    cur_xy = (wall_cx, wall_cy)

    for path, pen in drawable:
        # Estimate number of samples so wall spacing is ~ step_mm
        try:
            length_svg = path.length(error=1e-3)
        except TypeError:
            length_svg = path.length()

        length_wall = length_svg * scale
        n = max(1, int(math.ceil(length_wall / max(1e-9, args.step_mm))))

        pts = sample_path_uniform_t(path, n)
        poly_wall = [mapper.map_uv(pt.real, pt.imag) for pt in pts]

        g_draw += gcode_pen_select_ccw(pen, args.f_z, st_draw)

        # Reposition (pen-up), segmented
        lines, cur_xy = move_xy_segmented(
            cur_xy, poly_wall[0], args_D, args.f_travel, max_step_mm=args.travel_step_mm
        )
        g_draw += lines

        # Draw polyline (pen-down), already dense in wall space
        g_draw += gcode_pen_down()

        # Emit drawing as per-point deltas in LR space
        # (we keep this as a sequence of single-step moves; step_mm controls density)
        for xy in poly_wall[1:]:
            line, cur_xy = wall_xy_to_lr_delta_g1(cur_xy, xy, args_D, args.f_draw)
            g_draw.append(line)

        g_draw += gcode_pen_up(pen, args.f_z, st_draw)

    write_lines(args.out_draw, g_draw)

    print(f"Wrote: {args.out_bbox}")
    print(f"Wrote: {args.out_draw}")
    print(f"D_mm={args_D:.1f} scale={scale:.6f} fit_frac={args.fit_frac} step_mm={args.step_mm} travel_step_mm={args.travel_step_mm}")
    print(f"Color->pen map: {color_to_pen}")
    print(f"home_carousel={args.home_carousel} (disable with --no_home_carousel)")


if __name__ == "__main__":
    main()

