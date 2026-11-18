"""Writes tests/data/diw_linemerge.json: 50 random wall-fragment sets and shapely.linemerge's result for each
(diw/PLANNER_DESIGN.md T9: the C++ LineMerger port of DIWPowderPlanner is checked against these fixtures).

usage:  python make_linemerge_fixtures.py <out.json>

Each set holds 1-3 closed loops (regular polygons with jitter, coordinates on a 0.001 grid) cut into fragments at
random vertices, some fragments reversed, some with a repeated point, plus dangling spurs that share an end point
with a loop (degree-3 nodes), an isolated closed ring and an occasional single-point fragment.
"""
import json
import math
import random
import sys

import shapely
from shapely.geometry import LineString, MultiLineString
from shapely.ops import linemerge


def loop(rng, cx, cy, r, n):
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n + rng.uniform(-0.1, 0.1)
        rr = r * rng.uniform(0.8, 1.2)
        pts.append((round(cx + rr * math.cos(a), 3), round(cy + rr * math.sin(a), 3)))
    pts.append(pts[0])
    return pts


def fragments(rng, ring):
    n = len(ring) - 1
    cuts = sorted(rng.sample(range(1, n), rng.randint(1, min(4, n - 1))))
    idx = [0] + cuts + [n]
    out = []
    for a, b in zip(idx, idx[1:]):
        f = ring[a:b + 1]
        if rng.random() < 0.2 and len(f) > 2:
            k = rng.randrange(1, len(f) - 1)
            f = f[:k] + [f[k]] + f[k:]          # repeated point
        if rng.random() < 0.4:
            f = f[::-1]
        out.append(f)
    return out


def make_set(rng):
    lines = []
    for _ in range(rng.randint(1, 3)):
        ring = loop(rng, rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(3, 20), rng.randint(5, 24))
        frs = fragments(rng, ring)
        if rng.random() < 0.3:
            p = rng.choice(ring[:-1])
            frs.append([p, (round(p[0] + rng.uniform(1, 3), 3), round(p[1] + rng.uniform(1, 3), 3))])   # spur
        lines += frs
    if rng.random() < 0.3:
        lines.append(loop(rng, 80.0, 80.0, 2.0, 6))                                                     # isolated ring
    if rng.random() < 0.1:
        lines.append([(1.0, 1.0), (1.0, 1.0)])                                                          # single point
    rng.shuffle(lines)
    return lines


def main(argv):
    rng = random.Random(20261117)
    sets = []
    for _ in range(50):
        lines = make_set(rng)
        merged = linemerge(MultiLineString([LineString(l) for l in lines]))
        geoms = list(merged.geoms) if hasattr(merged, "geoms") else ([merged] if not merged.is_empty else [])
        sets.append({"lines": [[list(p) for p in l] for l in lines],
                     "merged": [[list(c) for c in g.coords] for g in geoms]})
    with open(argv[1], "w", encoding="ascii", newline="\n") as fh:
        json.dump({"shapely": shapely.__version__, "geos": shapely.geos_version_string, "sets": sets}, fh)
    print(f"{len(sets)} sets written to {argv[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
