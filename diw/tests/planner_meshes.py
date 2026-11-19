"""Generated ASCII STL meshes of the planner corpus (diw/PLANNER_DESIGN.md sections 2.1 and 10.1).

usage:  python planner_meshes.py <out dir> [--delta <mm>]

Writes, in mm, each part standing on z = 0 and centred near the origin:
  cube25.stl    a 25 mm cube
  ring.stl      outer diameter 30, hole diameter 14, 10 mm high
  lshape.stl    a 30 x 30 L with 12 mm arms, 10 mm high
  notch.stl     a 25 mm cube with a 15 degree V-notch cut into one side (tip 12.5 mm deep)
  cone.stl      a frustum from diameter 20 to diameter 4, 15 mm high
  slot.stl      a 20 x 20 x 10 block with a through slot 2 d_eff + 2 um wide (d_eff = delta + 0.001 mm,
                --delta defaults to 0.4), 12 mm long, open on one side
  overhang.stl  a wedge whose one side overhangs at 45 degrees (20 mm long, 10 mm base, 10 mm high)
  two_a.stl, two_b.stl  14 x 10 x 7 and 10 x 12 x 7 boxes, the two parts of the two_objects plate
  box20.stl, tube_thin.stl, spike_a/b/c.stl  the review regressions (regress_review.py): a 20 mm cube, a tube
                OD 12 / ID 9.6 (one bead), and three 30 mm tapered prisms (bases 6, 8, 5 mm), all 20 mm high
  cube10.stl    a 10 mm cube, the short part of the increment-5 G4 tall_short plate (with cube25)
The repo meshes tests/data/cube_with_hole.obj, cube_with_concave_hole.obj, overhang.obj and bridge.obj are used
as they are and are not written here.
"""
import math
import os
import sys

POWDER_GUARD_MM = 0.001


def write(path, tris, name):
    with open(path, "w", newline="\n") as f:
        f.write(f"solid {name}\n")
        for t in tris:
            f.write(" facet normal 0 0 0\n  outer loop\n")
            for p in t:
                f.write("   vertex %.9g %.9g %.9g\n" % p)
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def side(a, b, z0, z1):
    """Quad wall for the directed edge a->b (material on the left): outward normal by right-hand order."""
    A0, B0, A1, B1 = (*a, z0), (*b, z0), (*a, z1), (*b, z1)
    return [(A0, B0, B1), (A0, B1, A1)]


def fan(poly):
    """Triangle fan of a convex (or star-shaped from vertex 0) CCW polygon."""
    return [(poly[0], poly[i], poly[i + 1]) for i in range(1, len(poly) - 1)]


def prism(poly, cap_tris, h):
    """poly: CCW outer loop; cap_tris: CCW 2D triangles covering it."""
    t = []
    for a, b, c in cap_tris:
        t.append(((*a, h), (*b, h), (*c, h)))
        t.append(((*a, 0), (*c, 0), (*b, 0)))
    for i in range(len(poly)):
        t += side(poly[i], poly[(i + 1) % len(poly)], 0, h)
    return t


def box(sx, sy, h):
    p = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    return prism(p, fan(p), h)


def ring(ro, ri, h, n=96):
    O = [(ro * math.cos(2 * math.pi * i / n), ro * math.sin(2 * math.pi * i / n)) for i in range(n)]
    I = [(ri * math.cos(2 * math.pi * i / n), ri * math.sin(2 * math.pi * i / n)) for i in range(n)]
    t = []
    for i in range(n):
        j = (i + 1) % n
        for a, b, c in [(O[i], O[j], I[j]), (O[i], I[j], I[i])]:
            t.append(((*a, h), (*b, h), (*c, h)))
            t.append(((*a, 0), (*c, 0), (*b, 0)))
        t += side(O[i], O[j], 0, h)
        t += side(I[j], I[i], 0, h)                     # hole walls run CW
    return t


def lshape(L, arm, h):
    """An L of outer size L x L and arm width arm, the corner at (-L/2, -L/2)."""
    o = -L / 2
    p = [(o, o), (o + L, o), (o + L, o + arm), (o + arm, o + arm), (o + arm, o + L), (o, o + L)]
    caps = [(p[0], p[1], p[2]), (p[0], p[2], p[3]), (p[0], p[3], p[4]), (p[0], p[4], p[5])]
    return prism(p, caps, h)


def notch(a, angle_deg, depth, h):
    """A cube of side a with a V-notch of the given opening angle cut into the +X side, tip depth deep."""
    s = a / 2
    half = depth * math.tan(math.radians(angle_deg) / 2)
    tip = (s - depth, 0.0)
    p = [(-s, -s), (s, -s), (s, -half), tip, (s, half), (s, s), (-s, s)]
    # star-shaped from (-s, 0): fan around an extra centre vertex
    c = (-s / 2, 0.0)
    caps = [(c, p[i], p[(i + 1) % len(p)]) for i in range(len(p))]
    return prism(p, caps, h)


def frustum(r0, r1, h, n=96):
    B = [(r0 * math.cos(2 * math.pi * i / n), r0 * math.sin(2 * math.pi * i / n), 0) for i in range(n)]
    T = [(r1 * math.cos(2 * math.pi * i / n), r1 * math.sin(2 * math.pi * i / n), h) for i in range(n)]
    t = []
    for i in range(n):
        j = (i + 1) % n
        t.append((B[i], B[j], T[j]))
        t.append((B[i], T[j], T[i]))
        t.append(((0, 0, h), T[i], T[j]))
        t.append(((0, 0, 0), B[j], B[i]))
    return t


def slot(a, width, length, h):
    """An a x a block with a slot of the given width cut in from the +X side, length deep, centred on y = 0."""
    s, w = a / 2, width / 2
    x1 = s - length
    p = [(-s, -s), (s, -s), (s, -w), (x1, -w), (x1, w), (s, w), (s, s), (-s, s)]
    caps = [(p[0], p[1], p[2]), (p[0], p[2], p[3]), (p[0], p[3], p[4]), (p[0], p[4], p[7]),
            (p[4], p[5], p[6]), (p[4], p[6], p[7])]
    return prism(p, caps, h)


def wedge(length, base, h):
    """A prism along X whose YZ section is a right trapezium: the -Y side vertical, the +Y side overhanging at
    45 degrees (base at z = 0, base + h at the top)."""
    y0, y1, y2 = -base / 2, base / 2, base / 2 + h
    sec = [(y0, 0.0), (y1, 0.0), (y2, h), (y0, h)]              # CCW in (y, z)
    # extrude the section along w = 0..length, then map (u, v, w) -> (w - length/2, u, v): a cyclic axis
    # permutation, so the orientation is kept
    return [tuple((w - length / 2, u, v) for u, v, w in tri) for tri in prism(sec, fan(sec), length)]


def main(argv):
    args = argv[1:]
    if not args:
        print(__doc__)
        return 2
    delta = 0.4
    if "--delta" in args:
        i = args.index("--delta")
        delta = float(args[i + 1])
        del args[i:i + 2]
    out = args[0]
    os.makedirs(out, exist_ok=True)
    d_eff = delta + POWDER_GUARD_MM
    parts = {
        "cube25": box(25.0, 25.0, 25.0),
        "ring": ring(15.0, 7.0, 10.0),
        "lshape": lshape(30.0, 12.0, 10.0),
        "notch": notch(25.0, 15.0, 12.5, 25.0),
        "cone": frustum(10.0, 2.0, 15.0),
        "slot": slot(20.0, 2 * d_eff + 0.002, 12.0, 10.0),
        "overhang": wedge(20.0, 10.0, 10.0),
        # the two parts of the two_objects plate (the corpus' box_a / box_b)
        "two_a": box(14.0, 10.0, 7.0),
        "two_b": box(10.0, 12.0, 7.0),
        # review regressions (regress_review.py): a 20 mm cube (sparse infill, so the layers are core), a thin tube
        # printed as one bead (OD 12, wall 1.2 mm < the 1.4 mm line), and slender tapered prisms whose middle is
        # variable-width gap fill
        "box20": box(20.0, 20.0, 20.0),
        "tube_thin": ring(6.0, 4.8, 20.0, n=256),
        "spike_a": prism([(-3.0, -15.0), (3.0, -15.0), (0.0, 15.0)], fan([(-3.0, -15.0), (3.0, -15.0), (0.0, 15.0)]), 20.0),
        "spike_b": prism([(-4.0, -15.0), (4.0, -15.0), (0.0, 15.0)], fan([(-4.0, -15.0), (4.0, -15.0), (0.0, 15.0)]), 20.0),
        "spike_c": prism([(-2.5, -15.0), (2.5, -15.0), (0.0, 15.0)], fan([(-2.5, -15.0), (2.5, -15.0), (0.0, 15.0)]), 20.0),
        # increment 5 G4 (PRODUCT_DESIGN.md 8): the short part of the tall_short plate (cube25 + this 10 mm cube)
        "cube10": box(10.0, 10.0, 10.0),
    }
    for name, tris in parts.items():
        write(os.path.join(out, name + ".stl"), tris, name)
    print(f"{len(parts)} meshes written to {out} (slot width {2 * d_eff + 0.002:.6f} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
