#!/usr/bin/env python3
"""
fullface_test.py — find the largest FULLY-PAINTABLE rectangle of the cabinet
face by chaining IK along a boustrophedon raster (the same way the executor
will actually move). Per-cell "exists a solution" maps are branch-dependent
and misleading; this measures what a continuous chain can really sweep.

Usage:  python3 fullface_test.py [standoff] [x_max] [z_lo] [z_hi] [step]
"""
import sys
import numpy as np
import feasibility_check as fc

WP0 = np.array([2.52402, -1.85965, -1.87007, -2.85389, -2.16736, -1.74316])
HOME = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

STANDOFF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.36
X_MAX    = float(sys.argv[2]) if len(sys.argv) > 2 else 0.40
Z_LO     = float(sys.argv[3]) if len(sys.argv) > 3 else 0.60
Z_HI     = float(sys.argv[4]) if len(sys.argv) > 4 else 1.45
STEP     = float(sys.argv[5]) if len(sys.argv) > 5 else 0.10


def chain_ik_region(x_lo, x_hi, z_lo, z_hi, step=STEP, standoff=STANDOFF,
                    seed0=WP0, recover=None):
    """Boustrophedon chain over [x_lo,x_hi]x[z_lo,z_hi]. Returns dict
    (wx,wz)->q for reachable cells and the ordered waypoint list."""
    xs = np.round(np.arange(x_lo, x_hi + 1e-9, step), 3)
    zs = np.round(np.arange(z_lo, z_hi + 1e-9, step), 3)
    order = []
    for i, z in enumerate(zs):
        row = list(xs) if i % 2 == 0 else list(reversed(xs))
        order.extend((wx, z) for wx in row)
    if recover is None:
        recover = [WP0, np.zeros(6), HOME]

    waypoints = []          # (wx, wz, q or None)
    reachable = {}
    seed = seed0
    for wx, wz in order:
        bx, by, bz = fc.world_to_base(wx, standoff, wz)
        cand, ok = fc.ik_dls([bx, by, bz], fc.ROT, seed, n_iter=250, tol=1e-6)
        if not (ok and fc.in_limits(cand)):
            for s in recover:
                cand, ok = fc.ik_dls([bx, by, bz], fc.ROT, s, n_iter=250, tol=1e-6)
                if ok and fc.in_limits(cand):
                    break
        if ok and fc.in_limits(cand):
            reachable[(wx, wz)] = cand
            seed = cand
            waypoints.append((wx, wz, cand))
        else:
            waypoints.append((wx, wz, None))
    return reachable, waypoints


def largest_rect(reachable, xs, zs):
    """Biggest fully-reachable axis-aligned rectangle (area)."""
    best = None
    for i, z_lo in enumerate(zs):
        for j in range(i, len(zs)):
            z_hi = zs[j]
            cols = [wx for wx in xs
                    if all((wx, z) in reachable for z in zs[i:j + 1])]
            if len(cols) > 1 and j > i:
                area = (z_hi - z_lo) * (cols[-1] - cols[0])
                if best is None or area > best[0]:
                    best = (area, z_lo, z_hi, cols[0], cols[-1])
    return best


def main():
    xs = np.round(np.arange(-X_MAX, X_MAX + 1e-9, STEP), 3)
    zs = np.round(np.arange(Z_LO, Z_HI + 1e-9, STEP), 3)
    print(f"standoff={STANDOFF} region x∈[{-X_MAX},{X_MAX}] "
          f"z∈[{Z_LO},{Z_HI}] step={STEP}")
    print("x\\z " + "".join(f"{z:>5}" for z in zs))
    reachable, waypoints = chain_ik_region(-X_MAX, X_MAX, Z_LO, Z_HI)
    for wx in xs:
        line = f"{wx:>5} "
        for wz in zs:
            line += "    ·" if (wx, wz) in reachable else "    X"
        print(line, flush=True)

    n_ok = len(reachable)
    n_tot = len(xs) * len(zs)
    print(f"\nchained reachable: {n_ok}/{n_tot} cells "
          f"({100.0 * n_ok / n_tot:.0f}%)")
    ok_wps = [w for w in waypoints if w[2] is not None]
    print(f"waypoints solved: {len(ok_wps)}/{len(waypoints)}")

    best = largest_rect(reachable, xs, zs)
    if best:
        print(f"largest full rectangle: z∈[{best[1]:.2f},{best[2]:.2f}] "
              f"x∈[{best[3]:.2f},{best[4]:.2f}]  "
              f"= {best[4]-best[3]:.2f} m wide × {best[2]-best[1]:.2f} m tall")
    # per-row reachable x-span at each z
    print("\nper-pass (each z row) reachable x range:")
    for wz in zs:
        cols = [wx for wx in xs if (wx, wz) in reachable]
        if cols:
            print(f"  z={wz:.2f}: x∈[{min(cols):.2f},{max(cols):.2f}] "
                  f"({len(cols)} pts)")
        else:
            print(f"  z={wz:.2f}: —")
    return reachable, waypoints


if __name__ == "__main__":
    main()
