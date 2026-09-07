#!/usr/bin/env python3
"""
generate_fullface.py — regenerate config/cartesian_poses.yaml as a full-face
raster for the SHRUNKEN cabinet.

Design (measured with chained IK in fullface_test.py):
  * Cabinet front face: world x∈[-0.35,0.35] (0.7 m), z∈[0.62,1.18] (0.56 m),
    at world y = 0.945. Sits on a 0.62 m table so the face lands in the
    reachable window (nozzle z∈[0.6,1.45] full-width reachable).
  * Nozzle rows (world z): 0.78 / 0.93 / 1.08 / 1.23. The tool z-axis tilts
    DOWN ~14° toward the face, so paint lands ~0.148 m BELOW the nozzle z at
    the 0.585 m standoff: rows paint bands at 0.63/0.78/0.93/1.08, whose
    ~0.22 m cone union covers [0.52,1.19] ⊇ the 0.62–1.18 face.
  * Each row sweeps world x∈[-0.40,+0.40] (boustrophedon, 9 pts/row), the
    ±0.40 endpoints staying just off the 0.70 m face so vertical connectors
    overspray harmlessly off the cabinet.
  * 36 waypoints total, chain-IK solved (Newton-DLS, damp 1e-4) so the whole
    raster is one continuous joint-space path for the executor.

Output format matches the original cartesian_poses.yaml exactly.
"""
import os
import numpy as np
import yaml
import feasibility_check as fc

FACE_X_HALF = 0.35          # face spans world x∈[-0.35,+0.35]
SWEEP_HALF  = 0.40          # nozzle sweeps world x∈[-0.40,+0.40]
ROWS_Z      = [0.78, 0.93, 1.08, 1.23]   # nozzle world z per row
STANDOFF    = 0.36          # world_y (face at 0.945 → 0.585 m standoff)
STEP_X      = 0.10
QUAT        = [-0.000199, 0.788926, 0.000129, 0.614488]

WP0 = np.array([2.52402, -1.85965, -1.87007, -2.85389, -2.16736, -1.74316])
HOME = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

OUT = "/ws/src/gz_spray_painting_plugin_demo/config/cartesian_poses.yaml"


def main():
    xs = np.round(np.arange(-SWEEP_HALF, SWEEP_HALF + 1e-9, STEP_X), 4)
    # boustrophedon world waypoints
    wp_world = []                     # (wx, wz)
    for i, wz in enumerate(ROWS_Z):
        row = xs if i % 2 == 0 else xs[::-1]
        wp_world.extend((float(wx), float(wz)) for wx in row)

    # chain IK
    qs, seed, worst_step = [], WP0, 0.0
    for k, (wx, wz) in enumerate(wp_world):
        bx, by, bz = fc.world_to_base(wx, STANDOFF, wz)
        cand, ok = fc.ik_dls([bx, by, bz], fc.ROT, seed, n_iter=250, tol=1e-6)
        if not (ok and fc.in_limits(cand)):
            for s in (WP0, np.zeros(6), HOME):
                cand, ok = fc.ik_dls([bx, by, bz], fc.ROT, s, n_iter=250, tol=1e-6)
                if ok and fc.in_limits(cand):
                    break
        if not (ok and fc.in_limits(cand)):
            print(f"FAIL wp{k}: world ({wx:.2f},{wz:.2f}) base "
                  f"({bx:.3f},{by:.3f},{bz:.3f})")
            return
        if k:
            worst_step = max(worst_step, np.linalg.norm(cand - seed))
        seed = cand
        qs.append(cand)
        err = np.linalg.norm(fc.fk(cand)[:3, 3] - np.array([bx, by, bz]))
        print(f"wp{k:>2} world x={wx:+5.2f} z={wz:.2f} "
              f"pos_err={err*1000:4.1f}mm  q={cand[0]:6.3f},{cand[1]:6.3f}")

    print(f"\nall {len(qs)} waypoints solved; max joint step between "
          f"consecutive = {worst_step*180/np.pi:.1f} deg")

    # poses + joint_configs, same format as the original file
    poses, jcs = [], []
    for k, ((wx, wz), q) in enumerate(zip(wp_world, qs)):
        bx, by, bz = fc.world_to_base(wx, STANDOFF, wz)
        name = f"wp{k}"
        poses.append({
            "name": name,
            "position": {x_: float(np.round(v, 6)) for x_, v in
                         zip(("x", "y", "z"), (bx, by, bz))},
            "orientation": {x_: float(v) for x_, v in
                            zip(("x", "y", "z", "w"), QUAT)},
        })
        jcs.append({"name": name, **{
            jn: float(np.round(float(q[i]), 5)) for i, jn in
            enumerate(fc.JOINT_NAMES)}})

    header = (
        "# Full-face raster for the shrunken cabinet (see USE_GUIDE).\n"
        "# Nozzle world z per row: %s (paint lands ~0.148 m lower, tool tilts\n"
        "# down ~14 deg at 0.585 m standoff).\n"
        "# Rows sweep world x in [-0.40,+0.40] (face is [-0.35,+0.35]),\n"
        "# fixed TCP pose at world y=0.36, orientation\n"
        "#   (-0.0002, 0.7889, 0.0001, 0.6145) xyzw.\n"
        "# 36 waypoints, chain-IK continuous. Boustrophedon (spray ON the\n"
        "# whole raster; connectors at x=+-0.40 overspray off the face).\n"
        % (ROWS_Z,)
    )
    data = (header
            + "poses:\n" + yaml.safe_dump(poses, sort_keys=False, indent=2)
            + "joint_configs:\n"
            + yaml.safe_dump(jcs, sort_keys=False, indent=2))

    with open(OUT, "w") as f:
        f.write(data)
    print(f"\nwrote {OUT} ({len(poses)} waypoints)")


if __name__ == "__main__":
    main()
