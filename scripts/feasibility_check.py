#!/usr/bin/env python3
"""
Self-contained UR5e FK/IK + feasibility map over the cabinet face.

The original generator (generate_spray_poses.py) imports scipy, which is NOT
installed in the container python, so it can never run here. This script uses
pure numpy (damped least-squares IK on the DH FK) instead.

First mode  (default): verify the IK reproduces the working joint configs in
config/cartesian_poses.yaml — a correctness check of the whole kinematics.

Second mode (--map [standoff]): paint a reachability map of the cabinet face
at a fixed standoff (world_y), with the fixed known-good orientation.

Frame map (robot injected at world (0,0,0.8), yaw 90°):
    world_x = -base_y,  world_y = base_x,  world_z = base_z + 0.8
so for a nozzle at world_y=0.36 (face at y=0.945, standoff 0.585 m):
    base_x = world_y,  base_y = -world_x,  base_z = world_z - 0.8
"""
import sys
import numpy as np
import yaml

# ── UR5e DH kinematics (same tables as the generator) ─────────────────────────
D  = [0.1625,  0.0,    0.0,     0.1333, 0.0997,  0.0996]
A  = [0.0,    -0.425, -0.3922,  0.0,    0.0,     0.0   ]
AL = [np.pi/2, 0.0,   0.0,     np.pi/2, -np.pi/2, 0.0  ]

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def _dh(d, a, alpha, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q):
    T = np.eye(4)
    for i in range(6):
        T = T @ _dh(D[i], A[i], AL[i], q[i])
    return T


def fk_with_frames(q):
    """Return (tool0 T, list of T0..T6) for building the geometric Jacobian."""
    frames = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        T = T @ _dh(D[i], A[i], AL[i], q[i])
        frames.append(T)
    return frames[-1], frames


def geometric_jacobian(frames, p_tool):
    """Analytic geometric Jacobian: revolute joints, z-axis of each link frame."""
    J = np.zeros((6, 6))
    for i in range(6):
        T_prev = frames[i]
        z = T_prev[:3, :3] @ np.array([0.0, 0.0, 1.0])
        p_joint = T_prev[:3, 3]
        J[:3, i] = np.cross(z, p_tool - p_joint)
        J[3:, i] = z
    return J


def task_error(R_target, p_target, R, p):
    """[dp; 0.5·vex(R_targetᵀR − RᵀR_target)] — standard 6D task error."""
    A = R_target.T @ R - R.T @ R_target
    eo = 0.5 * np.array([A[2, 1], A[0, 2], A[1, 0]])
    return np.concatenate([np.asarray(p_target) - p, eo])


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def rotvec(R):
    """Rotation matrix → rotation vector (axis*angle), numpy-only."""
    R = np.asarray(R, dtype=float)
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(tr)
    if th < 1e-8:
        return np.zeros(3)
    s = np.sin(th)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * s) * th


def ik_dls(target_pos, target_rot, q0, n_iter=300, tol=1e-8, damp=1e-4):
    """Damped least-squares / Newton IK. Returns (q, ok), ok if pos_err < 2 mm."""
    q = np.array(q0, dtype=float)
    target_rot = np.asarray(target_rot, dtype=float)
    for _ in range(n_iter):
        T, frames = fk_with_frames(q)
        p, R = T[:3, 3], T[:3, :3]
        e = task_error(target_rot, target_pos, R, p)
        if np.linalg.norm(e) < 1e-8:
            break
        J = geometric_jacobian(frames, p)
        dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), e)
        q += dq
    err = np.linalg.norm(fk(q)[:3, 3] - np.asarray(target_pos))
    return q, err < 2e-3


# ── fixed orientation (ACTUAL URDF base frame) from the working YAML ─────────
QUAT = np.array([-0.000199, 0.788926, 0.000129, 0.614488])
ROT = quat_to_matrix(QUAT)

# Frame fix: the generator's DH model has its base rotated 180° about z
# relative to the URDF/sim base frame. Verified against all 8 working
# waypoints:  Rz(pi) @ DH_fk(q) == actual base-frame tool0 pose.
RZ = np.eye(4)
RZ[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])


def act_to_dh(T_act):
    """Actual base-frame tool0 pose → DH-model base-frame pose."""
    return RZ @ T_act

UR_LIMITS = [
    (-6.28319, 6.28319), (-2.618, 2.618), (-3.1416, 3.1416),
    (-6.28319, 6.28319), (-6.28319, 6.28319), (-6.28319, 6.28319),
]


def in_limits(q):
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, UR_LIMITS))


def world_to_base(wx, wy, wz):
    """world (wx, wy, wz) → base (bx, by, bz)."""
    return wy, -wx, wz - 0.8


def verify_yaml():
    """Recompute the working YAML's joint configs from its base poses.

    Each YAML pose is in the ACTUAL base frame; transform to the DH frame
    (Rz·pose), solve IK there, and compare with the stored joint configs.
    """
    with open("/ws/src/gz_spray_painting_plugin_demo/config/cartesian_poses.yaml") as f:
        data = yaml.safe_load(f)
    cfg = {c["name"]: c for c in data["joint_configs"]}
    q_seed = None
    ok_all = True
    for pose in data["poses"]:
        p = pose["position"]
        y = pose["orientation"]
        name = pose["name"]
        want = [cfg[name][j] for j in JOINT_NAMES]
        # actual frame target
        Tact = np.eye(4)
        Tact[:3, 3] = [p["x"], p["y"], p["z"]]
        Tact[:3, :3] = quat_to_matrix(np.array([y["x"], y["y"], y["z"], y["w"]]))
        Tdh = act_to_dh(Tact)
        q, ok = ik_dls(Tdh[:3, 3], Tdh[:3, :3],
                       want if q_seed is None else q_seed)
        q_seed = q
        err = np.linalg.norm(fk(q)[:3, 3] - Tdh[:3, 3])
        jerr = max(abs(q[i] - want[i]) for i in range(6))
        ok_all &= ok
        print(f"{name}: pos_err={err*1000:5.1f} mm  max|Δq|={jerr*1000:7.1f} mrad  "
              f"{'OK' if ok else 'FAIL'}")
    print("IK self-check vs working YAML:", "ALL OK" if ok_all else "MISMATCH")


def map_face(standoff):
    """Feasibility map. Each cell is tested with vertical chaining seed, then
    the known-good wp0 branch, then zeros — so a reachable cell is never
    missed just because one seed landed on a bad IK branch."""
    WX = np.round(np.arange(-0.55, 0.56, 0.05), 2)
    WZ = np.round(np.arange(0.10, 1.61, 0.05), 2)

    # known-good wp0 branch (world x≈+0.35, z≈1.03, standoff 0.36)
    WP0 = np.array([2.52402, -1.85965, -1.87007, -2.85389, -2.16736, -1.74316])

    cells = {}
    idx = {(wx, wz): (i, j) for i, wx in enumerate(WX) for j, wz in enumerate(WZ)}
    print(f"standoff world_y={standoff} (dist to face = {0.945 - standoff:.3f} m)")
    print("x\\z " + "".join(f"{z:>5}" for z in WZ))

    def try_seeds(bx, by, bz, seeds):
        for s in seeds:
            cand, ok = ik_dls([bx, by, bz], ROT, s, n_iter=120, tol=1e-6)
            if ok and in_limits(cand):
                return cand
        return None

    # Flood-fill: seed each unsolved cell from its already-solved neighbours
    # (8-connected), plus the known-good branch and zeros as fallbacks. This
    # propagates the reachable region across IK branches, giving a connected
    # map instead of the scattered result of single-seed chains.
    from collections import deque
    frontier = deque()
    for wx in WX:
        for wz in WZ:
            bx, by, bz = world_to_base(wx, standoff, wz)
            dist = np.sqrt(bx * bx + by * by + bz * bz)
            if not (0.2 <= dist <= 0.86):
                continue
            q = try_seeds(bx, by, bz, [WP0, np.zeros(6)])
            if q is not None:
                cells[(wx, wz)] = q
                frontier.append((wx, wz))

    while frontier:
        wx, wz = frontier.popleft()
        for dxx in (-1, 0, 1):
            for dzz in (-1, 0, 1):
                if dxx == 0 and dzz == 0:
                    continue
                nx, nz = wx + 0.05 * dxx, wz + 0.05 * dzz
                if (nx, nz) in cells or (nx, nz) not in idx:
                    continue
                bx, by, bz = world_to_base(nx, nz, standoff)
                q = try_seeds(bx, by, bz, [cells[(wx, wz)]])
                if q is None:
                    q = try_seeds(bx, by, bz, [WP0, np.zeros(6)])
                if q is not None:
                    cells[(nx, nz)] = q
                    frontier.append((nx, nz))

    for wx in WX:
        line = f"{wx:>5} "
        for wz in WZ:
            if (wx, wz) in cells:
                line += "    ·"
            else:
                bx, by, bz = world_to_base(wx, standoff, wz)
                dist = np.sqrt(bx * bx + by * by + bz * bz)
                line += "    -" if not (0.2 <= dist <= 0.86) else "    X"
        print(line, flush=True)

    # per-column reachable z range
    print("\nper-column reachable z range:")
    for wx in WX:
        zs = sorted(wz for (x, wz) in cells if x == wx)
        if zs:
            print(f"  x={wx:+.2f}: z [{min(zs):.2f}, {max(zs):.2f}]  span {max(zs)-min(zs):.2f}")
    xr = sorted({x for (x, z) in cells})
    print(f"\ncolumns with ≥1 reachable cell: {len(xr)}/{len(WX)} x∈[{min(xr):.2f},{max(xr):.2f}]")

    # biggest fully-reachable rectangle (greedy over z-window)
    best = None
    for z_lo in np.round(np.arange(0.10, 1.61, 0.05), 2):
        for z_hi in np.round(np.arange(z_lo, 1.61, 0.05), 2):
            zs = np.round(np.arange(z_lo, z_hi + 1e-9, 0.05), 2)
            xs = [wx for wx in WX if all((wx, wz) in cells for wz in zs)]
            if len(xs) > 2 and len(zs) > 1:
                span = z_hi - z_lo
                area = span * len(xs)
                if best is None or area > best[0]:
                    best = (area, span, len(xs), z_lo, z_hi, min(xs), max(xs))
    if best:
        print(f"best reachable rectangle: z∈[{best[3]:.2f},{best[4]:.2f}] "
              f"x∈[{best[5]:.2f},{best[6]:.2f}]  ({best[2]} cols × {best[1]:.2f} m tall)")
    return cells


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--map":
        map_face(float(sys.argv[2]) if len(sys.argv) > 2 else 0.36)
    else:
        verify_yaml()
