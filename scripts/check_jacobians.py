#!/usr/bin/env python3
"""
Check the wheel factor's analytical Jacobians against numerical differentiation.

The derivatives in include/viwo/wheel_odometry_factor.hpp are the actual
contribution of this repository, and they are the part that fails quietly: a
wrong derivative raises no error, it produces an optimiser that converges
slowly, or to the wrong answer, or appears to work until the problem gets
harder.

tests/test_jacobians.cpp already checks them against GTSAM's own
numericalDerivative, which is the check that matters because it uses the same
retraction the optimiser uses. This script is the independent one: it
reimplements the residual, the retraction and the derivatives from scratch in
numpy, with no GTSAM involved, so a shared misunderstanding of GTSAM's
conventions cannot hide in both. Agreement between the two is evidence; either
one alone is not.

The residual, for two SE(3) keyframe poses T_i and T_j:

    r(T_i, T_j) = R_i^T (p_j - p_i) - dp_wheel

The retraction is GTSAM's, a right perturbation on the manifold,
T -> T * Expmap(xi), with xi = [omega, v] ordered rotation first. Under it:

    H1 = [ skew(R_i^T (p_j - p_i))   -I ]
    H2 = [ 0                          R_i^T R_j ]

Usage:
    python3 scripts/check_jacobians.py [trials]

Requires numpy only.
"""

import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover - environment dependent
    # 77 is the exit code CMakeLists.txt registers as "skipped", so a machine
    # without numpy reports this check as not run rather than as failing.
    print("numpy is not installed, skipping the independent Jacobian check")
    sys.exit(77)


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def so3_exp(omega):
    """Rodrigues. Returns the rotation matrix for an axis-angle vector."""
    theta = np.linalg.norm(omega)
    K = skew(omega)
    if theta < 1e-12:
        # Second order is enough here and avoids dividing by zero.
        return np.eye(3) + K + 0.5 * K @ K
    return (np.eye(3)
            + (np.sin(theta) / theta) * K
            + ((1.0 - np.cos(theta)) / theta ** 2) * (K @ K))


def se3_exp(xi):
    """GTSAM's Pose3 Expmap: xi = [omega, v], rotation first.

    The translation picks up the left Jacobian V(omega) rather than v itself.
    That term is what makes this the group exponential rather than a naive
    concatenation, and leaving it out would still pass a first-order check,
    which is exactly why it is written out here.
    """
    omega, v = xi[:3], xi[3:]
    R = so3_exp(omega)
    theta = np.linalg.norm(omega)
    K = skew(omega)
    if theta < 1e-12:
        V = np.eye(3) + 0.5 * K
    else:
        V = (np.eye(3)
             + ((1.0 - np.cos(theta)) / theta ** 2) * K
             + ((theta - np.sin(theta)) / theta ** 3) * (K @ K))
    return R, V @ v


def compose(pose, xi):
    """Right retraction: T * Expmap(xi)."""
    R, p = pose
    dR, dp = se3_exp(xi)
    return R @ dR, p + R @ dp


def residual(pose_i, pose_j, dp):
    R_i, p_i = pose_i
    _, p_j = pose_j
    return R_i.T @ (p_j - p_i) - dp


def analytic(pose_i, pose_j):
    R_i, p_i = pose_i
    R_j, p_j = pose_j
    p_rel_body = R_i.T @ (p_j - p_i)

    H1 = np.zeros((3, 6))
    H1[:, 0:3] = skew(p_rel_body)
    H1[:, 3:6] = -np.eye(3)

    H2 = np.zeros((3, 6))
    H2[:, 0:3] = 0.0
    H2[:, 3:6] = R_i.T @ R_j
    return H1, H2


def numeric(pose_i, pose_j, dp, which, h=1e-6):
    """Central differences taken under the same retraction."""
    J = np.zeros((3, 6))
    for k in range(6):
        step = np.zeros(6)
        step[k] = h
        if which == 1:
            plus = residual(compose(pose_i, step), pose_j, dp)
            minus = residual(compose(pose_i, -step), pose_j, dp)
        else:
            plus = residual(pose_i, compose(pose_j, step), dp)
            minus = residual(pose_i, compose(pose_j, -step), dp)
        J[:, k] = (plus - minus) / (2.0 * h)
    return J


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    rng = np.random.default_rng(1)

    worst_h1 = 0.0
    worst_h2 = 0.0
    for _ in range(trials):
        pose_i = (so3_exp(rng.uniform(-1.0, 1.0, 3)), rng.uniform(-3.0, 3.0, 3))
        pose_j = (so3_exp(rng.uniform(-1.0, 1.0, 3)), rng.uniform(-3.0, 3.0, 3))
        dp = rng.uniform(-1.0, 1.0, 3)

        H1, H2 = analytic(pose_i, pose_j)
        N1 = numeric(pose_i, pose_j, dp, 1)
        N2 = numeric(pose_i, pose_j, dp, 2)

        worst_h1 = max(worst_h1, np.abs(H1 - N1).max())
        worst_h2 = max(worst_h2, np.abs(H2 - N2).max())

    print(f"Wheel factor Jacobians, {trials} random pose pairs, numpy only")
    print(f"  H1 (w.r.t. pose i): max |analytic - numerical| = {worst_h1:.3e}")
    print(f"  H2 (w.r.t. pose j): max |analytic - numerical| = {worst_h2:.3e}")

    # Central differences at h = 1e-6 carry an error of order 1e-10 on
    # quantities of this size, so the threshold checks the analytical form
    # rather than the finite difference.
    tol = 1e-5
    ok = worst_h1 < tol and worst_h2 < tol
    print(f"  {'PASS' if ok else 'FAIL'} (tolerance {tol:.0e})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
