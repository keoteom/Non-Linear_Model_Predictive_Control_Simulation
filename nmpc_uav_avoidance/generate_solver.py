#!/usr/bin/env python3
"""
OpEn NMPC Solver Generator — Dynamic Obstacle + SFC half-space constraints
==========================================================================

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
HYBRID-CONSTRAINT VERSION  (SFC = HARD, dynamic obstacle = SOFT)
---------------------------------------------------------------
The SFC half-space (corridor) constraints are enforced as **hard inequality
constraints via Augmented Lagrangian (ALM)** in F1. The dynamic spherical
obstacle is now a **soft quadratic penalty added directly into the cost**
(weight W_OBS), NOT a hard constraint.

Rationale: keeping the corridor hard but the obstacle soft means that when a
moving obstacle clips the corridor the optimiser yields on the obstacle term
instead of the whole problem going jointly infeasible. The trade-off is that
obstacle avoidance is no longer guaranteed — with a finite W_OBS the solver
may return a "Converged" solution that still penetrates the obstacle ball if
tracking + corridor cost outweighs the penalty. Tune W_OBS accordingly.

Constraint form for ALM (SFC only):
  F1(z, p) <= 0  via  set_c = Rectangle([-inf,...,-inf], [0,...,0])
                      set_y = BallInf(None, c_max)   (multiplier ball)

The soft obstacle penalty is folded into `cost`, so it does NOT share the
penalty weight of the input-rate soft constraints in F2 (avoids the units
mismatch between m^2 obstacle residuals and rad input-rate residuals).
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Parameter vector layout (total = 108 + 4*M_MAX*N = 1548):
  [0:8]      x0 = [px,py,pz, vx,vy,vz, phi,theta]
  [8:11]     u_prev = [T, phi_ref, theta_ref]_{k-1}
  [11:101]   p_ref  = 30 reference positions (3*30 = 90)
  [101:104]  p_obs  = [px_obs, py_obs, pz_obs]
  [104:107]  v_obs  = [vx_obs, vy_obs, vz_obs]
  [107]      r_obs
  [108:108 + 4*M_MAX*N]   SFC planes per horizon step
      For step j, plane m: 4 values [n_x, n_y, n_z, b]
      Layout: planes[j][m] starts at  108 + (j*M_MAX + m)*4
      Inactive planes are zero-padded (n=0, b=0) -> g = -b = 0 always.
"""

import casadi as cs
import opengen as og
import os

# ---------------------------------------------------------------------------
#  Horizon and dynamics
# ---------------------------------------------------------------------------
N  = 30
Ts = 0.05
nx = 8
nu = 3

# SFC: per-step plane budget. Pad with zeros if fewer planes are active.
M_MAX = 12

# Drone dynamics constants (must match nmpc_node.cpp)
tau_phi   = 0.5
tau_theta = 0.5
K_phi     = 1.0
K_theta   = 1.0
Ax, Ay, Az = 0.1, 0.1, 0.2
g = 9.81

# Cost weights
Qx  = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0, 8.0, 8.0]
Qu  = [5.0, 10.0, 10.0]
QdU = [5.0, 12.0, 12.0]

u_ref_vals = [g, 0.0, 0.0]

# Input bounds (rectangle on the decision variable)
u_min = [5.0, -0.35, -0.35]
u_max = [13.5, 0.35, 0.35]

# Input-rate limits
DELTA_PHI_MAX   = 0.08
DELTA_THETA_MAX = 0.08

# Dynamic-obstacle softening radius (linear ramp 0 -> R_S_MAX over horizon).
# Smaller value reduces risk of SFC ∩ obstacle becoming jointly infeasible.
R_S_MAX = 0.3

# Dynamic-obstacle SOFT penalty weight (quadratic, added to cost).
# TUNE THIS. The penalty term is  W_OBS * fmax(0, g_obs)^2, where g_obs has
# units of m^2. Compare against the position tracking weights Qx[0:3] = 5.0
# acting on dx^2 (also m^2):
#   - too small -> drone clips the obstacle (soft = no guarantee)
#   - too large -> stiff problem; can re-introduce the corridor-vs-obstacle
#                  conflict that motivated the all-hard version.
# 1e3 is a reasonable starting point (penalty ~200x the per-axis tracking
# weight); verify with /nmpc/obs_violation in PlotJuggler and adjust.
W_OBS = 1.0e3

# ---------------------------------------------------------------------------
#  Parameter layout
# ---------------------------------------------------------------------------
SFC_START = 108
SFC_BLOCK = 4 * M_MAX * N            # 4 * 12 * 30 = 1440
N_PARAMS  = SFC_START + SFC_BLOCK    # 108 + 1440 = 1548

# Decision variable size
n_z = nu * N                         # 3 * 30 = 90


def uav_dynamics_euler(x, u):
    """Forward Euler step of the simplified UAV model."""
    px, py, pz = x[0], x[1], x[2]
    vx, vy, vz = x[3], x[4], x[5]
    phi, theta = x[6], x[7]

    T_thrust    = u[0]
    phi_ref_u   = u[1]
    theta_ref_u = u[2]

    ax =  T_thrust * cs.cos(phi) * cs.sin(theta)
    ay = -T_thrust * cs.sin(phi)
    az =  T_thrust * cs.cos(phi) * cs.cos(theta) - g

    return cs.vertcat(
        px + Ts * vx,
        py + Ts * vy,
        pz + Ts * vz,
        vx + Ts * (ax - Ax * vx),
        vy + Ts * (ay - Ay * vy),
        vz + Ts * (az - Az * vz),
        phi   + Ts / tau_phi   * (K_phi   * phi_ref_u   - phi),
        theta + Ts / tau_theta * (K_theta * theta_ref_u - theta),
    )


def sfc_offset(j, m):
    """Start index in p of plane m at step j (4 values: n_x, n_y, n_z, b)."""
    return SFC_START + (j * M_MAX + m) * 4


def build_problem():
    z = cs.SX.sym('z', n_z)
    p = cs.SX.sym('p', N_PARAMS)

    x0     = p[0:8]
    u_prev = p[8:11]

    p_ref_start = 11
    p_obs = p[101:104]
    v_obs = p[104:107]
    r_obs = p[107]

    u_ref = cs.vertcat(*u_ref_vals)

    cost           = 0.0
    g_hard_list    = []   # ALM hard inequality constraints g_i(z,p) <= 0
    g_soft_list    = []   # Soft penalty constraints (input rate, slack-friendly)
    x_k            = x0

    for j in range(N):
        uj      = z[j * nu : (j + 1) * nu]
        uj_prev = u_prev if j == 0 else z[(j - 1) * nu : j * nu]

        # Reference at step j
        p_ref_j = p[p_ref_start + j * 3 : p_ref_start + j * 3 + 3]
        x_ref_j = cs.vertcat(p_ref_j, cs.SX.zeros(5, 1))

        # ---- State tracking cost ----
        dx = x_ref_j - x_k
        for i in range(nx):
            cost += Qx[i] * dx[i] ** 2

        # ---- Input cost ----
        du_ref = u_ref - uj
        for i in range(nu):
            cost += Qu[i] * du_ref[i] ** 2

        # ---- Input-rate cost ----
        du = uj - uj_prev
        for i in range(nu):
            cost += QdU[i] * du[i] ** 2

        # ---- SOFT: Dynamic spherical obstacle (quadratic penalty in cost) ----
        # g_obs > 0  <=> pos inside the (r_obs + r_s_j) ball -> penalise.
        # SFC stays HARD (F1); the obstacle being soft lets the optimiser
        # yield here when the obstacle clips the corridor, instead of the
        # whole problem going infeasible.
        p_obs_j = p_obs + j * Ts * v_obs
        r_s_j   = R_S_MAX * j / N
        dp      = x_k[0:3] - p_obs_j
        dist_sq = cs.dot(dp, dp)
        g_obs   = (r_obs + r_s_j) ** 2 - dist_sq
        cost   += W_OBS * cs.fmax(0.0, g_obs) ** 2

        # ---- HARD: SFC half-space constraints ----
        # n_jm . pos_k - b_jm <= 0   (inactive: n=0, b=0 -> 0 <= 0 trivially)
        pos_k = x_k[0:3]
        for m in range(M_MAX):
            off  = sfc_offset(j, m)
            n_jm = p[off : off + 3]
            b_jm = p[off + 3]
            g_sfc = cs.dot(n_jm, pos_k) - b_jm
            g_hard_list.append(g_sfc)

        # ---- SOFT: Input-rate constraints (kept as penalty) ----
        # These are simple, never infeasible, and cheaper as soft penalty.
        g_soft_list.append(cs.fmax(0.0, uj_prev[1] - uj[1] - DELTA_PHI_MAX))
        g_soft_list.append(cs.fmax(0.0, uj[1] - uj_prev[1] - DELTA_PHI_MAX))
        g_soft_list.append(cs.fmax(0.0, uj_prev[2] - uj[2] - DELTA_THETA_MAX))
        g_soft_list.append(cs.fmax(0.0, uj[2] - uj_prev[2] - DELTA_THETA_MAX))

        # Roll-out
        x_k = uav_dynamics_euler(x_k, uj)

    # ---- Terminal cost ----
    p_ref_N = p[p_ref_start + (N - 1) * 3 : p_ref_start + (N - 1) * 3 + 3]
    x_ref_N = cs.vertcat(p_ref_N, cs.SX.zeros(5, 1))
    dx = x_ref_N - x_k
    for i in range(nx):
        cost += Qx[i] * dx[i] ** 2

    # ---- SOFT: Terminal dynamic-obstacle penalty ----
    p_obs_N = p_obs + N * Ts * v_obs
    dp      = x_k[0:3] - p_obs_N
    dist_sq = cs.dot(dp, dp)
    g_obs_N = (r_obs + R_S_MAX) ** 2 - dist_sq
    cost   += W_OBS * cs.fmax(0.0, g_obs_N) ** 2

    # ---- HARD: Terminal SFC (re-use last step's planes, j = N-1) ----
    pos_N = x_k[0:3]
    for m in range(M_MAX):
        off  = sfc_offset(N - 1, m)
        n_jm = p[off : off + 3]
        b_jm = p[off + 3]
        g_sfc_N = cs.dot(n_jm, pos_N) - b_jm
        g_hard_list.append(g_sfc_N)

    F1 = cs.vertcat(*g_hard_list)
    F2 = cs.vertcat(*g_soft_list)
    return z, p, cost, F1, F2


def generate():
    z, p, cost, F1, F2 = build_problem()

    n_F1 = F1.shape[0]
    print(f"[INFO] # hard ALM constraints (F1): {n_F1}")
    print(f"[INFO] # soft penalty constraints (F2): {F2.shape[0]}")

    # Rectangle bounds on the decision variable (input box)
    bounds = og.constraints.Rectangle(u_min * N, u_max * N)

    # Hard inequality F1 <= 0  is encoded via set_c = Rectangle([-inf, 0])
    # (componentwise: each F1[i] in (-inf, 0] => F1[i] <= 0)
    inf = float("inf")
    set_c = og.constraints.Rectangle([-inf] * n_F1, [0.0] * n_F1)

    # Lagrange-multiplier domain (BallInf cap; large but bounded for stability)
    set_y = og.constraints.BallInf(None, 1.0e6)

    problem = (
        og.builder.Problem(z, p, cost)
        .with_aug_lagrangian_constraints(F1, set_c, set_y)
        .with_penalty_constraints(F2)
        .with_constraints(bounds)
    )

    meta = og.config.OptimizerMeta().with_optimizer_name("nmpc_uav_obstacle")

    build_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "nmpc_solver")

    build_config = (
        og.config.BuildConfiguration()
        .with_build_directory(build_dir)
        .with_build_mode("release")
        .with_build_c_bindings()
    )

    # ALM tuning:
    #   - tighter outer convergence tolerance on F1 (delta) for hard feel
    #   - more outer iterations to drive multipliers to KKT values
    #   - moderate penalty update; too aggressive -> ill-conditioning
    solver_config = (
        og.config.SolverConfiguration()
        .with_tolerance(1e-4)
        .with_delta_tolerance(1e-3)            # F1 feasibility tolerance
        .with_max_outer_iterations(10)
        .with_max_inner_iterations(800)
        .with_penalty_weight_update_factor(5.0)
        .with_initial_penalty(20.0)
        .with_sufficient_decrease_coefficient(0.7)
    )

    builder = og.builder.OpEnOptimizerBuilder(
        problem, meta, build_config, solver_config
    )
    builder.build()

    print("\n[OK] Solver compiled: HARD (ALM) SFC + SOFT (cost) obstacle.")
    print(f"     N_PARAMS         : {N_PARAMS}")
    print(f"     decision vars    : {n_z}")
    print(f"     M_MAX (per step) : {M_MAX}")
    print(f"     SFC block size   : {SFC_BLOCK}  (4 * {M_MAX} * {N})")
    print(f"     SFC start index  : {SFC_START}")
    print(f"     # hard (ALM) F1  : {n_F1}")
    print()


if __name__ == "__main__":
    generate()