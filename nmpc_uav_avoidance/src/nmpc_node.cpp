/**
 * @file nmpc_node.cpp
 * @brief C++ NMPC UAV — RILS static SFC + dynamic-obstacle TV linear cutting plane.
 *
 *  The dynamic obstacle is converted, per horizon step, into a single LINEAR
 *  cutting plane (RSFC-style separating hyperplane) and packed into SFC slot
 *  m = M_MAX-1. The previous nonlinear quadratic distance constraint has been
 *  removed from the OpEn build (see build_solver.py).
 *
 *  Constraint-violation diagnostics for PlotJuggler. Subscribe to:
 *    /nmpc/obs_violation        - current  (>0 => inside r_ego+r_obs)
 *    /nmpc/obs_distance         - current  ||drone - obs||
 *    /nmpc/sfc_violation        - current  (>0 => outside RILS SFC)
 *    /nmpc/sfc_margin           - per-plane signed margin (positive = safe)
 *    /nmpc/pred_obs_violation   - predicted over horizon (k=0..N)
 *    /nmpc/pred_sfc_violation   - predicted over horizon (k=0..N)
 *    /nmpc/solver_converged     - 1.0 if exit_status=Converged else 0.0
 */

#include "nmpc_uav_avoidance/nmpc_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>

namespace nmpc_uav {

// ============================================================================
//  Free helpers
// ============================================================================

EulerAngles quaternion_to_euler(double qx, double qy, double qz, double qw) {
    EulerAngles e;
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    e.roll = std::atan2(sinr, cosr);

    double sinp = 2.0 * (qw * qy - qz * qx);
    sinp = std::clamp(sinp, -1.0, 1.0);
    e.pitch = std::asin(sinp);

    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    e.yaw = std::atan2(siny, cosy);
    return e;
}

Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    double cr = std::cos(roll/2.0), sr = std::sin(roll/2.0);
    double cp = std::cos(pitch/2.0), sp = std::sin(pitch/2.0);
    double cy = std::cos(yaw/2.0),   sy = std::sin(yaw/2.0);
    return {
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    };
}

std::array<double, NX> dynamics_step(const std::array<double, NX>& x,
                                     const std::array<double, NU>& u) {
    double px=x[0], py=x[1], pz=x[2];
    double vx=x[3], vy=x[4], vz=x[5];
    double phi=x[6], theta=x[7];
    double T_t=u[0], phi_ref=u[1], theta_ref=u[2];

    double ax =  T_t * std::cos(phi) * std::sin(theta);
    double ay = -T_t * std::sin(phi);
    double az =  T_t * std::cos(phi) * std::cos(theta) - GRAVITY;

    return {
        px + Ts*vx,
        py + Ts*vy,
        pz + Ts*vz,
        vx + Ts*(ax - AX*vx),
        vy + Ts*(ay - AY*vy),
        vz + Ts*(az - AZ*vz),
        phi   + (Ts/TAU_PHI)  *(K_PHI  *phi_ref   - phi),
        theta + (Ts/TAU_THETA)*(K_THETA*theta_ref - theta)
    };
}

// ============================================================================
//  Constructor / destructor
// ============================================================================

NMPCNode::NMPCNode() : Node("nmpc_uav_node") {
    this->declare_parameter<double>("obs_x",      3.0);
    this->declare_parameter<double>("obs_y",      0.0);
    this->declare_parameter<double>("obs_z",      1.0);
    this->declare_parameter<double>("obs_vx",     0.0);
    this->declare_parameter<double>("obs_vy",     0.0);
    this->declare_parameter<double>("obs_vz",     0.0);
    this->declare_parameter<double>("r_obs",      0.5);
    this->declare_parameter<double>("r_ego",      0.25);
    this->declare_parameter<double>("thrust_max", 13.5);
    this->declare_parameter<double>("thrust_min", 5.0);
    this->declare_parameter<double>("yaw_gain",   1.0);

    obs_pos_[0] = this->get_parameter("obs_x").as_double();
    obs_pos_[1] = this->get_parameter("obs_y").as_double();
    obs_pos_[2] = this->get_parameter("obs_z").as_double();
    obs_vel_[0] = this->get_parameter("obs_vx").as_double();
    obs_vel_[1] = this->get_parameter("obs_vy").as_double();
    obs_vel_[2] = this->get_parameter("obs_vz").as_double();
    r_obs_  = this->get_parameter("r_obs").as_double();
    r_ego_  = this->get_parameter("r_ego").as_double();
    T_max_  = this->get_parameter("thrust_max").as_double();
    T_min_  = this->get_parameter("thrust_min").as_double();
    K_psi_  = this->get_parameter("yaw_gain").as_double();

    tv_normal_prev_.assign(N, {0.0, 0.0, 0.0});

    RCLCPP_INFO(this->get_logger(), "Initialising OpEn solver (C bindings)...");
    solver_ = std::make_unique<OpEnSolver>();
    solver_->start();
    RCLCPP_INFO(this->get_logger(), "OpEn solver ready. N_PARAMS=%d", N_PARAMS);

    rmw_qos_profile_t qos_raw = rmw_qos_profile_sensor_data;
    auto qos_px4 = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_raw.history, 1), qos_raw);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_nmpc", 10,
        std::bind(&NMPCNode::odom_callback, this, std::placeholders::_1));
    sub_obs_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/obstacle/odom", 10,
        std::bind(&NMPCNode::obs_odom_callback, this, std::placeholders::_1));
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&NMPCNode::path_callback, this, std::placeholders::_1));
    // NOTE: option A (TV plane built inside this node) -> keep the RAW RILS
    // SFC topic. We do NOT subscribe to "/sfc_coefficients_tv"; that topic
    // only exists in the separate-node design (option B).
    sub_sfc_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/sfc_coefficients", 10,
        std::bind(&NMPCNode::sfc_callback, this, std::placeholders::_1));
    sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos_px4,
        std::bind(&NMPCNode::vehicle_status_callback, this, std::placeholders::_1));

    pub_thrust_       = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
        "/fmu/in/vehicle_thrust_setpoint", qos_px4);
    pub_att_          = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint_v1", qos_px4);
    pub_offboard_     = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_px4);
    pub_command_      = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_px4);
    pub_pred_path_    = this->create_publisher<nav_msgs::msg::Path>(
        "/nmpc/predicted_path", 10);
    pub_obs_marker_   = this->create_publisher<visualization_msgs::msg::Marker>(
        "/nmpc/obstacle_marker", 10);
    pub_tv_plane_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/nmpc/tv_plane_marker", 10);
    pub_solver_time_  = this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/solver_time_ms", 10);
    pub_cost_function_= this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/cost_function", 10);
    pub_control_input_= this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/nmpc/control_input", 10);
    pub_jerk_         = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/nmpc/jerk", 10);

    // Constraint diagnostics
    pub_obs_viol_         = this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/obs_violation", 10);
    pub_obs_dist_         = this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/obs_distance", 10);
    pub_sfc_viol_         = this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/sfc_violation", 10);
    pub_sfc_margin_       = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/nmpc/sfc_margin", 10);
    pub_pred_obs_viol_    = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/nmpc/pred_obs_violation", 10);
    pub_pred_sfc_viol_    = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/nmpc/pred_sfc_violation", 10);
    pub_solver_converged_ = this->create_publisher<std_msgs::msg::Float64>(
        "/nmpc/solver_converged", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(Ts * 1000)),
        std::bind(&NMPCNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
        "NMPC node ready (RILS SFC + dyn-obs TV plane, %d params, M_MAX=%d, "
        "TV_SLOT=%d).", N_PARAMS, M_MAX, TV_SLOT);
}

NMPCNode::~NMPCNode() {
    RCLCPP_INFO(this->get_logger(), "Disarming and shutting down...");
    disarm();
    if (solver_) solver_->kill_solver();
}

// ============================================================================
//  Callbacks
// ============================================================================

void NMPCNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto& p = msg->pose.pose.position;
    auto& v = msg->twist.twist.linear;
    auto& q = msg->pose.pose.orientation;
    EulerAngles euler = quaternion_to_euler(q.x, q.y, q.z, q.w);

    state_[0] = p.x;  state_[1] = p.y;  state_[2] = p.z;
    state_[3] = v.x;  state_[4] = v.y;  state_[5] = v.z;
    state_[6] = -euler.roll;
    state_[7] = -euler.pitch;
    yaw_            = euler.yaw;
    state_received_ = true;
}

void NMPCNode::obs_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    obs_pos_[0] = msg->pose.pose.position.x;
    obs_pos_[1] = msg->pose.pose.position.y;
    obs_pos_[2] = msg->pose.pose.position.z;
    obs_vel_[0] = msg->twist.twist.linear.x;
    obs_vel_[1] = msg->twist.twist.linear.y;
    obs_vel_[2] = msg->twist.twist.linear.z;
    obs_received_ = true;
}

void NMPCNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_points_.clear();
    for (const auto& ps : msg->poses) {
        path_points_.push_back({
            ps.pose.position.x,
            ps.pose.position.y,
            ps.pose.position.z
        });
    }
    path_idx_          = 0;
    last_solve_time_s_ = Ts;
    RCLCPP_INFO(this->get_logger(),
        "Received path with %zu waypoints.", path_points_.size());
}

void NMPCNode::sfc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const auto& d = msg->data;
    if (d.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty SFC message.");
        return;
    }

    std::vector<SfcPolytope> sfcs;
    size_t idx = 0;
    int n_poly = static_cast<int>(d[idx++]);
    sfcs.reserve(std::max(0, n_poly));

    for (int i = 0; i < n_poly; ++i) {
        if (idx + 2 > d.size()) {
            RCLCPP_WARN(this->get_logger(), "SFC msg truncated at header %d.", i);
            break;
        }
        SfcPolytope poly;
        poly.segment_index = static_cast<int>(d[idx++]);
        int M = static_cast<int>(d[idx++]);
        if (M < 0 || idx + 4*static_cast<size_t>(M) > d.size()) {
            RCLCPP_WARN(this->get_logger(), "SFC msg malformed at poly %d (M=%d).",
                        i, M);
            break;
        }

        poly.planes.reserve(M);
        for (int m = 0; m < M; ++m) {
            SfcPlane pl;
            pl.n = { d[idx], d[idx+1], d[idx+2] };
            pl.b = d[idx+3];
            idx += 4;
            poly.planes.push_back(pl);
        }
        sfcs.push_back(std::move(poly));
    }

    sfcs_          = std::move(sfcs);
    sfc_received_  = !sfcs_.empty();
    last_poly_idx_ = 0;

    RCLCPP_INFO(this->get_logger(), "Received %zu SFC polytopes.", sfcs_.size());
}

void NMPCNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    nav_state_    = msg->nav_state;
    arming_state_ = msg->arming_state;
}

// ============================================================================
//  PX4 helpers
// ============================================================================

void NMPCNode::publish_vehicle_command(uint16_t command,
                                       float p1, float p2, float p3, float p4,
                                       float p5, float p6, float p7) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp        = px4_timestamp();
    msg.param1 = p1; msg.param2 = p2; msg.param3 = p3; msg.param4 = p4;
    msg.param5 = static_cast<double>(p5);
    msg.param6 = static_cast<double>(p6);
    msg.param7 = p7;
    msg.command          = command;
    msg.target_system    = 1;  msg.target_component  = 1;
    msg.source_system    = 1;  msg.source_component  = 1;
    msg.from_external    = true;
    pub_command_->publish(msg);
}

void NMPCNode::set_offboard_mode() {
    publish_vehicle_command(176, 1.0f, 6.0f);
    RCLCPP_INFO(this->get_logger(), "Offboard mode requested.");
}
void NMPCNode::arm()    { publish_vehicle_command(400, 1.0f); }
void NMPCNode::disarm() { publish_vehicle_command(400, 0.0f); }

void NMPCNode::publish_offboard_mode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = px4_timestamp();
    msg.attitude  = true;
    pub_offboard_->publish(msg);
}

uint64_t NMPCNode::px4_timestamp() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
}

// ============================================================================
//  Reference / lookahead
// ============================================================================

void NMPCNode::update_reference() {
    if (path_points_.empty()) return;

    double min_dist = 1e9;
    for (size_t i = path_idx_; i < path_points_.size(); ++i) {
        double dx = state_[0] - path_points_[i][0];
        double dy = state_[1] - path_points_[i][1];
        double dz = state_[2] - path_points_[i][2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < min_dist) { min_dist = dist; path_idx_ = i; }
    }

    double seg_len = 0.1;
    if (path_idx_ + 1 < path_points_.size()) {
        double dx = path_points_[path_idx_+1][0] - path_points_[path_idx_][0];
        double dy = path_points_[path_idx_+1][1] - path_points_[path_idx_][1];
        double dz = path_points_[path_idx_+1][2] - path_points_[path_idx_][2];
        seg_len = std::sqrt(dx*dx + dy*dy + dz*dz);
        seg_len = std::max(seg_len, 0.01);
    }

    double steps          = last_solve_time_s_ / Ts;
    double lookahead_dist = seg_len * steps;
    lookahead_dist = std::clamp(lookahead_dist, seg_len, seg_len * 10.0);

    double accum = 0.0;
    for (size_t i = path_idx_; i + 1 < path_points_.size(); ++i) {
        double dx = path_points_[i+1][0] - path_points_[i][0];
        double dy = path_points_[i+1][1] - path_points_[i][1];
        double dz = path_points_[i+1][2] - path_points_[i][2];
        accum += std::sqrt(dx*dx + dy*dy + dz*dz);
        if (accum >= lookahead_dist) {
            path_idx_ = i + 1;
            break;
        }
    }
}

void NMPCNode::global_to_body_angles(double phi_g, double theta_g,
                                     double& phi_b, double& theta_b) const {
    double c = std::cos(yaw_), s = std::sin(yaw_);
    phi_b   =  c * phi_g + s * theta_g;
    theta_b = -s * phi_g + c * theta_g;
}

// ============================================================================
//  SFC: polytope assignment + parameter packing
// ============================================================================

bool NMPCNode::point_inside_polytope(const std::array<double,3>& p,
                                     const std::vector<SfcPlane>& planes,
                                     double tol) const {
    for (const auto& pl : planes) {
        double v = pl.n[0]*p[0] + pl.n[1]*p[1] + pl.n[2]*p[2] - pl.b;
        if (v > tol) return false;
    }
    return true;
}

int NMPCNode::find_closest_polytope(const std::array<double,3>& p) const {
    int best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < sfcs_.size(); ++i) {
        const auto& w0 = sfcs_[i].w0;
        const auto& w1 = sfcs_[i].w1;
        std::array<double,3> ab = {w1[0]-w0[0], w1[1]-w0[1], w1[2]-w0[2]};
        double ab2 = ab[0]*ab[0] + ab[1]*ab[1] + ab[2]*ab[2];
        double d;
        if (ab2 < 1e-12) {
            double dx = p[0]-w0[0], dy = p[1]-w0[1], dz = p[2]-w0[2];
            d = std::sqrt(dx*dx + dy*dy + dz*dz);
        } else {
            double t = ((p[0]-w0[0])*ab[0]
                      + (p[1]-w0[1])*ab[1]
                      + (p[2]-w0[2])*ab[2]) / ab2;
            t = std::clamp(t, 0.0, 1.0);
            std::array<double,3> q = {w0[0]+t*ab[0],
                                      w0[1]+t*ab[1],
                                      w0[2]+t*ab[2]};
            double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
            d = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        if (d < best_d) { best_d = d; best = static_cast<int>(i); }
    }
    return best;
}

int NMPCNode::find_polytope_for_point(const std::array<double,3>& p,
                                      int idx_start) const {
    if (sfcs_.empty()) return -1;
    const int Np = static_cast<int>(sfcs_.size());
    idx_start = std::clamp(idx_start, 0, Np - 1);

    for (int j = idx_start; j < Np; ++j) {
        if (point_inside_polytope(p, sfcs_[j].planes)) return j;
    }
    for (int j = idx_start - 1; j >= 0; --j) {
        if (point_inside_polytope(p, sfcs_[j].planes)) return j;
    }
    return find_closest_polytope(p);
}

std::vector<int> NMPCNode::assign_polytopes_to_horizon() {
    std::vector<int> assignment(N, -1);
    if (sfcs_.empty() || path_points_.empty()) return assignment;

    std::array<double,3> cur_pos = {state_[0], state_[1], state_[2]};
    int idx = find_polytope_for_point(cur_pos, last_poly_idx_);
    if (idx < 0) idx = 0;
    last_poly_idx_ = idx;

    for (int j = 0; j < N; ++j) {
        size_t ref_i = path_idx_ + j;
        if (ref_i >= path_points_.size())
            ref_i = path_points_.size() - 1;
        const std::array<double,3> p_ref = {
            path_points_[ref_i][0],
            path_points_[ref_i][1],
            path_points_[ref_i][2]
        };
        int new_idx = find_polytope_for_point(p_ref, idx);
        if (new_idx < 0) new_idx = idx;
        assignment[j] = new_idx;
        idx = new_idx;
    }
    return assignment;
}

void NMPCNode::fill_sfc_params(std::vector<double>& params,
                               const std::vector<int>& assignment) const {
    if (sfcs_.empty()) return;

    // When a dynamic obstacle is active, reserve the last slot (TV_SLOT) for
    // the dynamic TV cutting plane, so RILS static planes get at most M_MAX-1.
    // If RILS has more planes than the budget, the first `budget` planes in
    // the polytope's plane list are used (see note in the report: a
    // midpoint-distance selection would need segment endpoints that the
    // current /sfc_coefficients message does not carry).
    const int rils_budget = obs_received_ ? (M_MAX - 1) : M_MAX;

    for (int j = 0; j < N; ++j) {
        int p_idx = assignment[j];
        if (p_idx < 0 || p_idx >= static_cast<int>(sfcs_.size())) continue;
        const auto& planes = sfcs_[p_idx].planes;

        const int M = std::min(static_cast<int>(planes.size()), rils_budget);
        for (int m = 0; m < M; ++m) {
            const int off = SFC_START + (j * M_MAX + m) * 4;
            params[off + 0] = planes[m].n[0];
            params[off + 1] = planes[m].n[1];
            params[off + 2] = planes[m].n[2];
            params[off + 3] = planes[m].b;
        }
    }
}

// ============================================================================
//  Dynamic-obstacle Time-Varying cutting plane
// ============================================================================

std::array<double, 3> NMPCNode::predict_obstacle(int k) const {
    const double t = static_cast<double>(k) * Ts;
    return {
        obs_pos_[0] + t * obs_vel_[0],
        obs_pos_[1] + t * obs_vel_[1],
        obs_pos_[2] + t * obs_vel_[2]
    };
}

bool NMPCNode::compute_tv_plane(int k, const std::array<double,3>& p_ref_k,
                                SfcPlane& out) {
    const double eps = 1e-3;
    const std::array<double,3> p_obs_k = predict_obstacle(k);

    // Unit normal u points obstacle -> reference (the note's "n_k").
    std::array<double,3> dir = {
        p_ref_k[0] - p_obs_k[0],
        p_ref_k[1] - p_obs_k[1],
        p_ref_k[2] - p_obs_k[2]
    };
    double norm = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);

    if (norm < eps) {
        // Degenerate: reference ~ obstacle. Fall back to current ego->obstacle.
        dir = { state_[0] - p_obs_k[0],
                state_[1] - p_obs_k[1],
                state_[2] - p_obs_k[2] };
        norm = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
        if (norm < eps) return false;  // no reliable direction -> skip plane
    }
    std::array<double,3> u = { dir[0]/norm, dir[1]/norm, dir[2]/norm };

    // Optional temporal EMA on the normal (TV_NORMAL_EMA == 0 -> disabled).
    if (TV_NORMAL_EMA > 0.0 && k < static_cast<int>(tv_normal_prev_.size())) {
        const auto& up = tv_normal_prev_[k];
        std::array<double,3> bl = {
            TV_NORMAL_EMA*up[0] + (1.0-TV_NORMAL_EMA)*u[0],
            TV_NORMAL_EMA*up[1] + (1.0-TV_NORMAL_EMA)*u[1],
            TV_NORMAL_EMA*up[2] + (1.0-TV_NORMAL_EMA)*u[2]
        };
        double bn = std::sqrt(bl[0]*bl[0] + bl[1]*bl[1] + bl[2]*bl[2]);
        if (bn > eps) u = { bl[0]/bn, bl[1]/bn, bl[2]/bn };
    }
    if (k < static_cast<int>(tv_normal_prev_.size())) tv_normal_prev_[k] = u;

    // Horizon-dependent safety distance (matches the R_S_MAX ramp).
    const double r_s_k = R_S_MAX * static_cast<double>(k) / static_cast<double>(N);
    const double d     = r_ego_ + r_obs_ + r_s_k;

    // SFC-convention plane  n_sfc . x <= b_sfc  with n_sfc = -u (CORRECTED SIGN
    // vs. the report's 3.1, which as written would push the ego toward the
    // obstacle). Enforces  u . (x - p_obs) >= d, i.e. the ego stays on the
    // reference side at least d away from the obstacle along u.
    out.n = { -u[0], -u[1], -u[2] };
    out.b = -(u[0]*p_obs_k[0] + u[1]*p_obs_k[1] + u[2]*p_obs_k[2]) - d;
    return true;
}

void NMPCNode::fill_tv_planes(std::vector<double>& params) {
    tv_vis_valid_ = false;             // reset RViz cache for this cycle
    if (!obs_received_)       return;  // no dynamic obstacle -> no TV plane
    if (path_points_.empty()) return;  // need references to orient the plane

    for (int j = 0; j < N; ++j) {
        size_t ref_i = path_idx_ + j;
        if (ref_i >= path_points_.size())
            ref_i = path_points_.size() - 1;
        const std::array<double,3> p_ref = {
            path_points_[ref_i][0],
            path_points_[ref_i][1],
            path_points_[ref_i][2]
        };

        SfcPlane pl;
        if (!compute_tv_plane(j, p_ref, pl)) continue;  // leave slot inactive

        // Cache the current (k=0) plane for visualization. n_sfc = -u, and at
        // k=0 the safety distance is d = r_ego + r_obs (r_s_0 = 0), so the
        // plane boundary is tangent to the (r_ego+r_obs) ball around the
        // obstacle. center = p_obs(0) + d * u lies exactly on that boundary.
        if (j == 0) {
            const std::array<double,3> u = { -pl.n[0], -pl.n[1], -pl.n[2] };
            const double d  = r_ego_ + r_obs_;
            const auto   p0 = predict_obstacle(0);
            tv_vis_normal_ = u;
            tv_vis_center_ = { p0[0] + d*u[0], p0[1] + d*u[1], p0[2] + d*u[2] };
            tv_vis_valid_  = true;
        }

        const int off = SFC_START + (j * M_MAX + TV_SLOT) * 4;
        params[off + 0] = pl.n[0];
        params[off + 1] = pl.n[1];
        params[off + 2] = pl.n[2];
        params[off + 3] = pl.b;
    }
}

// ============================================================================
//  Constraint diagnostics
// ============================================================================

void NMPCNode::publish_constraint_diagnostics(
    const std::vector<double>& z_star,
    const std::vector<int>& assignment,
    bool converged)
{
    // ---------- Solver status ----------
    {
        std_msgs::msg::Float64 m;
        m.data = converged ? 1.0 : 0.0;
        pub_solver_converged_->publish(m);
    }

    // ---------- CURRENT obstacle violation/distance ----------
    // Collision threshold is the center-to-center distance r_ego + r_obs.
    //   obs_violation = (r_ego + r_obs) - ||p - p_obs||   (>0 => collision)
    //   obs_distance  = ||p - p_obs||
    {
        const double dx = state_[0] - obs_pos_[0];
        const double dy = state_[1] - obs_pos_[1];
        const double dz = state_[2] - obs_pos_[2];
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        std_msgs::msg::Float64 m;
        m.data = dist; pub_obs_dist_->publish(m);
        m.data = (r_ego_ + r_obs_) - dist; pub_obs_viol_->publish(m);
    }

    // ---------- CURRENT SFC violation (RILS static planes only) ----------
    // For the polytope containing the drone (or closest), compute
    //   margin_m = b_m - n_m . p   (positive = inside)
    //   violation = max_m (-margin_m) = max_m (n_m.p - b_m)
    {
        const std::array<double,3> p_now = {state_[0], state_[1], state_[2]};
        std_msgs::msg::Float64MultiArray margins;
        double worst = -std::numeric_limits<double>::infinity();

        if (!sfcs_.empty()) {
            int idx = (last_poly_idx_ >= 0 &&
                       last_poly_idx_ < static_cast<int>(sfcs_.size()))
                          ? last_poly_idx_ : 0;
            const auto& planes = sfcs_[idx].planes;
            margins.data.reserve(planes.size());
            for (const auto& pl : planes) {
                double margin = pl.b - (pl.n[0]*p_now[0]
                                       + pl.n[1]*p_now[1]
                                       + pl.n[2]*p_now[2]);
                margins.data.push_back(margin);
                if (-margin > worst) worst = -margin;
            }
        }
        if (worst == -std::numeric_limits<double>::infinity()) worst = 0.0;

        pub_sfc_margin_->publish(margins);
        std_msgs::msg::Float64 m;
        m.data = worst;
        pub_sfc_viol_->publish(m);
    }

    // ---------- PREDICTED violations over horizon ----------
    // Roll out the optimal input sequence and evaluate constraints at each step.
    std_msgs::msg::Float64MultiArray pred_obs;
    std_msgs::msg::Float64MultiArray pred_sfc;
    pred_obs.data.reserve(N + 1);
    pred_sfc.data.reserve(N + 1);

    std::array<double, NX> x_k = state_;
    for (int j = 0; j <= N; ++j) {
        // Obstacle violation at step j with safety ramp r_s_j and ego radius.
        const double r_s_j = R_S_MAX * static_cast<double>(j) / static_cast<double>(N);
        const double obs_x = obs_pos_[0] + j * Ts * obs_vel_[0];
        const double obs_y = obs_pos_[1] + j * Ts * obs_vel_[1];
        const double obs_z = obs_pos_[2] + j * Ts * obs_vel_[2];
        const double dx = x_k[0] - obs_x;
        const double dy = x_k[1] - obs_y;
        const double dz = x_k[2] - obs_z;
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        // Violation: positive means inside the (r_ego + r_obs + r_s_j) ball
        pred_obs.data.push_back((r_ego_ + r_obs_ + r_s_j) - dist);

        // SFC violation at step j (use this step's assigned polytope)
        double worst_sfc = 0.0;
        if (!sfcs_.empty() && !assignment.empty()) {
            int j_use = std::min(j, N - 1);  // step N reuses last
            int p_idx = (j_use < static_cast<int>(assignment.size()))
                            ? assignment[j_use] : -1;
            if (p_idx >= 0 && p_idx < static_cast<int>(sfcs_.size())) {
                const auto& planes = sfcs_[p_idx].planes;
                double worst = -std::numeric_limits<double>::infinity();
                for (const auto& pl : planes) {
                    double v = pl.n[0]*x_k[0] + pl.n[1]*x_k[1]
                             + pl.n[2]*x_k[2] - pl.b;
                    if (v > worst) worst = v;
                }
                if (worst != -std::numeric_limits<double>::infinity())
                    worst_sfc = worst;
            }
        }
        pred_sfc.data.push_back(worst_sfc);

        // Roll out one more step (only up to N-1, since z_star only has N inputs)
        if (j < N) {
            int idx_u = j * NU;
            if (idx_u + NU <= static_cast<int>(z_star.size())) {
                std::array<double, NU> uj = {
                    z_star[idx_u], z_star[idx_u+1], z_star[idx_u+2]
                };
                x_k = dynamics_step(x_k, uj);
            } else {
                break;
            }
        }
    }

    pub_pred_obs_viol_->publish(pred_obs);
    pub_pred_sfc_viol_->publish(pred_sfc);
}

// ============================================================================
//  Main control loop (20 Hz)
// ============================================================================

void NMPCNode::control_loop() {
    if (!state_received_) return;

    publish_offboard_mode();
    offboard_counter_++;
    if (offboard_counter_ == OFFBOARD_SETPOINT_COUNT && !offboard_mode_set_) {
        set_offboard_mode();
        offboard_mode_set_ = true;
    }

    update_reference();

    // ----- Build parameter vector -----
    std::vector<double> params(N_PARAMS, 0.0);

    for (int i = 0; i < NX; ++i) params[i]     = state_[i];
    for (int i = 0; i < NU; ++i) params[8 + i] = u_prev_[i];

    const int ref_start = 11;
    for (int j = 0; j < N; ++j) {
        size_t idx = path_idx_ + j;
        if (idx >= path_points_.size())
            idx = path_points_.empty() ? 0 : path_points_.size() - 1;
        if (!path_points_.empty()) {
            params[ref_start + j*3 + 0] = path_points_[idx][0];
            params[ref_start + j*3 + 1] = path_points_[idx][1];
            params[ref_start + j*3 + 2] = path_points_[idx][2];
        } else {
            params[ref_start + j*3 + 0] = state_[0];
            params[ref_start + j*3 + 1] = state_[1];
            params[ref_start + j*3 + 2] = state_[2];
        }
    }

    // Obstacle params [101:108] are now used only by diagnostics + TV-plane
    // construction (the solver no longer reads them as a constraint), but we
    // still pack them to keep the parameter layout identical.
    params[101] = obs_pos_[0];
    params[102] = obs_pos_[1];
    params[103] = obs_pos_[2];
    params[104] = obs_vel_[0];
    params[105] = obs_vel_[1];
    params[106] = obs_vel_[2];
    params[107] = r_obs_;

    // ----- SFC packing: RILS static planes + dynamic TV cutting plane -----
    auto poly_assignment = assign_polytopes_to_horizon();
    fill_sfc_params(params, poly_assignment);  // slots 0 .. M_MAX-2
    fill_tv_planes(params);                     // slot   M_MAX-1 (TV_SLOT)
    publish_tv_plane_marker();                  // RViz: current (k=0) plane

    if (!obs_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Obstacle odom not yet received; no TV cutting plane added.");
    }
    if (!sfc_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "SFC not yet received; flying without static corridor constraints.");
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "obs=[%.2f,%.2f,%.2f] | UAV=[%.1f,%.1f,%.1f] idx=%zu/%zu poly=%d",
        obs_pos_[0], obs_pos_[1], obs_pos_[2],
        state_[0], state_[1], state_[2],
        path_idx_, path_points_.size(),
        last_poly_idx_);

    // ----- Solve -----
    OpEnResult result;
    try {
        result = solver_->solve(params);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Solver call failed: %s", e.what());
        publish_fallback_setpoint();
        return;
    }

    const bool converged =
        result.ok && (result.exit_status.find("Converged") != std::string::npos);

    if (!result.ok || result.solution.size() < static_cast<size_t>(NU)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Solver hard error (%s); using fallback setpoint.",
            result.exit_status.c_str());
        // Even on hard fail, emit current-state diagnostics so PlotJuggler
        // doesn't get gaps in the trace.
        publish_constraint_diagnostics({}, poly_assignment, false);
        publish_fallback_setpoint();
        return;
    }
    if (!converged) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Solver did not converge (%s) — tight corridor/cutting-plane "
            "intersection; using best-iterate solution.",
            result.exit_status.c_str());
    }

    const auto& z_star = result.solution;

    if (converged) {
        last_solve_time_s_ = result.solve_time_ms * 1e-3;
    }

    double T_opt         = z_star[0];
    double phi_ref_opt   = z_star[1];
    double theta_ref_opt = z_star[2];

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "SOLVE: T=%.3f phi=%.6f theta=%.6f | %s | %.2fms",
        T_opt, phi_ref_opt, theta_ref_opt,
        result.exit_status.c_str(), result.solve_time_ms);

    // ----- Telemetry -----
    {
        std_msgs::msg::Float64 m;
        m.data = result.solve_time_ms; pub_solver_time_->publish(m);
    }
    {
        std_msgs::msg::Float64 m;
        m.data = result.cost; pub_cost_function_->publish(m);
    }
    {
        std_msgs::msg::Float64MultiArray m;
        m.data = {T_opt, phi_ref_opt, theta_ref_opt};
        pub_control_input_->publish(m);
    }
    {
        std_msgs::msg::Float64MultiArray m;
        if (z_star.size() >= static_cast<size_t>(2 * NU)) {
            std::array<double, NX> x0 = state_;
            std::array<double, NU> u0 = {z_star[0],    z_star[1],    z_star[2]};
            std::array<double, NU> u1 = {z_star[NU],   z_star[NU+1], z_star[NU+2]};
            std::array<double, NX> x1 = dynamics_step(x0, u0);
            std::array<double, NX> x2 = dynamics_step(x1, u1);

            const double ax0 = (x1[3] - x0[3]) / Ts;
            const double ay0 = (x1[4] - x0[4]) / Ts;
            const double az0 = (x1[5] - x0[5]) / Ts;
            const double ax1 = (x2[3] - x1[3]) / Ts;
            const double ay1 = (x2[4] - x1[4]) / Ts;
            const double az1 = (x2[5] - x1[5]) / Ts;

            m.data = {
                (ax1 - ax0) / Ts,
                (ay1 - ay0) / Ts,
                (az1 - az0) / Ts
            };
        } else {
            m.data = {0.0, 0.0, 0.0};
        }
        pub_jerk_->publish(m);
    }

    // ----- Constraint diagnostics for PlotJuggler -----
    publish_constraint_diagnostics(z_star, poly_assignment, converged);

    u_prev_ = {T_opt, phi_ref_opt, theta_ref_opt};

    double phi_ref_b   = -phi_ref_opt;
    double theta_ref_b = -theta_ref_opt;
    double yaw_rate    = -K_psi_ * yaw_;

    double T_clamped   = std::clamp(T_opt, T_min_, T_max_);
    double thrust_norm = std::sqrt(T_clamped / T_max_);

    {
        px4_msgs::msg::VehicleThrustSetpoint msg;
        msg.timestamp = px4_timestamp();
        msg.xyz[0] = 0.0f; msg.xyz[1] = 0.0f;
        msg.xyz[2] = static_cast<float>(-thrust_norm);
        pub_thrust_->publish(msg);
    }
    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp = px4_timestamp();
        Quaternion q  = euler_to_quaternion(phi_ref_b, theta_ref_b, 0.0);
        msg.q_d[0] = static_cast<float>(q.w);
        msg.q_d[1] = static_cast<float>(q.x);
        msg.q_d[2] = static_cast<float>(q.y);
        msg.q_d[3] = static_cast<float>(q.z);
        msg.thrust_body[0] = 0.0f; msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = static_cast<float>(-thrust_norm);
        msg.yaw_sp_move_rate = static_cast<float>(yaw_rate);
        pub_att_->publish(msg);
    }

    publish_predicted_path(z_star);
    publish_obstacle_marker();
}

// ============================================================================
//  Fallback / visualisation
// ============================================================================

void NMPCNode::publish_fallback_setpoint() {
    double T_clamped   = std::clamp(u_prev_[0], T_min_, T_max_);
    double thrust_norm = std::sqrt(T_clamped / T_max_);
    double phi_b, theta_b;
    global_to_body_angles(u_prev_[1], u_prev_[2], phi_b, theta_b);

    px4_msgs::msg::VehicleAttitudeSetpoint msg;
    msg.timestamp = px4_timestamp();
    Quaternion q  = euler_to_quaternion(phi_b, theta_b, 0.0);
    msg.q_d[0] = static_cast<float>(q.w); msg.q_d[1] = static_cast<float>(q.x);
    msg.q_d[2] = static_cast<float>(q.y); msg.q_d[3] = static_cast<float>(q.z);
    msg.thrust_body[2]   = static_cast<float>(-thrust_norm);
    msg.yaw_sp_move_rate = static_cast<float>(-K_psi_ * yaw_);
    pub_att_->publish(msg);
}

void NMPCNode::publish_predicted_path(const std::vector<double>& z_star) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = this->now();
    path_msg.header.frame_id = "map";

    std::array<double, NX> x_k = state_;
    for (int j = 0; j < N; ++j) {
        int idx = j * NU;
        if (idx + NU > static_cast<int>(z_star.size())) break;
        std::array<double, NU> uj = {z_star[idx], z_star[idx+1], z_star[idx+2]};
        x_k = dynamics_step(x_k, uj);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = x_k[0];
        ps.pose.position.y = x_k[1];
        ps.pose.position.z = x_k[2];
        path_msg.poses.push_back(ps);
    }
    pub_pred_path_->publish(path_msg);
}

void NMPCNode::publish_obstacle_marker() {
    visualization_msgs::msg::Marker m;
    m.header.stamp    = this->now();
    m.header.frame_id = "map";
    m.ns = "obstacle"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = obs_pos_[0];
    m.pose.position.y = obs_pos_[1];
    m.pose.position.z = obs_pos_[2];
    m.pose.orientation.w = 1.0;
    double d = 2.0 * r_obs_;
    m.scale.x = d; m.scale.y = d; m.scale.z = d;
    m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.5f;
    pub_obs_marker_->publish(m);
}

void NMPCNode::publish_tv_plane_marker() {
    visualization_msgs::msg::Marker m;
    m.header.stamp    = this->now();
    m.header.frame_id = "map";
    m.ns = "tv_plane";
    m.id = 0;

    if (!tv_vis_valid_) {
        // No active TV plane this cycle -> remove any stale marker.
        m.action = visualization_msgs::msg::Marker::DELETE;
        pub_tv_plane_marker_->publish(m);
        return;
    }

    m.type   = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;          // vertices are absolute map coords
    m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 1.0;
    m.color.r = 0.0f; m.color.g = 0.6f; m.color.b = 1.0f; m.color.a = 0.35f;

    // Orthonormal in-plane basis (t1, t2) perpendicular to the plane normal u.
    const std::array<double,3> u = tv_vis_normal_;
    std::array<double,3> ref = (std::fabs(u[2]) < 0.9)
        ? std::array<double,3>{0.0, 0.0, 1.0}
        : std::array<double,3>{1.0, 0.0, 0.0};
    std::array<double,3> t1 = {            // t1 = u x ref
        u[1]*ref[2] - u[2]*ref[1],
        u[2]*ref[0] - u[0]*ref[2],
        u[0]*ref[1] - u[1]*ref[0]
    };
    double n1 = std::sqrt(t1[0]*t1[0] + t1[1]*t1[1] + t1[2]*t1[2]);
    if (n1 < 1e-9) { t1 = {1.0, 0.0, 0.0}; n1 = 1.0; }
    t1 = {t1[0]/n1, t1[1]/n1, t1[2]/n1};
    std::array<double,3> t2 = {            // t2 = u x t1 (already unit)
        u[1]*t1[2] - u[2]*t1[1],
        u[2]*t1[0] - u[0]*t1[2],
        u[0]*t1[1] - u[1]*t1[0]
    };

    const double h = 2.0;                  // half-extent of rendered patch [m]
    const auto&  c = tv_vis_center_;
    auto corner = [&](double a, double b) {
        geometry_msgs::msg::Point pt;
        pt.x = c[0] + a*h*t1[0] + b*h*t2[0];
        pt.y = c[1] + a*h*t1[1] + b*h*t2[1];
        pt.z = c[2] + a*h*t1[2] + b*h*t2[2];
        return pt;
    };
    const auto p00 = corner(-1, -1);
    const auto p10 = corner( 1, -1);
    const auto p11 = corner( 1,  1);
    const auto p01 = corner(-1,  1);

    // Two triangles, both windings so the patch is visible from either side.
    m.points = { p00, p10, p11,  p00, p11, p01,
                 p00, p11, p10,  p00, p01, p11 };
    pub_tv_plane_marker_->publish(m);
}

}  // namespace nmpc_uav