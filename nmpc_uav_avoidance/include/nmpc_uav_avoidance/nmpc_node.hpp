/**
 * @file nmpc_node.hpp
 * @brief NMPC UAV — RILS static SFC + dynamic-obstacle TV linear cutting plane.
 *
 * The static RILS SFC half-spaces are enforced as HARD inequality constraints
 * via Augmented Lagrangian (ALM) inside OpEn. The dynamic obstacle is NO longer
 * a nonlinear quadratic distance constraint: it is converted (per horizon step)
 * into a single LINEAR cutting plane (RSFC-style separating hyperplane) and
 * packed into SFC slot m = M_MAX-1. Input-rate limits remain soft penalties.
 *
 * Constraint-monitoring topics (for PlotJuggler):
 *   /nmpc/obs_violation       (Float64)            >0 if drone inside (r_ego+r_obs)
 *   /nmpc/obs_distance        (Float64)            ||drone - obs||
 *   /nmpc/sfc_violation       (Float64)            >0 if drone outside RILS SFC
 *   /nmpc/sfc_margin          (Float64MultiArray)  per-plane signed margin
 *                                                  (positive = inside)
 *   /nmpc/pred_obs_violation  (Float64MultiArray)  predicted, k=0..N
 *   /nmpc/pred_sfc_violation  (Float64MultiArray)  predicted, k=0..N
 *   /nmpc/solver_converged    (Float64)            1.0 if exit_status=Converged
 *
 * Parameter layout (1548 total) — UNCHANGED from the baseline build:
 *   [0:8]      x0
 *   [8:11]     u_prev
 *   [11:101]   p_ref × 30 (3*30 = 90)
 *   [101:104]  p_obs        (now used only for TV-plane build + diagnostics)
 *   [104:107]  v_obs        (now used only for TV-plane build + diagnostics)
 *   [107]      r_obs        (now used only for TV-plane build + diagnostics)
 *   [108:1548] SFC planes (4 * M_MAX * N = 4 * 12 * 30 = 1440)
 *              slots 0..M_MAX-2 : RILS static planes
 *              slot  M_MAX-1    : dynamic TV cutting plane
 */

#ifndef NMPC_UAV_AVOIDANCE__NMPC_NODE_HPP_
#define NMPC_UAV_AVOIDANCE__NMPC_NODE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

#include "nmpc_uav_avoidance/open_solver.hpp"

namespace nmpc_uav {

// ---------------------------------------------------------------------------
//  Constants — must match build_solver.py
// ---------------------------------------------------------------------------
constexpr int    N        = 30;
constexpr int    NX       = 8;
constexpr int    NU       = 3;
constexpr double Ts       = 0.05;

constexpr double GRAVITY  = 9.81;
constexpr double TAU_PHI  = 0.5;
constexpr double TAU_THETA = 0.5;
constexpr double K_PHI    = 1.0;
constexpr double K_THETA  = 1.0;
constexpr double AX = 0.1, AY = 0.1, AZ = 0.2;

// SFC plane budget (per horizon step)
constexpr int M_MAX = 12;

// The last SFC slot is reserved for the dynamic Time-Varying cutting plane.
// RILS static planes therefore use at most M_MAX-1 (= 11) slots when a
// dynamic obstacle is present.
constexpr int TV_SLOT = M_MAX - 1;

// Temporal EMA smoothing factor for the TV plane normal (anti-chatter).
//   n_smoothed = normalize(alpha * n_prev + (1 - alpha) * n_new)
// 0.0 = disabled (default, no lag). Increase ONLY if you observe normal
// chattering; note that large alpha lags a fast-moving obstacle and can
// REDUCE the avoidance margin, so keep it small (e.g. <= 0.3).
constexpr double TV_NORMAL_EMA = 0.0;

// Parameter layout
constexpr int SFC_START = 108;
constexpr int SFC_BLOCK = 4 * M_MAX * N;          // 1440
constexpr int N_PARAMS  = SFC_START + SFC_BLOCK;  // 1548

constexpr int OFFBOARD_SETPOINT_COUNT = 10;

// Must match R_S_MAX in build_solver.py (ramp 0 -> R_S_MAX over horizon)
constexpr double R_S_MAX = 0.3;

// ---------------------------------------------------------------------------
//  POD types
// ---------------------------------------------------------------------------
struct EulerAngles { double roll, pitch, yaw; };
struct Quaternion  { double x, y, z, w; };

struct SfcPlane {
  std::array<double, 3> n;
  double b;
};

struct SfcPolytope {
  int segment_index{0};
  std::array<double, 3> w0{0,0,0};
  std::array<double, 3> w1{0,0,0};
  std::vector<SfcPlane> planes;
};

// ---------------------------------------------------------------------------
//  Free helpers
// ---------------------------------------------------------------------------
EulerAngles quaternion_to_euler(double qx, double qy, double qz, double qw);
Quaternion  euler_to_quaternion(double roll, double pitch, double yaw);

std::array<double, NX> dynamics_step(const std::array<double, NX>& x,
                                     const std::array<double, NU>& u);

// ===========================================================================
//  Node
// ===========================================================================
class NMPCNode : public rclcpp::Node
{
public:
  NMPCNode();
  ~NMPCNode();

private:
  // ---- Callbacks --------------------------------------------------------
  void odom_callback     (const nav_msgs::msg::Odometry::SharedPtr msg);
  void obs_odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void path_callback     (const nav_msgs::msg::Path::SharedPtr msg);
  void sfc_callback      (const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // ---- PX4 helpers ------------------------------------------------------
  void publish_vehicle_command(uint16_t command,
                               float p1=0.f, float p2=0.f, float p3=0.f,
                               float p4=0.f, float p5=0.f, float p6=0.f, float p7=0.f);
  void set_offboard_mode();
  void arm();
  void disarm();
  void publish_offboard_mode();
  static uint64_t px4_timestamp();

  // ---- Reference / control ---------------------------------------------
  void update_reference();
  void global_to_body_angles(double phi_g, double theta_g,
                             double& phi_b, double& theta_b) const;
  void control_loop();
  void publish_fallback_setpoint();
  void publish_predicted_path(const std::vector<double>& z_star);
  void publish_obstacle_marker();
  void publish_tv_plane_marker();   // RViz marker for the k=0 TV cutting plane

  // ---- Constraint monitoring (for PlotJuggler) -------------------------
  void publish_constraint_diagnostics(const std::vector<double>& z_star,
                                      const std::vector<int>& assignment,
                                      bool converged);

  // ---- SFC helpers ------------------------------------------------------
  bool point_inside_polytope(const std::array<double,3>& p,
                             const std::vector<SfcPlane>& planes,
                             double tol = 1e-3) const;
  int  find_closest_polytope(const std::array<double,3>& p) const;
  int  find_polytope_for_point(const std::array<double,3>& p,
                               int idx_start) const;

  std::vector<int> assign_polytopes_to_horizon();
  void fill_sfc_params(std::vector<double>& params,
                       const std::vector<int>& assignment) const;

  // ---- Dynamic-obstacle Time-Varying cutting plane ---------------------
  std::array<double, 3> predict_obstacle(int k) const;
  // Builds the per-step TV plane in the SFC convention (n . x <= b) with the
  // CORRECTED sign so the ego is kept on the reference side of the obstacle.
  // Returns false if no reliable separating direction exists (slot left
  // inactive). Non-const: may update the EMA normal history.
  bool compute_tv_plane(int k, const std::array<double,3>& p_ref_k,
                        SfcPlane& out);
  // Writes the per-step TV planes into SFC slot TV_SLOT of `params`.
  void fill_tv_planes(std::vector<double>& params);

  // ---- Members ----------------------------------------------------------
  std::unique_ptr<OpEnSolver> solver_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     sub_obs_odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr         sub_path_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_sfc_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr   pub_thrust_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr pub_att_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr     pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          pub_command_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                    pub_pred_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr        pub_obs_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr        pub_tv_plane_marker_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 pub_solver_time_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 pub_cost_function_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       pub_control_input_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       pub_jerk_;

  // ---- Constraint diagnostics publishers (for PlotJuggler) -------------
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_obs_viol_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_obs_dist_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_sfc_viol_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_sfc_margin_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pred_obs_viol_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pred_sfc_viol_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           pub_solver_converged_;

  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::array<double, NX> state_  {0,0,0, 0,0,0, 0,0};
  std::array<double, NU> u_prev_ {GRAVITY, 0.0, 0.0};
  double yaw_ = 0.0;
  bool   state_received_ = false;

  // Dynamic obstacle
  std::array<double, 3> obs_pos_ {3.0, 0.0, 1.0};
  std::array<double, 3> obs_vel_ {0.0, 0.0, 0.0};
  double r_obs_ = 1.0;
  double r_ego_ = 0.25;   // ego radius used in TV-plane safety distance
  bool   obs_received_ = false;

  // TV cutting plane: previous-iteration normal per horizon step (EMA state)
  std::vector<std::array<double, 3>> tv_normal_prev_;

  // Cached k=0 TV cutting plane for RViz (filled each cycle in fill_tv_planes)
  bool                  tv_vis_valid_  = false;
  std::array<double, 3> tv_vis_normal_ {0.0, 0.0, 0.0};  // plane unit normal (u)
  std::array<double, 3> tv_vis_center_ {0.0, 0.0, 0.0};  // a point on the plane

  // Path / reference
  std::vector<std::array<double, 3>> path_points_;
  size_t path_idx_         = 0;
  double last_solve_time_s_ = Ts;

  // SFC
  std::vector<SfcPolytope> sfcs_;
  int  last_poly_idx_  = 0;
  bool sfc_received_   = false;

  // PX4 mode
  bool   offboard_mode_set_ = false;
  int    offboard_counter_  = 0;
  uint8_t nav_state_    = 0;
  uint8_t arming_state_ = 0;

  // Tunables
  double T_max_ = 13.5, T_min_ = 5.0, K_psi_ = 1.0;
};

}  // namespace nmpc_uav

#endif  // NMPC_UAV_AVOIDANCE__NMPC_NODE_HPP_