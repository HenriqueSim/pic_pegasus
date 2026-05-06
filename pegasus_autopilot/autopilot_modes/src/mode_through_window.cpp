#include "autopilot_modes/mode_through_window.hpp"
#include <pegasus_utils/rotations.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace autopilot {

// ═══════════════════════════════════════════════════════════════════════════
//  Destructor
// ═══════════════════════════════════════════════════════════════════════════

ThroughWindowMode::~ThroughWindowMode() {
    window_service_.reset();
}

// ═══════════════════════════════════════════════════════════════════════════
//  initialize
// ═══════════════════════════════════════════════════════════════════════════

void ThroughWindowMode::initialize() {

    // ── Service ────────────────────────────────────────────────────────────
    node_->declare_parameter<std::string>(
        "autopilot.ThroughWindowMode.set_window_service", "set_window");
    window_service_ = node_->create_service<pegasus_msgs::srv::ThroughWindow>(
        node_->get_parameter("autopilot.ThroughWindowMode.set_window_service").as_string(),
        std::bind(&ThroughWindowMode::through_window_callback,
                  this, std::placeholders::_1, std::placeholders::_2));

    // ── Trajectory structure ───────────────────────────────────────────────
    node_->declare_parameter<int>   ("autopilot.polynomial_trajectory.num_laps",              1);
    node_->declare_parameter<int>   ("autopilot.polynomial_trajectory.num_entry_segments",    1);
    node_->declare_parameter<int>   ("autopilot.polynomial_trajectory.num_circular_segments", 1);
    node_->declare_parameter<int>   ("autopilot.polynomial_trajectory.num_segments_total",    2);
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.entry_duration",        3.0);
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.circular_lap_duration", 5.0);

    num_laps_              = node_->get_parameter("autopilot.polynomial_trajectory.num_laps").as_int();
    num_entry_segments_    = node_->get_parameter("autopilot.polynomial_trajectory.num_entry_segments").as_int();
    num_circular_segments_ = node_->get_parameter("autopilot.polynomial_trajectory.num_circular_segments").as_int();
    num_segments_total_    = node_->get_parameter("autopilot.polynomial_trajectory.num_segments_total").as_int();
    entry_duration_        = node_->get_parameter("autopilot.polynomial_trajectory.entry_duration").as_double();
    circular_lap_duration_ = node_->get_parameter("autopilot.polynomial_trajectory.circular_lap_duration").as_double();
    T_final_               = entry_duration_ + num_laps_ * circular_lap_duration_;

    RCLCPP_INFO(node_->get_logger(),
        "Trajectory: %d entry segs (%.2fs) + %d circular segs x %d laps (%.2fs/lap) = %.2fs total",
        num_entry_segments_, entry_duration_,
        num_circular_segments_, num_laps_, circular_lap_duration_, T_final_);

    // ── Load coefficients ──────────────────────────────────────────────────
    auto load_vec = [&](const std::string& name) -> Eigen::VectorXd {
        std::vector<double> v;
        node_->declare_parameter(name, v);
        v = node_->get_parameter(name).as_double_array();
        return Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
    };

    x_segments_.clear(); y_segments_.clear(); z_segments_.clear();
    segment_durations_.clear();
    entry_start_times_.clear();
    circ_start_times_.clear();

    double t_entry = 0.0;
    double t_circ  = 0.0;

    for (int i = 0; i < num_segments_total_; ++i) {
        const std::string p = "autopilot.polynomial_trajectory.segments.seg" + std::to_string(i);
        node_->declare_parameter<double>(p + ".duration", 1.0);
        double dur = node_->get_parameter(p + ".duration").as_double();

        segment_durations_.push_back(dur);
        x_segments_.push_back(load_vec(p + ".x"));
        y_segments_.push_back(load_vec(p + ".y"));
        z_segments_.push_back(load_vec(p + ".z"));

        if (i < num_entry_segments_) {
            entry_start_times_.push_back(t_entry);
            t_entry += dur;
        } else {
            circ_start_times_.push_back(t_circ);
            t_circ += dur;
        }

        RCLCPP_INFO(node_->get_logger(),
            "  seg%d [%s]: duration=%.3fs  coeffs=%ld",
            i, (i < num_entry_segments_ ? "entry" : "circ"),
            dur, x_segments_.back().size());
    }

    // Active coefficients start at segment 0
    x_coeffs_ = x_segments_[0];
    y_coeffs_ = y_segments_[0];
    z_coeffs_ = z_segments_[0];

    // ── Controller gains ───────────────────────────────────────────────────
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.kp",         std::vector<double>());
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.kd",         std::vector<double>());
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.ki",         std::vector<double>());
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.kr",         std::vector<double>());
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>(
        "autopilot.YawLessController.gains.max_output", std::vector<double>());

    auto kp_v  = node_->get_parameter("autopilot.YawLessController.gains.kp").as_double_array();
    auto kd_v  = node_->get_parameter("autopilot.YawLessController.gains.kd").as_double_array();
    auto ki_v  = node_->get_parameter("autopilot.YawLessController.gains.ki").as_double_array();
    auto kr_v  = node_->get_parameter("autopilot.YawLessController.gains.kr").as_double_array();
    auto min_v = node_->get_parameter("autopilot.YawLessController.gains.min_output").as_double_array();
    auto max_v = node_->get_parameter("autopilot.YawLessController.gains.max_output").as_double_array();

    for (int i = 0; i < 3; ++i) {
        kp_[i] = kp_v[i];  kd_[i] = kd_v[i];  ki_[i] = ki_v[i];
        kr_(i, i) = kr_v[i];
        min_output_[i] = min_v[i];  max_output_[i] = max_v[i];
    }

    // ── Drag (optional override in YAML) ───────────────────────────────────
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.drag.rho_x", 0.0);
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.drag.rho_y", 0.0);
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.drag.rho_z", 0.0);
    rho_x_ = node_->get_parameter("autopilot.polynomial_trajectory.drag.rho_x").as_double();
    rho_y_ = node_->get_parameter("autopilot.polynomial_trajectory.drag.rho_y").as_double();
    rho_z_ = node_->get_parameter("autopilot.polynomial_trajectory.drag.rho_z").as_double();

    // ── Vehicle mass ───────────────────────────────────────────────────────
    mass_ = get_vehicle_constants().mass;

    // ── Publishers ─────────────────────────────────────────────────────────
    // Static reference paths are published once — use transient_local so
    // Foxglove receives them even if it subscribes after initialize().
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    path_pub_         = node_->create_publisher<nav_msgs::msg::Path>("/debug/trajectory",    latched_qos);
    tracked_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/debug/tracked_path",  10);
    marker_pub_       = node_->create_publisher<visualization_msgs::msg::Marker>(
                            "/debug/trajectory_markers", latched_qos);
    drone_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
                            "/debug/drone_marker", 10);

    seg_path_pubs_.clear();
    for (int i = 0; i < num_segments_total_; ++i) {
        const std::string topic = "/debug/trajectory/seg" + std::to_string(i);
        seg_path_pubs_.push_back(
            node_->create_publisher<nav_msgs::msg::Path>(topic, latched_qos));

    }

    traj_pos_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/trajectory_position",     10);
    traj_vel_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/trajectory_velocity",     10);
    traj_acc_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/trajectory_acceleration", 10);
    real_pos_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/real_position", 10);
    real_vel_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/real_velocity",  10);
    att_error_pub_    = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/attitude_error",          10);
    att_ref_pub_      = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/attitude_reference",      10);
    att_rate_ref_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/attitude_rate_reference", 10);
    att_real_pub_     = node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/attitude_real",           10);
    att_rate_real_pub_= node_->create_publisher<geometry_msgs::msg::Vector3>(
                            "/debug/attitude_rate_real",      10);
    thrust_pub_       = node_->create_publisher<std_msgs::msg::Float64>("/debug/thrust", 10);

    // ── Pre-compute and publish static visualisation ───────────────────────
    generate_path_msg();
    generate_segment_path_msgs();
    generate_trajectory_markers(0.1);

    path_pub_->publish(path_msg_);
    for (int i = 0; i < num_segments_total_; ++i)
        seg_path_pubs_[i]->publish(seg_path_msgs_[i]);

    RCLCPP_INFO(node_->get_logger(), "ThroughWindowMode initialised.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  enter / exit
// ═══════════════════════════════════════════════════════════════════════════

bool ThroughWindowMode::enter() {
    if (!window_point_set_) {
        RCLCPP_ERROR(node_->get_logger(),
            "ThroughWindowMode: window not set — cannot enter.");
        return false;
    }
    window_point_set_ = false;
    t_               = 0.0;
    current_segment_ = 0;
    current_lap_     = 0;
    in_entry_phase_  = true;
    x_coeffs_ = x_segments_[0];
    y_coeffs_ = y_segments_[0];
    z_coeffs_ = z_segments_[0];

    // Reset live tracked path
    tracked_path_msg_.poses.clear();
    tracked_path_msg_.header.frame_id = "world";

    // Re-publish static reference paths and markers
    path_pub_->publish(path_msg_);
    marker_pub_->publish(marker_msg_);
    for (int i = 0; i < num_segments_total_; ++i)
        seg_path_pubs_[i]->publish(seg_path_msgs_[i]);

    return true;
}

bool ThroughWindowMode::exit() {
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  update  (called at controller rate)
// ═══════════════════════════════════════════════════════════════════════════

void ThroughWindowMode::update(double dt) {

    t_ += dt;

    // ── All laps finished ──────────────────────────────────────────────────
    if (t_ >= T_final_) {
        t_ = T_final_;
        const State s = get_vehicle_state();
        controller_->set_position(s.position, 0.0, dt);
        signal_mode_finished();
        return;
    }

    double t_local = 0.0;

    if (t_ < entry_duration_) {
        // ── Entry phase ────────────────────────────────────────────────────
        in_entry_phase_ = true;
        current_lap_    = -1;
        current_segment_ = num_entry_segments_ - 1;   // fallback to last entry seg

        for (int i = 0; i < num_entry_segments_; ++i) {
            if (t_ < entry_start_times_[i] + segment_durations_[i]) {
                current_segment_ = i;
                break;
            }
        }
        t_local = t_ - entry_start_times_[current_segment_];

    } else {
        // ── Circular phase ─────────────────────────────────────────────────
        in_entry_phase_ = false;
        const double t_circ = t_ - entry_duration_;
        current_lap_ = static_cast<int>(t_circ / circular_lap_duration_);
        if (current_lap_ >= num_laps_) current_lap_ = num_laps_ - 1;

        const double t_in_lap = t_circ - current_lap_ * circular_lap_duration_;

        // Find which circular segment within this lap
        int circ_idx = num_circular_segments_ - 1;   // fallback to last
        for (int i = 0; i < num_circular_segments_; ++i) {
            if (t_in_lap < circ_start_times_[i] + segment_durations_[num_entry_segments_ + i]) {
                circ_idx = i;
                break;
            }
        }
        current_segment_ = num_entry_segments_ + circ_idx;
        t_local = t_in_lap - circ_start_times_[circ_idx];
    }

    if (t_local < 0.0) t_local = 0.0;

    x_coeffs_ = x_segments_[current_segment_];
    y_coeffs_ = y_segments_[current_segment_];
    z_coeffs_ = z_segments_[current_segment_];

    // ── Evaluate references ────────────────────────────────────────────────
    Eigen::Vector3d pos, vel, acc, jerk;
    eval_trajectory(t_local, pos, vel, acc, jerk);

    // ── Linear drag feedforward ────────────────────────────────────────────
    const Eigen::Vector3d rho(rho_x_, rho_y_, rho_z_);
    const Eigen::Vector3d drag     = rho.cwiseProduct(vel);
    const Eigen::Vector3d drag_dot = rho.cwiseProduct(acc);

    // ── Publish reference debug ────────────────────────────────────────────
    geometry_msgs::msg::Vector3 p_msg, v_msg, a_msg;
    p_msg.x = pos.x();  p_msg.y = pos.y();  p_msg.z = pos.z();
    v_msg.x = vel.x();  v_msg.y = vel.y();  v_msg.z = vel.z();
    a_msg.x = acc.x();  a_msg.y = acc.y();  a_msg.z = acc.z() - G;
    traj_pos_pub_->publish(p_msg);
    traj_vel_pub_->publish(v_msg);
    traj_acc_pub_->publish(a_msg);

    debug_attitude_refs(acc + drag, jerk + drag_dot, get_vehicle_state());

    set_position(pos, vel, acc + drag, jerk + drag_dot, 0.0, 0.0, dt);

    publish_drone_marker();

    // ── Append to live tracked path ────────────────────────────────────────
    const State tracked_state = get_vehicle_state();
    geometry_msgs::msg::PoseStamped tracked_pose;
    tracked_pose.header.frame_id = "world";
    tracked_pose.header.stamp    = node_->now();
    tracked_pose.pose.position.x =  tracked_state.position.x();
    tracked_pose.pose.position.y = -tracked_state.position.y();
    tracked_pose.pose.position.z = -tracked_state.position.z();
    tracked_pose.pose.orientation.w = 1.0;
    tracked_path_msg_.header.stamp = tracked_pose.header.stamp;
    tracked_path_msg_.poses.push_back(tracked_pose);
    tracked_path_pub_->publish(tracked_path_msg_);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Trajectory helpers
// ═══════════════════════════════════════════════════════════════════════════

void ThroughWindowMode::eval_trajectory(double t_local,
                                         Eigen::Vector3d& pos,
                                         Eigen::Vector3d& vel,
                                         Eigen::Vector3d& acc,
                                         Eigen::Vector3d& jerk) const {
    const int n = x_coeffs_.size();
    Eigen::VectorXd tv(n), dv(n), ddv(n), dddv(n);
    for (int i = 0; i < n; ++i) {
        tv(i)   = std::pow(t_local, i);
        dv(i)   = (i == 0) ? 0.0 : i * std::pow(t_local, i - 1);
        ddv(i)  = (i <= 1) ? 0.0 : i * (i - 1) * std::pow(t_local, i - 2);
        dddv(i) = (i <= 2) ? 0.0 : i * (i - 1) * (i - 2) * std::pow(t_local, i - 3);
    }
    pos  = {x_coeffs_.dot(tv),   y_coeffs_.dot(tv),   z_coeffs_.dot(tv)};
    vel  = {x_coeffs_.dot(dv),   y_coeffs_.dot(dv),   z_coeffs_.dot(dv)};
    acc  = {x_coeffs_.dot(ddv),  y_coeffs_.dot(ddv),  z_coeffs_.dot(ddv)};
    jerk = {x_coeffs_.dot(dddv), y_coeffs_.dot(dddv), z_coeffs_.dot(dddv)};
}

// ═══════════════════════════════════════════════════════════════════════════
//  Control
// ═══════════════════════════════════════════════════════════════════════════

double ThroughWindowMode::compute_output(double pos_err, double vel_err,
                                          double ff_acc, unsigned int axis) {
    return kp_[axis] * pos_err + kd_[axis] * vel_err + ff_acc;
}

void ThroughWindowMode::set_position(const Eigen::Vector3d& pos,
                                      const Eigen::Vector3d& vel,
                                      const Eigen::Vector3d& acc,
                                      const Eigen::Vector3d& jerk,
                                      double yaw, double /*yaw_rate*/, double /*dt*/) {

    const State state = get_vehicle_state();

    const Eigen::Vector3d pos_err = pos - state.position;
    const Eigen::Vector3d vel_err = vel - state.velocity;

    Eigen::Vector3d ff;
    ff << acc.x(), acc.y(), acc.z() - G;

    Eigen::Vector3d F_des;
    for (int i = 0; i < 3; ++i)
        F_des[i] = mass_ * compute_output(pos_err[i], vel_err[i], ff[i], i);

    const Eigen::Vector3d Z_b_des = -F_des / F_des.norm();

    const double yaw_rad = Pegasus::Rotations::deg_to_rad(yaw);
    const Eigen::Vector3d Y_C(-std::sin(yaw_rad), std::cos(yaw_rad), 0.0);
    const Eigen::Vector3d X_b_des = (Y_C.cross(Z_b_des)).normalized();
    const Eigen::Vector3d Y_b_des = (Z_b_des.cross(X_b_des)).normalized();

    Eigen::Matrix3d R_des;
    R_des.col(0) = X_b_des;
    R_des.col(1) = Y_b_des;
    R_des.col(2) = Z_b_des;

    const double T = F_des.norm();
    const Eigen::Matrix3d R = state.attitude.toRotationMatrix();
    const Eigen::Vector3d e3(0, 0, 1);
    const Eigen::Matrix3d PI_e3 = Eigen::Matrix3d::Identity() - e3 * e3.transpose();

    Eigen::Vector3d attitude_rate =
        kr_ * e3.cross(R.transpose() * Z_b_des)
        - PI_e3 * (R.transpose() * Z_b_des.cross(jerk)) * (mass_ / T);

    attitude_rate = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_rate[0]),
        Pegasus::Rotations::rad_to_deg(attitude_rate[1]),
        Pegasus::Rotations::rad_to_deg(attitude_rate[2]));

    controller_->set_attitude_rate(attitude_rate, T);

    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = T;
    thrust_pub_->publish(thrust_msg);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Debug / visualisation
// ═══════════════════════════════════════════════════════════════════════════

void ThroughWindowMode::debug_attitude_refs(const Eigen::Vector3d& acc,
                                             const Eigen::Vector3d& jerk,
                                             const State& state) {
    const Eigen::Vector3d b = acc - Eigen::Vector3d(0.0, 0.0, G);
    const Eigen::Vector3d F_des  = mass_ * b;
    const Eigen::Vector3d Z_b_des = -F_des / F_des.norm();
    const double T = F_des.norm();

    const Eigen::Vector3d Y_C(0.0, 1.0, 0.0);
    const Eigen::Vector3d X_b_des = (Y_C.cross(Z_b_des)).normalized();
    const Eigen::Vector3d Y_b_des = (Z_b_des.cross(X_b_des)).normalized();
    Eigen::Matrix3d R_des;
    R_des.col(0) = X_b_des; R_des.col(1) = Y_b_des; R_des.col(2) = Z_b_des;

    const Eigen::Vector3d euler_des = R_des.eulerAngles(2, 1, 0);
    const Eigen::Vector3d e3(0, 0, 1);
    const Eigen::Matrix3d PI_e3 = Eigen::Matrix3d::Identity() - e3 * e3.transpose();
    const Eigen::Vector3d w_des = -PI_e3 * (R_des.transpose() * Z_b_des.cross(jerk)) * (mass_ / T);

    Eigen::Vector3d euler_real = state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
    auto clamp_angle = [](double& a, double lim) {
        if (a >  lim) a -= M_PI;
        if (a < -lim) a += M_PI;
    };
    clamp_angle(euler_real(0), M_PI / 2);
    clamp_angle(euler_real(2), M_PI / 2);
    if (euler_real(1) >  M_PI / 2) euler_real(1) = -euler_real(1) + M_PI;
    if (euler_real(1) < -M_PI / 2) euler_real(1) = -euler_real(1) - M_PI;

    const Eigen::Vector3d w_real  = state.angular_velocity;
    const Eigen::Matrix3d R       = state.attitude.toRotationMatrix();
    const Eigen::Vector3d r3_err  = PI_e3 * e3.cross(R.transpose() * Z_b_des);

    auto fill = [](geometry_msgs::msg::Vector3& m, double x, double y, double z) {
        m.x = x; m.y = y; m.z = z;
    };
    geometry_msgs::msg::Vector3 err_m, ref_m, rref_m, real_m, rreal_m;
    fill(err_m,  r3_err(0),                r3_err(1),                r3_err(2));
    fill(ref_m,  euler_des(2)*RAD2DEG,     euler_des(1)*RAD2DEG,     euler_des(0)*RAD2DEG);
    fill(rref_m, w_des(0)*RAD2DEG,         w_des(1)*RAD2DEG,         w_des(2)*RAD2DEG);
    fill(real_m, euler_real(2)*RAD2DEG,    euler_real(1)*RAD2DEG,    euler_real(0)*RAD2DEG);
    fill(rreal_m,w_real(0)*RAD2DEG,        w_real(1)*RAD2DEG,        w_real(2)*RAD2DEG);

    att_error_pub_   ->publish(err_m);
    att_ref_pub_     ->publish(ref_m);
    att_rate_ref_pub_->publish(rref_m);
    att_real_pub_    ->publish(real_m);
    att_rate_real_pub_->publish(rreal_m);
}

void ThroughWindowMode::generate_segment_path_msgs() {
    seg_path_msgs_.clear();
    seg_path_msgs_.resize(num_segments_total_);
    constexpr int STEPS = 100;
    const rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    for (int s = 0; s < num_segments_total_; ++s) {
        x_coeffs_ = x_segments_[s];
        y_coeffs_ = y_segments_[s];
        z_coeffs_ = z_segments_[s];
        const double dur = segment_durations_[s];

        seg_path_msgs_[s].header.frame_id = "world";
        seg_path_msgs_[s].header.stamp    = now;
        seg_path_msgs_[s].poses.clear();

        for (int i = 0; i <= STEPS; ++i) {
            Eigen::Vector3d pos, vel, acc, jerk;
            eval_trajectory((dur * i) / STEPS, pos, vel, acc, jerk);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id   = "world";
            pose.header.stamp      = now;
            pose.pose.position.x   =  pos.x();
            pose.pose.position.y   = -pos.y();
            pose.pose.position.z   = -pos.z();
            pose.pose.orientation.w = 1.0;
            seg_path_msgs_[s].poses.push_back(pose);
        }
    }
    x_coeffs_ = x_segments_[current_segment_];
    y_coeffs_ = y_segments_[current_segment_];
    z_coeffs_ = z_segments_[current_segment_];
}

void ThroughWindowMode::generate_path_msg() {
    path_msg_.header.frame_id = "world";
    path_msg_.header.stamp    = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    path_msg_.poses.clear();
    constexpr int STEPS = 50;

    for (int s = 0; s < num_segments_total_; ++s) {
        x_coeffs_ = x_segments_[s];
        y_coeffs_ = y_segments_[s];
        z_coeffs_ = z_segments_[s];
        const double dur = segment_durations_[s];

        for (int i = 0; i <= STEPS; ++i) {
            Eigen::Vector3d pos, vel, acc, jerk;
            eval_trajectory((dur * i) / STEPS, pos, vel, acc, jerk);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg_.header;
            pose.pose.position.x   =  pos.x();
            pose.pose.position.y   = -pos.y();
            pose.pose.position.z   = -pos.z();
            pose.pose.orientation.w = 1.0;
            path_msg_.poses.push_back(pose);
        }
    }
    x_coeffs_ = x_segments_[current_segment_];
    y_coeffs_ = y_segments_[current_segment_];
    z_coeffs_ = z_segments_[current_segment_];
}

void ThroughWindowMode::generate_trajectory_markers(double dt_marker) {
    marker_msg_.header.frame_id = "world";
    marker_msg_.header.stamp    = node_->now();
    marker_msg_.ns   = "trajectory";
    marker_msg_.id   = 0;
    marker_msg_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.scale.x = marker_msg_.scale.y = marker_msg_.scale.z = 0.05;

    // Entry segments: grey.  Circular segments: colour cycle.
    auto seg_color = [&](int s) -> std_msgs::msg::ColorRGBA {
        std_msgs::msg::ColorRGBA c;  c.a = 1.0;
        if (s < num_entry_segments_) {
            c.r = 0.5f; c.g = 0.5f; c.b = 0.5f;   // grey
            return c;
        }
        const float h = static_cast<float>(s - num_entry_segments_) /
                        static_cast<float>(std::max(1, num_circular_segments_));
        const int   hi = static_cast<int>(h * 6) % 6;
        const float f  = h * 6.0f - static_cast<int>(h * 6);
        const float q  = 1.0f - f;
        switch (hi) {
            case 0: c.r=1; c.g=f; c.b=0; break;
            case 1: c.r=q; c.g=1; c.b=0; break;
            case 2: c.r=0; c.g=1; c.b=f; break;
            case 3: c.r=0; c.g=q; c.b=1; break;
            case 4: c.r=f; c.g=0; c.b=1; break;
            default:c.r=1; c.g=0; c.b=q; break;
        }
        return c;
    };

    for (int s = 0; s < num_segments_total_; ++s) {
        x_coeffs_ = x_segments_[s];
        y_coeffs_ = y_segments_[s];
        z_coeffs_ = z_segments_[s];
        const double dur   = segment_durations_[s];
        const auto   color = seg_color(s);

        for (double tau = 0.0; tau <= dur; tau += dt_marker) {
            Eigen::Vector3d pos, vel, acc, jerk;
            eval_trajectory(tau, pos, vel, acc, jerk);
            geometry_msgs::msg::Point pt;
            pt.x =  pos.x(); pt.y = -pos.y(); pt.z = -pos.z();
            marker_msg_.points.push_back(pt);
            marker_msg_.colors.push_back(color);
        }
    }
    x_coeffs_ = x_segments_[current_segment_];
    y_coeffs_ = y_segments_[current_segment_];
    z_coeffs_ = z_segments_[current_segment_];
}

void ThroughWindowMode::publish_drone_marker() {
    const State        state = get_vehicle_state();
    const rclcpp::Time now   = node_->now();

    // NED → ENU: x_enu = x_ned, y_enu = -y_ned, z_enu = -z_ned
    const Eigen::Vector3d pos_enu(state.position.x(),
                                  -state.position.y(),
                                  -state.position.z());

    // Rotate attitude from NED body frame to ENU: apply 180° around X
    const Eigen::Quaterniond q_ned_to_enu(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond q_enu = q_ned_to_enu * state.attitude;

    // ── Body cube (pose = drone position + ENU attitude) ──────────────────
    visualization_msgs::msg::Marker body;
    body.header.frame_id = "world";
    body.header.stamp    = now;
    body.ns   = "drone";
    body.id   = 0;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x    = pos_enu.x();
    body.pose.position.y    = pos_enu.y();
    body.pose.position.z    = pos_enu.z();
    body.pose.orientation.x = q_enu.x();
    body.pose.orientation.y = q_enu.y();
    body.pose.orientation.z = q_enu.z();
    body.pose.orientation.w = q_enu.w();
    body.scale.x = 0.3;  body.scale.y = 0.3;  body.scale.z = 0.1;
    body.color.r = 0.0;  body.color.g = 0.7;  body.color.b = 1.0;  body.color.a = 1.0;
    drone_marker_pub_->publish(body);

    // ── Arrow markers ─────────────────────────────────────────────────────
    // For 2-point ARROW markers the direction comes from points[], not pose.
    // The pose must be IDENTITY, otherwise the points get double-transformed.
    // We compute the tip by rotating the body-frame direction into ENU world frame.
    auto publish_arrow = [&](int id,
                              const Eigen::Vector3d& body_dir,
                              float r, float g, float b)
    {
        // Rotate body-frame direction vector into ENU world frame
        const Eigen::Vector3d world_dir = q_enu * body_dir.normalized();
        const Eigen::Vector3d tip       = pos_enu + 0.5 * world_dir;

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp    = now;
        arrow.ns   = "drone";
        arrow.id   = id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        // Identity pose — points are already in world (ENU) frame
        arrow.pose.orientation.w = 1.0;

        arrow.scale.x = 0.05;   // shaft diameter
        arrow.scale.y = 0.10;   // head diameter
        arrow.scale.z = 0.10;   // head length

        arrow.color.r = r;  arrow.color.g = g;
        arrow.color.b = b;  arrow.color.a = 1.0;

        geometry_msgs::msg::Point pt_start, pt_end;
        pt_start.x = pos_enu.x();  pt_start.y = pos_enu.y();  pt_start.z = pos_enu.z();
        pt_end.x   = tip.x();      pt_end.y   = tip.y();      pt_end.z   = tip.z();
        arrow.points.push_back(pt_start);
        arrow.points.push_back(pt_end);

        drone_marker_pub_->publish(arrow);
    };

    // -Z body axis (thrust direction) → red
    publish_arrow(1, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0f, 0.0f, 0.0f);
    // +X body axis (forward)          → green
    publish_arrow(2, Eigen::Vector3d(1.0, 0.0,  0.0), 0.0f, 1.0f, 0.0f);

    // ── Real position / velocity for logging ─────────────────────────────
    geometry_msgs::msg::Vector3 rp, rv;
    rp.x = state.position.x();  rp.y = state.position.y();  rp.z = state.position.z();
    rv.x = state.velocity.x();  rv.y = state.velocity.y();  rv.z = state.velocity.z();
    real_pos_pub_->publish(rp);
    real_vel_pub_->publish(rv);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Service callback
// ═══════════════════════════════════════════════════════════════════════════

void ThroughWindowMode::through_window_callback(
    const pegasus_msgs::srv::ThroughWindow::Request::SharedPtr  req,
    const pegasus_msgs::srv::ThroughWindow::Response::SharedPtr res) {

    window_position_[0] = req->window_position[0];
    window_position_[1] = req->window_position[1];
    window_position_[2] = req->window_position[2];
    window_angle_       = req->window_angle;
    window_point_set_   = true;
    t_                  = 0.0;

    res->success = true;
    RCLCPP_WARN(node_->get_logger(),
        "Window set to (%.2f, %.2f, %.2f), angle=%.2f",
        window_position_[0], window_position_[1],
        window_position_[2], window_angle_);
}

} // namespace autopilot

PLUGINLIB_EXPORT_CLASS(autopilot::ThroughWindowMode, autopilot::Mode)