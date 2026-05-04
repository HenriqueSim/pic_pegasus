#include "autopilot_modes/mode_through_window.hpp"
#include <pegasus_utils/rotations.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace autopilot {

ThroughWindowMode::~ThroughWindowMode() {
    this->window_service_.reset();
}

void ThroughWindowMode::initialize() {
    node_->declare_parameter<std::string>("autopilot.ThroughWindowMode.set_window_service", "set_window");
    this->window_service_ = this->node_->create_service<pegasus_msgs::srv::ThroughWindow>(
        node_->get_parameter("autopilot.ThroughWindowMode.set_window_service").as_string(),
        std::bind(&ThroughWindowMode::throughWindowCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Load trajectory timing
    node_->declare_parameter<double>("autopilot.polynomial_trajectory.timing.tf", 5.0);
    T_final_ = node_->get_parameter("autopilot.polynomial_trajectory.timing.tf").as_double();
    // Warn the Tfinal loaded
    RCLCPP_WARN(this->node_->get_logger(), "Loaded trajectory timing: T_final = %.2f", T_final_);

    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.kp", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.kd", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.ki", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.kr", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.YawLessController.gains.max_output", std::vector<double>());
     // Load the controller gains from the parameter server
    auto kp_ = node_->get_parameter("autopilot.YawLessController.gains.kp").as_double_array();
    auto kd_ = node_->get_parameter("autopilot.YawLessController.gains.kd").as_double_array();
    auto ki_ = node_->get_parameter("autopilot.YawLessController.gains.ki").as_double_array();
    auto kr_ = node_->get_parameter("autopilot.YawLessController.gains.kr").as_double_array();
    auto min_output_ = node_->get_parameter("autopilot.YawLessController.gains.min_output").as_double_array();
    auto max_output_ = node_->get_parameter("autopilot.YawLessController.gains.max_output").as_double_array();

    // ========== PREDICTIVE CONTROL PARAMETERS ==========
    node_->declare_parameter<bool>("autopilot.predictive_control.enabled", false);
    node_->declare_parameter<double>("autopilot.predictive_control.delay", 0.01);
    node_->declare_parameter<double>("autopilot.predictive_control.dt_control", 0.01);
    node_->declare_parameter<std::vector<double>>("autopilot.predictive_control.inertia", 
        std::vector<double>{0.029125, 0.029125, 0.055225});
    
    use_predictive_control_ = node_->get_parameter("autopilot.predictive_control.enabled").as_bool();
    delay_ = node_->get_parameter("autopilot.predictive_control.delay").as_double();
    dt_control_ = node_->get_parameter("autopilot.predictive_control.dt_control").as_double();
    
    auto inertia_vec = node_->get_parameter("autopilot.predictive_control.inertia").as_double_array();
    inertia_ = Eigen::Vector3d(inertia_vec[0], inertia_vec[1], inertia_vec[2]).asDiagonal();
    
    // Calculate prediction horizon
    K_ = static_cast<int>(std::round(delay_ / dt_control_));
    
    // Initialize control queue with hovering controls
    queued_control_actions_.clear();
    for (int i = 0; i < K_; ++i) {
        queued_control_actions_.push_back(ControlAction());  // Default: a_des=0, omega=0, T=g
    }

    // Initialize predicted states queue with k current states get from the vehicle state (or default if not available)
    State current_state = get_vehicle_state();
    PredictedState initial_predicted_state = state_to_predicted(current_state);
    queued_predicted_states_.clear();
    for (int i = 0; i < K_; ++i) {
        queued_predicted_states_.push_back(initial_predicted_state);
    }

    predicted_position_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/predicted_position", 10);
    predicted_velocity_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/predicted_velocity", 10);
    predicted_attitude_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/predicted_attitude", 10);
    
    RCLCPP_INFO(this->node_->get_logger(), 
        "Predictive control: %s, K=%d, delay=%.3fs, dt_control=%.3fs",
        use_predictive_control_ ? "ENABLED" : "DISABLED", K_, delay_, dt_control_);
    // ================= END PREDICTIVE CONTROL =================    

    // For loop to assign gains
    for(unsigned int i=0; i < 3; i++) {
        kp[i] = kp_[i];
        kd[i] = kd_[i];
        ki[i] = ki_[i];
        kr(i, i) = kr_[i];
        min_output[i] = min_output_[i];
        max_output[i] = max_output_[i];
    }
    // ROS warning to show the loaded gains
    RCLCPP_INFO(this->node_->get_logger(), "ThroughWindowMode controller gains initialized.");
    Eigen::Vector3d kff(1.0, 1.0, 1.0);


    auto load_vector_param = [&](const std::string& name) -> Eigen::VectorXd {
        std::vector<double> vec;
        node_->declare_parameter(name, vec);
        vec = node_->get_parameter(name).as_double_array();
        return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
    };

    x_segments.clear();
    y_segments.clear();
    z_segments.clear();

    x_segments.push_back(load_vector_param("autopilot.polynomial_trajectory.x_axis.segment_1"));
    y_segments.push_back(load_vector_param("autopilot.polynomial_trajectory.y_axis.segment_1"));
    z_segments.push_back(load_vector_param("autopilot.polynomial_trajectory.z_axis.segment_1"));
    

    x_coeffs_ = x_segments[0];
    y_coeffs_ = y_segments[0];
    z_coeffs_ = z_segments[0];

    RCLCPP_INFO(this->node_->get_logger(), "Loaded trajectory coefficients from parameters");

    path_publisher_ = this->node_->create_publisher<nav_msgs::msg::Path>("/debug/trajectory", 10);
    trajectory_accel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/trajectory_acceleration", 10);
    trajectory_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/trajectory_velocity", 10);
    trajectory_position_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/trajectory_position", 10);
    trajectory_marker_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("/debug/trajectory_markers", 10);
    drone_marker_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("/debug/drone_marker", 10);
    velocity_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/real_velocity", 10);
    position_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/real_position", 10);
    acceleration_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/real_acceleration", 10);
    drag_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/drag_force", 10);

    // Controller debug publishers
    attitude_error_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/attitude_error", 10);
    attitude_ref_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/attitude_reference", 10);
    attitude_rate_ref_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/attitude_rate_reference", 10);
    attitude_real_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/attitude_real", 10);
    attitude_rate_real_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/debug/attitude_rate_real", 10);

    thrust_pub_ = this->node_->create_publisher<std_msgs::msg::Float64>("/debug/thrust", 10);
    thrust_no_drag_pub_ = this->node_->create_publisher<std_msgs::msg::Float64>("/debug/thrust_no_drag", 10);


    // attitude_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("debug/real_attitude", 10);

    VehicleConstants vehicle_constants = get_vehicle_constants();
    mass_ = vehicle_constants.mass;

    generatePathMsg();
    generateTrajectoryMarkers(0.1);
}

bool ThroughWindowMode::enter() {
    if (!this->window_point_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Settings not set - cannot enter ThroughWindowMode mode.");
        return false;
    }
    this->window_point_set_ = false;
    this->t = 0.0;
    return true;
}

bool ThroughWindowMode::exit() {
    return true;
}

void ThroughWindowMode::update(double dt) {
    t += dt;
    
    if (t > T_final_) {
        t = T_final_;
        vehicle_state = get_vehicle_state();
        controller_->set_position(vehicle_state.position, 0.0, dt);
    } else {
        x_coeffs_ = x_segments[0];
        y_coeffs_ = y_segments[0];
        z_coeffs_ = z_segments[0];
        
        // Determine which time to use for reference
        double t_ref = t;
        // if (use_predictive_control_) {
        //     // Use future reference (K steps ahead)
        //     t_ref = t + K_ * dt_control_;
        //     // Clamp to trajectory duration
        //     if (t_ref > T_final_) t_ref = T_final_;
        // }
        
        Eigen::VectorXd time_vec(x_coeffs_.size());
        Eigen::VectorXd d_time_vec(x_coeffs_.size());
        Eigen::VectorXd dd_time_vec(x_coeffs_.size());
        Eigen::VectorXd ddd_time_vec(x_coeffs_.size());
        
        for (int i = 0; i < time_vec.size(); ++i) {
            time_vec(i) = std::pow(t_ref, i);
            d_time_vec(i) = (i == 0) ? 0.0 : i * std::pow(t_ref, i - 1);
            dd_time_vec(i) = (i <= 1) ? 0.0 : i * (i - 1) * std::pow(t_ref, i - 2);
            ddd_time_vec(i) = (i <= 2) ? 0.0 : i * (i - 1) * (i - 2) * std::pow(t_ref, i - 3);
        }
        
        // Assign references
        Eigen::Vector3d position(x_coeffs_.dot(time_vec), y_coeffs_.dot(time_vec), z_coeffs_.dot(time_vec));
        Eigen::Vector3d velocity(x_coeffs_.dot(d_time_vec), y_coeffs_.dot(d_time_vec), z_coeffs_.dot(d_time_vec));
        Eigen::Vector3d acceleration(x_coeffs_.dot(dd_time_vec), y_coeffs_.dot(dd_time_vec), z_coeffs_.dot(dd_time_vec));
        Eigen::Vector3d jerk(x_coeffs_.dot(ddd_time_vec), y_coeffs_.dot(ddd_time_vec), z_coeffs_.dot(ddd_time_vec));
        
        vehicle_state = get_vehicle_state();

        double rho_x = 0.6;//*1.0;
        double rho_y = 0.6; //*1.5;
        double rho_z = 0.4;

        // // Compute drag force (assuming drag is proportional to velocity)
        Eigen::Vector3d rho(rho_x, rho_y, rho_z);
        Eigen::Vector3d drag_force = rho.cwiseProduct(velocity);
        Eigen::Vector3d rho_acc = rho.cwiseProduct(acceleration);   // Result (jx,jy,jz) = (rho_x*ax, rho_y*ay, rho_z*az)

        // // Quadratic drag: f_drag = -rho * |v| * v   (per axis)
        // Eigen::Vector3d rho(rho_x, rho_y, rho_z);
        // Eigen::Vector3d drag_force =
        //     -rho.cwiseProduct(velocity.cwiseAbs().cwiseProduct(velocity));

        // // Time derivative of quadratic drag:
        // // d/dt(-rho * |v| * v) = -2 * rho * |v| * a
        // Eigen::Vector3d rho_acc =
        //     -2.0 * rho.cwiseProduct(velocity.cwiseAbs().cwiseProduct(acceleration));


        debuggingAttitudeRefs(acceleration + drag_force, jerk + rho_acc, vehicle_state);        
        // debuggingAttitudeRefs(acceleration, jerk, vehicle_state);
        
        // Publish reference trajectory
        position_msg.x = position.x();
        position_msg.y = position.y();
        position_msg.z = position.z();
        velocity_msg.x = velocity.x();
        velocity_msg.y = velocity.y();
        velocity_msg.z = velocity.z();
        acceleration_msg.x = acceleration.x();
        acceleration_msg.y = acceleration.y();
        acceleration_msg.z = acceleration.z() - 9.81;
        
        trajectory_position_pub_->publish(position_msg);
        trajectory_vel_pub_->publish(velocity_msg);
        trajectory_accel_pub_->publish(acceleration_msg);
        
        // Call set_position with the appropriate reference
        // set_position(position, velocity, acceleration, jerk, 0.0, 0.0, dt);
        set_position(position, velocity, acceleration+drag_force, jerk+rho_acc, 0.0, 0.0, dt);
        publishDroneMarker();
    }
}

void ThroughWindowMode::debuggingRealAcceleration(const State& vehicle_state, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
    // Get rotation matrix from quaternion
    Eigen::Matrix3d R = vehicle_state.attitude.toRotationMatrix();
    Eigen::Vector3d F_des;
    
    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = position - vehicle_state.position;
    Eigen::Vector3d vel_error = velocity - vehicle_state.velocity;

    Eigen::Vector3d external_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    external_force[0] = acceleration[0];
    external_force[1] = acceleration[1];
    external_force[2] = acceleration[2] - 9.81;


    // Get thrust force (assumed to be along body z-axis)
    for(unsigned int i=0; i < 3; i++) F_des[i] = mass_ * compute_output(pos_error[i], vel_error[i], external_force[i],i);
    
    // Get the desired total thrust (in Newtons) in Z_B direction (u_1)
    Eigen::Vector3d Z_B = R.col(2);
    double T = -F_des.dot(Z_B);

    // Compute thrust force vector in inertial frame
    Eigen::Vector3d F_thrust_inertial = -T * Z_B;

    // Compute acceleration: a = (F_thrust / m) + g
    Eigen::Vector3d acceleration_inertial = (F_thrust_inertial / mass_) + Eigen::Vector3d(0.0, 0.0, g);

    // Publish real acceleration for debugging
    geometry_msgs::msg::Vector3 real_accel_msg;
    real_accel_msg.x = acceleration_inertial.x();
    real_accel_msg.y = acceleration_inertial.y();
    real_accel_msg.z = acceleration_inertial.z();
    acceleration_pub_->publish(real_accel_msg);

    // // publish thrust which is just a double for debugging
    // std_msgs::msg::Float64 thrust_msg_;
    // thrust_msg_.data = T;
    // thrust_pub_->publish(thrust_msg_);
}

double ThroughWindowMode::compute_output(double pos_error, double vel_error, double external_force, unsigned int i) {

    return kp[i] * pos_error + kd[i] * vel_error + external_force;
}

void ThroughWindowMode::debuggingAttitudeRefs(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const State& vehicle_state) {
    double yaw_rate_rad = 0.0; // desired yaw rate in rad/s

    // build b and b_dot (b_dot = jerk, since g is constant)
    Eigen::Vector3d b = acceleration - Eigen::Vector3d(0.0, 0.0, g);

    // Compute desired force output
    Eigen::Vector3d F_des = mass_ * b;

    //Compute the desired body-frame z-axis (NED)
    Eigen::Vector3d Z_b_des = -F_des / F_des.norm();

    // Compute Y_C
    Eigen::Vector3d Y_C = Eigen::Vector3d(0.0, 1.0, 0.0);

    // Compute X_B_des
    Eigen::Vector3d X_b_des = Y_C.cross(Z_b_des);
    X_b_des = X_b_des / X_b_des.norm();

    // Compute Y_B_des
    Eigen::Vector3d Y_b_des = Z_b_des.cross(X_b_des);
    Y_b_des = Y_b_des / Y_b_des.norm();

    // Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
    Eigen::Matrix3d R_des;
    R_des.col(0) = X_b_des;
    R_des.col(1) = Y_b_des;
    R_des.col(2) = Z_b_des;

    // Get the desired Euler-angles (for debugging purposes only)
    Eigen::Vector3d euler_angles_des = R_des.eulerAngles(2, 1, 0);

    double T = F_des.norm();

    // Compute the desired angular velocity for the feed-forward terms
    // Eigen::Vector3d w_des;
    // w_des(0) =  mass_ / T * Y_b_des.dot(jerk);
    // w_des(1) = -mass_ / T * X_b_des.dot(jerk);
    // w_des(2) = -mass_/T * Z_b_des.transpose().dot(Y_C) * X_b_des.transpose().dot(jerk) / Y_C.cross(Z_b_des).norm(); // + yaw_rate_rad * Z_b_des[2]
    Eigen::Vector3d e_3 = Eigen::Vector3d(0,0,1);
    Eigen::Matrix3d PI_e3 = Eigen::Matrix3d::Identity() - e_3 * e_3.transpose();
    Eigen::Vector3d w_des = - PI_e3 * (R_des.transpose() * (Z_b_des.cross(jerk))) * (mass_ / T);


    // Publish real attitude and attitude rate for debugging
    Eigen::Vector3d euler_angles_real = vehicle_state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
    // Make sure the angles are the solution between -pi/2 and pi/2
    if (euler_angles_real(0) > M_PI/2) { euler_angles_real(0) -= M_PI; }
    else if (euler_angles_real(0) < -M_PI/2) { euler_angles_real(0) += M_PI; }
    if (euler_angles_real(1) > M_PI/2) { euler_angles_real(1) =  -1*euler_angles_real(1) + M_PI; }
    else if (euler_angles_real(1) < -M_PI/2) { euler_angles_real(1) = -1*euler_angles_real(1) - M_PI; }
    if (euler_angles_real(2) > M_PI/2) { euler_angles_real(2) -= M_PI; }
    else if (euler_angles_real(2) < -M_PI/2) { euler_angles_real(2) += M_PI; }

    Eigen::Vector3d w_real = vehicle_state.angular_velocity; // in rad/s

    geometry_msgs::msg::Vector3 att_real_msg;
    geometry_msgs::msg::Vector3 att_rate_real_msg;

    geometry_msgs::msg::Vector3 att_ref_msg;
    geometry_msgs::msg::Vector3 att_rate_ref_msg;

    // Build and publish messages (same types as your ref messages)
    att_ref_msg.x = euler_angles_des(2) * RAD2DEG;    // roll
    att_ref_msg.y = euler_angles_des(1) * RAD2DEG;    // pitch
    att_ref_msg.z = euler_angles_des(0) * RAD2DEG;    // yaw
    att_real_msg.x = euler_angles_real(2) * RAD2DEG;  // roll Real
    att_real_msg.y = euler_angles_real(1) * RAD2DEG;  // pitch Real
    att_real_msg.z = euler_angles_real(0) * RAD2DEG;  // yaw Real

    att_rate_ref_msg.x = w_des(0) * RAD2DEG;    // roll rate
    att_rate_ref_msg.y = w_des(1) * RAD2DEG;    // pitch rate
    att_rate_ref_msg.z = w_des(2) * RAD2DEG;    // yaw rate
    att_rate_real_msg.x = w_real(0) * RAD2DEG;  // roll rate Real
    att_rate_real_msg.y = w_real(1) * RAD2DEG;  // pitch rate Real
    att_rate_real_msg.z = w_real(2) * RAD2DEG;  // yaw rate Real

    // Get the current attitude in quaternion and generate a rotation matrix
    Eigen::Matrix3d R = vehicle_state.attitude.toRotationMatrix();

    // r3 error message
    Eigen::Vector3d r3_error = PI_e3 * e_3.cross(R.transpose() * Z_b_des);
    geometry_msgs::msg::Vector3 r3_error_msg;
    r3_error_msg.x = r3_error(0);
    r3_error_msg.y = r3_error(1);
    r3_error_msg.z = r3_error(2);

    // publish
    attitude_error_pub_->publish(r3_error_msg);
    attitude_ref_pub_->publish(att_ref_msg);
    attitude_rate_ref_pub_->publish(att_rate_ref_msg);
    attitude_real_pub_->publish(att_real_msg);
    attitude_rate_real_pub_->publish(att_rate_real_msg);
}

void ThroughWindowMode::generatePathMsg() {
    path_msg_.header.frame_id = "world";
    path_msg_.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    path_msg_.poses.clear();

    for (int i = 0; i <= 50; ++i) {
        double tau = (T_final_ * i) / 50.0;
        x_coeffs_ = x_segments[0];
        y_coeffs_ = y_segments[0];
        z_coeffs_ = z_segments[0];
    
        Eigen::VectorXd time_vec(x_coeffs_.size());

        for (int j = 0; j < time_vec.size(); ++j) {
            time_vec(j) = std::pow(tau, j);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        pose.pose.position.x = x_coeffs_.dot(time_vec);
        pose.pose.position.y = -y_coeffs_.dot(time_vec);
        pose.pose.position.z = -z_coeffs_.dot(time_vec);
        pose.pose.orientation.w = 1.0;
        path_msg_.poses.push_back(pose);
    }
}

void ThroughWindowMode::generateTrajectoryMarkers(double dt_marker) {
    marker_msg_.header.frame_id = "world";
    marker_msg_.header.stamp = node_->now();
    marker_msg_.ns = "trajectory";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.scale.x = 0.05;
    marker_msg_.scale.y = 0.05;
    marker_msg_.scale.z = 0.05;

    // Define distinct colors for each segment
    std_msgs::msg::ColorRGBA color_seg1, color_seg2, color_seg3;
    color_seg1.a = color_seg2.a = color_seg3.a = 1.0;
    color_seg1.r = 1.0; color_seg1.g = 0.0; color_seg1.b = 0.0;  // Red
    color_seg2.r = 0.0; color_seg2.g = 1.0; color_seg2.b = 0.0;  // Green
    color_seg3.r = 0.0; color_seg3.g = 0.0; color_seg3.b = 1.0;  // Blue

    for (double tau = 0.0; tau <= T_final_; tau += dt_marker) {
        std_msgs::msg::ColorRGBA current_color;

        x_coeffs_ = x_segments[0];
        y_coeffs_ = y_segments[0];
        z_coeffs_ = z_segments[0];
        current_color = color_seg1;

        Eigen::VectorXd time_vec(x_coeffs_.size());

        for (int j = 0; j < time_vec.size(); ++j) {
            time_vec(j) = std::pow(tau, j);
        }

        geometry_msgs::msg::Point point;
        point.x = x_coeffs_.dot(time_vec);
        point.y = -y_coeffs_.dot(time_vec);
        point.z = -z_coeffs_.dot(time_vec);

        marker_msg_.points.push_back(point);
        marker_msg_.colors.push_back(current_color);  // Assign color per point
    }
}

void ThroughWindowMode::publishDroneMarker() {
    autopilot::State vehicle_state = get_vehicle_state();
    rclcpp::Time current_time = node_->now();

    // Convert NED to ENU for position
    Eigen::Vector3d position_enu(
        vehicle_state.position.x(),
        -vehicle_state.position.y(),
        -vehicle_state.position.z()
    );

    // Convert attitude to ENU
    Eigen::AngleAxisd ned_to_enu_rot(M_PI, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_ned_to_enu(ned_to_enu_rot);
    Eigen::Quaterniond q_enu = q_ned_to_enu * vehicle_state.attitude;

    // ---------- Base marker (drone body as cube/prism) ----------
    visualization_msgs::msg::Marker base_marker;
    base_marker.header.frame_id = "world";
    base_marker.header.stamp = current_time;
    base_marker.ns = "drone";
    base_marker.id = 0;
    base_marker.type = visualization_msgs::msg::Marker::CUBE;
    base_marker.action = visualization_msgs::msg::Marker::ADD;

    base_marker.pose.position.x = position_enu.x();
    base_marker.pose.position.y = position_enu.y();
    base_marker.pose.position.z = position_enu.z();
    base_marker.pose.orientation.x = q_enu.x();
    base_marker.pose.orientation.y = q_enu.y();
    base_marker.pose.orientation.z = q_enu.z();
    base_marker.pose.orientation.w = q_enu.w();

    base_marker.scale.x = 0.3;  // adjust to match drone dimensions
    base_marker.scale.y = 0.3;
    base_marker.scale.z = 0.1;

    base_marker.color.r = 0.0;
    base_marker.color.g = 0.7;
    base_marker.color.b = 1.0;
    base_marker.color.a = 1.0;

    // ---- Orientation marker (negative body z-axis) ----
    visualization_msgs::msg::Marker z_axis_marker;
    z_axis_marker.header.frame_id = "world";
    z_axis_marker.header.stamp = current_time;
    z_axis_marker.ns = "drone";
    z_axis_marker.id = 1;
    z_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    z_axis_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = position_enu.x();
    start.y = position_enu.y();
    start.z = position_enu.z();

    // -Z axis in NED (i.e., the drone's downward normal)
    Eigen::Vector3d minus_z_axis_ned(0.0, 0.0, -1.0);

    // Transform -Z body axis to ENU
    Eigen::Vector3d minus_z_axis_enu = q_enu * minus_z_axis_ned.normalized();

    // Scale and compute end point
    Eigen::Vector3d arrow_tip = position_enu + 0.5 * minus_z_axis_enu;
    end.x = arrow_tip.x();
    end.y = arrow_tip.y();
    end.z = arrow_tip.z();

    z_axis_marker.points.push_back(start);
    z_axis_marker.points.push_back(end);

    z_axis_marker.scale.x = 0.05;  // shaft diameter
    z_axis_marker.scale.y = 0.1;   // head diameter
    z_axis_marker.scale.z = 0.1;   // head length

    z_axis_marker.color.r = 1.0;
    z_axis_marker.color.g = 0.0;
    z_axis_marker.color.b = 0.0;
    z_axis_marker.color.a = 1.0;

    // ---- X-axis marker (body +X axis) ----
    visualization_msgs::msg::Marker x_axis_marker;
    x_axis_marker.header.frame_id = "world";
    x_axis_marker.header.stamp = current_time;
    x_axis_marker.ns = "drone";
    x_axis_marker.id = 2;
    x_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    x_axis_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point x_start, x_end;
    x_start.x = position_enu.x();
    x_start.y = position_enu.y();
    x_start.z = position_enu.z();

    // +X axis in NED body frame
    Eigen::Vector3d x_axis_ned(1.0, 0.0, 0.0);

    // Transform +X body axis to ENU using attitude
    Eigen::Vector3d x_axis_enu = q_enu * x_axis_ned.normalized();

    // Scale for visualization
    Eigen::Vector3d x_arrow_tip = position_enu + 0.5 * x_axis_enu;
    x_end.x = x_arrow_tip.x();
    x_end.y = x_arrow_tip.y();
    x_end.z = x_arrow_tip.z();

    x_axis_marker.points.push_back(x_start);
    x_axis_marker.points.push_back(x_end);

    x_axis_marker.scale.x = 0.05;  // shaft diameter
    x_axis_marker.scale.y = 0.1;   // head diameter
    x_axis_marker.scale.z = 0.1;   // head length

    // Color (green)
    x_axis_marker.color.r = 0.0;
    x_axis_marker.color.g = 1.0;
    x_axis_marker.color.b = 0.0;
    x_axis_marker.color.a = 1.0;

    // Publish both markers
    drone_marker_pub_->publish(base_marker);
    drone_marker_pub_->publish(z_axis_marker);
    drone_marker_pub_->publish(x_axis_marker);

    // // ---- Position Vector3 ----
    geometry_msgs::msg::Vector3 real_position_msg;
    real_position_msg.x = vehicle_state.position.x();
    real_position_msg.y = vehicle_state.position.y();
    real_position_msg.z = vehicle_state.position.z();
    position_pub_->publish(real_position_msg);

    // // ---- Velocity Vector3 ----
    geometry_msgs::msg::Vector3 real_velocity_msg;
    real_velocity_msg.x = vehicle_state.velocity.x();
    real_velocity_msg.y = vehicle_state.velocity.y();
    real_velocity_msg.z = vehicle_state.velocity.z();
    velocity_pub_->publish(real_velocity_msg);
}

void ThroughWindowMode::set_position(
    const Eigen::Vector3d& position, 
    const Eigen::Vector3d& velocity, 
    const Eigen::Vector3d& acceleration, 
    const Eigen::Vector3d& jerk, 
    double yaw, 
    double yaw_rate, 
    double dt) 
{
    // Get the current state of the vehicle
    State state = get_vehicle_state();
    
    if (use_predictive_control_) {
        // ========== PREDICTIVE CONTROL PATH ==========
        
        // Step 1: Predict future state by applying queued controls
        int integration_steps = static_cast<int>(std::round(dt_control_ / dt));
        if (integration_steps < 1) integration_steps = 1;
        
        PredictedState predicted_state = predict_state(state, dt_control_, integration_steps);
        
        // Step 2: Compute future reference (K steps ahead)
        double t_future = t + K_ * dt_control_;
        if (t_future > T_final_) t_future = T_final_;
        
        // Evaluate trajectory at future time
        Eigen::VectorXd time_vec(x_coeffs_.size());
        Eigen::VectorXd d_time_vec(x_coeffs_.size());
        Eigen::VectorXd dd_time_vec(x_coeffs_.size());
        Eigen::VectorXd ddd_time_vec(x_coeffs_.size());
        
        for (int i = 0; i < time_vec.size(); ++i) {
            time_vec(i) = std::pow(t_future, i);
            d_time_vec(i) = (i == 0) ? 0.0 : i * std::pow(t_future, i - 1);
            dd_time_vec(i) = (i <= 1) ? 0.0 : i * (i - 1) * std::pow(t_future, i - 2);
            ddd_time_vec(i) = (i <= 2) ? 0.0 : i * (i - 1) * (i - 2) * std::pow(t_future, i - 3);
        }
        
        Eigen::Vector3d position_future(x_coeffs_.dot(time_vec), y_coeffs_.dot(time_vec), z_coeffs_.dot(time_vec));
        Eigen::Vector3d velocity_future(x_coeffs_.dot(d_time_vec), y_coeffs_.dot(d_time_vec), z_coeffs_.dot(d_time_vec));
        Eigen::Vector3d acceleration_future(x_coeffs_.dot(dd_time_vec), y_coeffs_.dot(dd_time_vec), z_coeffs_.dot(dd_time_vec));
        Eigen::Vector3d jerk_future(x_coeffs_.dot(ddd_time_vec), y_coeffs_.dot(ddd_time_vec), z_coeffs_.dot(ddd_time_vec));
        
        // Step 3: Compute new control action based on predicted state and future reference
        ControlAction new_control = compute_control_action(
            position_future,
            velocity_future,
            acceleration_future,
            jerk_future,
            predicted_state,
            yaw,
            yaw_rate,
            dt
        );
        
        // Step 4: Update queues (shift and add)
        queued_control_actions_.pop_front();  // Remove oldest
        queued_control_actions_.push_back(new_control);  // Add newest
        
        queued_predicted_states_.pop_front();  // Remove oldest
        queued_predicted_states_.push_back(predicted_state);  // Add newest
        
        // Step 5: Apply the control that was calculated now and publish the state that was computed K steps ago
        PredictedState published_state = queued_predicted_states_.front();
        // Send to low-level controller
        controller_->set_attitude_rate(new_control.attitude_rate, new_control.thrust);
        
        // Publish predicted state for visualization
        geometry_msgs::msg::Vector3 pred_pos_msg;
        pred_pos_msg.x = published_state.position.x();
        pred_pos_msg.y = published_state.position.y();
        pred_pos_msg.z = published_state.position.z();
        predicted_position_pub_->publish(pred_pos_msg);
        
        geometry_msgs::msg::Vector3 pred_vel_msg;
        pred_vel_msg.x = published_state.velocity.x();
        pred_vel_msg.y = published_state.velocity.y();
        pred_vel_msg.z = published_state.velocity.z();
        predicted_velocity_pub_->publish(pred_vel_msg);
        
        // Convert predicted rotation to euler for visualization
        Eigen::Vector3d pred_euler = published_state.rotation.eulerAngles(2, 1, 0);
        geometry_msgs::msg::Vector3 pred_att_msg;
        pred_att_msg.x = Pegasus::Rotations::rad_to_deg(pred_euler[2]);  // Roll
        pred_att_msg.y = Pegasus::Rotations::rad_to_deg(pred_euler[1]);  // Pitch
        pred_att_msg.z = Pegasus::Rotations::rad_to_deg(pred_euler[0]);  // Yaw
        
        // Normalize angles to be within ±110 degrees
        auto normalize_angle = [](double& angle) {
            while (angle > 110.0) angle -= 180.0;
            while (angle < -110.0) angle += 180.0;
        };
        
        normalize_angle(pred_att_msg.x);
        normalize_angle(pred_att_msg.y);
        normalize_angle(pred_att_msg.z);
        predicted_attitude_pub_->publish(pred_att_msg);
    } else {
        // ========== STANDARD CONTROL PATH (NO PREDICTION) ==========
        
        // Get the current attitude in quaternion and generate a rotation matrix
        Eigen::Matrix3d R = state.attitude.toRotationMatrix();
        
        // Compute the position error and velocity error
        Eigen::Vector3d pos_error = position - state.position;
        Eigen::Vector3d vel_error = velocity - state.velocity;
        
        Eigen::Vector3d external_force = Eigen::Vector3d(0.0, 0.0, 0.0);
        external_force[0] = acceleration[0];
        external_force[1] = acceleration[1];
        external_force[2] = acceleration[2] - 9.81;
        
        // Compute the desired force output using a PID scheme
        Eigen::Vector3d F_des;
        for(unsigned int i=0; i < 3; i++) {
            F_des[i] = mass_ * compute_output(pos_error[i], vel_error[i], external_force[i], dt);
        }
        
        // Compute the desired body-frame axis Z_b (b3d)
        Eigen::Vector3d Z_b_des = -F_des / F_des.norm();
        
        // Get yaw in radians
        double yaw_rad = Pegasus::Rotations::deg_to_rad(yaw);
        double yaw_rate_rad = Pegasus::Rotations::deg_to_rad(yaw_rate);
        
        // Compute Y_C
        Eigen::Vector3d Y_C = Eigen::Vector3d(-sin(yaw_rad), cos(yaw_rad), 0.0);
        
        // Compute X_B_des
        Eigen::Vector3d X_b_des = Y_C.cross(Z_b_des);
        X_b_des = X_b_des / X_b_des.norm();
        
        // Compute Y_B_des
        Eigen::Vector3d Y_b_des = Z_b_des.cross(X_b_des);
        Y_b_des = Y_b_des / Y_b_des.norm();
        
        // Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        Eigen::Matrix3d R_des;
        R_des.col(0) = X_b_des;
        R_des.col(1) = Y_b_des;
        R_des.col(2) = Z_b_des;
        
        // Get the desired total thrust (in Newtons) in Z_B direction
        double T = F_des.norm();
        
        // Compute the desired angular velocity for the feed-forward terms
        Eigen::Vector3d e_3 = Eigen::Vector3d(0,0,1);
        Eigen::Matrix3d PI_e3 = Eigen::Matrix3d::Identity() - e_3 * e_3.transpose();
        
        // Compute the target attitude rate
        Eigen::Vector3d attitude_rate =
            kr * e_3.cross(R.transpose() * Z_b_des)
            - PI_e3 * (R.transpose() * (Z_b_des.cross(jerk))) * (mass_ / T);
        
        // Convert the output to degrees
        attitude_rate = Eigen::Vector3d(
            Pegasus::Rotations::rad_to_deg(attitude_rate[0]), 
            Pegasus::Rotations::rad_to_deg(attitude_rate[1]), 
            Pegasus::Rotations::rad_to_deg(attitude_rate[2]));
        
        // Send the attitude rate and thrust to the attitude-rate controller
        controller_->set_attitude_rate(attitude_rate, T);
    }
}

// ========== HELPER FUNCTION: Skew-symmetric matrix ==========
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

ThroughWindowMode::PredictedState ThroughWindowMode::state_to_predicted(const State& state) {
    PredictedState pred;
    pred.position = state.position;
    pred.velocity = state.velocity;
    pred.rotation = state.attitude.toRotationMatrix();
    pred.angular_velocity = state.angular_velocity;
    return pred;
}

// ========== ODE FUNCTION: Rigid body dynamics ==========
ThroughWindowMode::PredictedState ThroughWindowMode::rigid_body_ode(
    const PredictedState& state, 
    const ControlAction& control, 
    double dt) 
{
    PredictedState derivative;
    
    // Position derivative
    derivative.position = state.velocity;
    
    // Velocity derivative (translational dynamics)
    Eigen::Vector3d e3(0, 0, 1);
    Eigen::Vector3d r3 = state.rotation.col(2);  // Z-axis of rotation matrix
    derivative.velocity = -control.thrust * r3 + g * e3;
    
    // Rotation derivative (R_dot = R * skew(omega))
    derivative.rotation = state.rotation * skew(state.angular_velocity);
    
    // Angular velocity derivative (rotational dynamics)
    // omega_dot = J^-1 * (-omega x (J*omega) + tau)
    // Note: control.attitude_rate is used as the torque command here
    // In reality, you'd have a separate torque, but we're using attitude_rate as proxy
    Eigen::Vector3d J_omega = inertia_ * state.angular_velocity;
    Eigen::Vector3d gyroscopic = -state.angular_velocity.cross(J_omega);
    
    // Convert attitude_rate command to torque (simplified)
    // In full implementation, you'd have proper torque from attitude controller
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();  // Placeholder
    derivative.angular_velocity = inertia_.inverse() * (gyroscopic + tau);
    
    return derivative;
}

// ========== RK4 INTEGRATION STEP ==========
ThroughWindowMode::PredictedState ThroughWindowMode::rk4_step(
    const PredictedState& state, 
    const ControlAction& control, 
    double dt) 
{
    // RK4 integration
    PredictedState k1 = rigid_body_ode(state, control, dt);
    
    PredictedState state_k2;
    state_k2.position = state.position + 0.5 * dt * k1.position;
    state_k2.velocity = state.velocity + 0.5 * dt * k1.velocity;
    state_k2.rotation = state.rotation + 0.5 * dt * k1.rotation;
    state_k2.angular_velocity = state.angular_velocity + 0.5 * dt * k1.angular_velocity;
    PredictedState k2 = rigid_body_ode(state_k2, control, dt);
    
    PredictedState state_k3;
    state_k3.position = state.position + 0.5 * dt * k2.position;
    state_k3.velocity = state.velocity + 0.5 * dt * k2.velocity;
    state_k3.rotation = state.rotation + 0.5 * dt * k2.rotation;
    state_k3.angular_velocity = state.angular_velocity + 0.5 * dt * k2.angular_velocity;
    PredictedState k3 = rigid_body_ode(state_k3, control, dt);
    
    PredictedState state_k4;
    state_k4.position = state.position + dt * k3.position;
    state_k4.velocity = state.velocity + dt * k3.velocity;
    state_k4.rotation = state.rotation + dt * k3.rotation;
    state_k4.angular_velocity = state.angular_velocity + dt * k3.angular_velocity;
    PredictedState k4 = rigid_body_ode(state_k4, control, dt);
    
    // Combine
    PredictedState result;
    result.position = state.position + (dt / 6.0) * (k1.position + 2*k2.position + 2*k3.position + k4.position);
    result.velocity = state.velocity + (dt / 6.0) * (k1.velocity + 2*k2.velocity + 2*k3.velocity + k4.velocity);
    result.rotation = state.rotation + (dt / 6.0) * (k1.rotation + 2*k2.rotation + 2*k3.rotation + k4.rotation);
    result.angular_velocity = state.angular_velocity + (dt / 6.0) * 
        (k1.angular_velocity + 2*k2.angular_velocity + 2*k3.angular_velocity + k4.angular_velocity);
    
    // Orthonormalize rotation matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(result.rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    result.rotation = svd.matrixU() * svd.matrixV().transpose();
    
    return result;
}


// ========== PREDICT STATE BY APPLYING QUEUED CONTROLS ==========
ThroughWindowMode::PredictedState ThroughWindowMode::predict_state(
    const State& current_state, 
    double dt_control, 
    int integration_steps) 
{
    // Start from current actual state
    PredictedState predicted = state_to_predicted(current_state);
    
    // Apply each queued control action sequentially
    double dt_integration = dt_control / integration_steps;
    
    for (const auto& control : queued_control_actions_) {
        // Integrate for dt_control duration with this control
        for (int step = 0; step < integration_steps; ++step) {
            predicted = rk4_step(predicted, control, dt_integration);
        }
    }
    
    return predicted;
}

// ========== COMPUTE CONTROL ACTION ==========
ThroughWindowMode::ControlAction ThroughWindowMode::compute_control_action(
    const Eigen::Vector3d& position_ref,
    const Eigen::Vector3d& velocity_ref, 
    const Eigen::Vector3d& acceleration_ref,
    const Eigen::Vector3d& jerk_ref,
    const PredictedState& predicted_state,
    double yaw,
    double yaw_rate,
    double dt)
{
    ControlAction control;
    
    // Compute position and velocity errors
    Eigen::Vector3d pos_error = position_ref - predicted_state.position;
    Eigen::Vector3d vel_error = velocity_ref - predicted_state.velocity;
    
    // External force (feedforward acceleration - gravity)
    Eigen::Vector3d external_force;
    external_force[0] = acceleration_ref[0];
    external_force[1] = acceleration_ref[1];
    external_force[2] = acceleration_ref[2] - g;
    
    // Compute desired force using PID
    Eigen::Vector3d F_des;
    for(unsigned int i=0; i < 3; i++) {
        F_des[i] = mass_ * compute_output(pos_error[i], vel_error[i], external_force[i], dt);
    }
    
    // Compute desired body Z-axis
    Eigen::Vector3d Z_b_des = -F_des / F_des.norm();
    
    // Compute desired rotation matrix
    double yaw_rad = Pegasus::Rotations::deg_to_rad(yaw);
    Eigen::Vector3d Y_C(-sin(yaw_rad), cos(yaw_rad), 0.0);
    
    Eigen::Vector3d X_b_des = Y_C.cross(Z_b_des);
    X_b_des.normalize();
    
    Eigen::Vector3d Y_b_des = Z_b_des.cross(X_b_des);
    Y_b_des.normalize();
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = X_b_des;
    R_des.col(1) = Y_b_des;
    R_des.col(2) = Z_b_des;
    
    // Thrust magnitude
    control.thrust = F_des.norm();
    
    // Compute desired attitude rate
    Eigen::Vector3d e_3(0, 0, 1);
    Eigen::Matrix3d PI_e3 = Eigen::Matrix3d::Identity() - e_3 * e_3.transpose();
    
    control.attitude_rate = 
        kr * e_3.cross(predicted_state.rotation.transpose() * Z_b_des)
        - PI_e3 * (predicted_state.rotation.transpose() * (Z_b_des.cross(jerk_ref))) * (mass_ / control.thrust);
    
    // Convert to degrees
    control.attitude_rate = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(control.attitude_rate[0]),
        Pegasus::Rotations::rad_to_deg(control.attitude_rate[1]),
        Pegasus::Rotations::rad_to_deg(control.attitude_rate[2])
    );
    
    // Store desired acceleration for debugging
    control.a_des = acceleration_ref;
    
    return control;
}


void ThroughWindowMode::throughWindowCallback(const pegasus_msgs::srv::ThroughWindow::Request::SharedPtr request, const pegasus_msgs::srv::ThroughWindow::Response::SharedPtr response) {

    this->window_position[0] = request->window_position[0];
    this->window_position[1] = request->window_position[1];
    this->window_position[2] = request->window_position[2];
    this->window_angle = request->window_angle;
    this->t = 0.0;
    this->window_point_set_ = true;

    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Window set to (%f, %f, %f), angle %f",
                this->window_position[0], this->window_position[1],
                this->window_position[2], this->window_angle);
}

}  // namespace autopilot

PLUGINLIB_EXPORT_CLASS(autopilot::ThroughWindowMode, autopilot::Mode)
