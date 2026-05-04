#include "autopilot_modes/mode_systemID.hpp"
#include <pegasus_utils/rotations.hpp>


namespace autopilot {

SystemIdentification::~SystemIdentification() {
    // Terminate the waypoint service
    this->systemID_service_.reset();
}

void SystemIdentification::initialize() {

    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.SystemIdentification.set_settings_service", "set_settings"); 
    this->systemID_service_ = this->node_->create_service<pegasus_msgs::srv::SystemID>(node_->get_parameter("autopilot.SystemIdentification.set_settings_service").as_string(), std::bind(&SystemIdentification::systemID_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->node_->get_logger(), "SystemIdentification initialized");

    path_publisher_ = this->node_->create_publisher<nav_msgs::msg::Path>("/systemID/debug/trajectory", 10);
    trajectory_accel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/systemID/debug/trajectory_acceleration", 10);
    trajectory_vel_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/systemID/debug/trajectory_velocity", 10);
    trajectory_position_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/systemID/debug/trajectory_position", 10);
    drone_marker_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("/systemID/debug/drone_marker", 10);

    // Controller debug publishers
    attitude_ref_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/systemID/debug/attitude_reference", 10);
    attitude_rate_ref_pub_ = this->node_->create_publisher<geometry_msgs::msg::Vector3>("/systemID/debug/attitude_rate_reference", 10);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    this->mass_ = vehicle_constansts.mass;
}

bool SystemIdentification::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->settings_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Settings not set - cannot enter System Identification mode.");
        return false;
    }

    // Reset the settings flag (to make sure we do not enter twice in this mode without setting new settings)
    this->settings_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool SystemIdentification::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void SystemIdentification::update(double dt) {      //Circular motion + thrust control
    // Get the current state of the vehicle
    State state = get_vehicle_state();
    if (t < 2.0) {
        // Get the vehicle to hover at the origin of the referential.
        this->controller_->set_position({0.0, this->amplitude, -1.5}, 0.0, dt);
        debuggingAttitudeRefs({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0);
        debugTrajectoryRefs({0.0, this->amplitude, -1.5}, {0.0,0.0,0.0}, {0.0,0.0,0.0});
    } else if (t < 30.0) {

        Eigen::Vector3d position_target = compute_position(t);
        Eigen::Vector3d velocity_target = compute_velocity(t);
        Eigen::Vector3d acceleration_target = compute_acceleration(t);
        Eigen::Vector3d jerk_target = compute_jerk(t);

        double rho_x = 0.560;//.0;
        double rho_y = 0.560; //*1.5;
        double rho_z = 0.0;

        Eigen::Vector3d rho(rho_x, rho_y, rho_z);
        Eigen::Vector3d drag_force = rho.cwiseProduct(velocity_target);
        Eigen::Vector3d rho_acc = rho.cwiseProduct(acceleration_target);   // Result (jx,jy,jz) = (rho_x*ax, rho_y*ay, rho_z*az)

        double yaw = 2.0 * M_PI * frequency * (t - 2.0); // Yaw changes over time
        double yaw_rate = 2.0 * M_PI * frequency; // Constant yaw rate
        this->controller_->set_position(position_target, velocity_target, acceleration_target + drag_force, jerk_target + rho_acc, yaw, yaw_rate, dt);

        debuggingAttitudeRefs(acceleration_target + drag_force, jerk_target + rho_acc, yaw, yaw_rate);
        debugTrajectoryRefs(position_target, velocity_target, acceleration_target);
    }
    else {
        Eigen::Vector3d stop_position = state.position;
        this->controller_->set_position({stop_position[0], stop_position[1], -1.5}, 0.0, dt);
        // this->controller_->set_position({0.0, 0.0, -1.5}, 0.0, dt);
    }

    path_publisher_->publish(path_msg_);
    publishDroneMarker();
    this->t += dt; // Increment time
}

void SystemIdentification::debuggingAttitudeRefs(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, double yaw_rad, double yaw_rate_rad) {
    // build b and b_dot (b_dot = jerk, since g is constant)
    double g = 9.81;
    Eigen::Vector3d b = acceleration - Eigen::Vector3d(0.0, 0.0, g);
    // Compute desired force output
    Eigen::Vector3d F_des = mass_ * b;
    //Compute the desired body-frame z-axis (NED)
    Eigen::Vector3d Z_b_des = -F_des / F_des.norm();
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
    // Get the desired Euler-angles (for debugging purposes only)
    Eigen::Vector3d euler_angles_des = R_des.eulerAngles(2, 1, 0);
    double T = F_des.norm();
    // Compute the desired angular velocity for the feed-forward terms
    Eigen::Vector3d w_des;
    w_des(0) =  mass_ / T * Y_b_des.dot(jerk);
    w_des(1) = -mass_ / T * X_b_des.dot(jerk);
    w_des(2) = yaw_rate_rad * Z_b_des[2];

    geometry_msgs::msg::Vector3 att_ref_msg;
    geometry_msgs::msg::Vector3 att_rate_ref_msg;

    // Build and publish messages (same types as your ref messages)
    att_ref_msg.x = euler_angles_des(2) * RAD2DEG;    // roll
    att_ref_msg.y = euler_angles_des(1) * RAD2DEG;    // pitch
    att_ref_msg.z = euler_angles_des(0) * RAD2DEG;    // yaw
    att_rate_ref_msg.x = w_des(0) * RAD2DEG;    // roll rate
    att_rate_ref_msg.y = w_des(1) * RAD2DEG;    // pitch rate
    att_rate_ref_msg.z = w_des(2) * RAD2DEG;    // yaw rate
    // publish
    attitude_ref_pub_->publish(att_ref_msg);
    attitude_rate_ref_pub_->publish(att_rate_ref_msg);
}

void SystemIdentification::debugTrajectoryRefs(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
    // Create messages
    geometry_msgs::msg::Vector3 position_msg, velocity_msg, acceleration_msg;

    position_msg.x = position.x();
    position_msg.y = position.y();
    position_msg.z = position.z();
    velocity_msg.x = velocity.x();
    velocity_msg.y = velocity.y();
    velocity_msg.z = velocity.z();
    acceleration_msg.x = acceleration.x();
    acceleration_msg.y = acceleration.y();
    acceleration_msg.z = acceleration.z() - 9.81; // publish NED acceleration

    // Publish messages
    trajectory_position_pub_->publish(position_msg);
    trajectory_vel_pub_->publish(velocity_msg);
    trajectory_accel_pub_->publish(acceleration_msg);
}

// void SystemIdentification::update(double dt) {
//     // Get the current state of the vehicle
//     State state = get_vehicle_state();
//     if (t < 2.0) {
//         // Get the vehicle to hover at the origin of the referential.
//         this->controller_->set_position({state.position[0], state.position[1], -1.5}, 0.0, dt);

//         // Generate the Vector3Stamped message
//         euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
//         euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

//         euler_msg.vector.x = 0; // roll
//         euler_msg.vector.y = 0; // pitch
//         euler_msg.vector.z = 0; // yaw

//         // Publish the Vector3Stamped message
//         attitude_target_publisher_->publish(euler_msg);
//     } else if (t < 4.0) {

//         // Calculate the position error
//         float height_error = state.position[2] - (-1.5);
        

//         // Calculate the velocity error using the previous position error
//         float velocity_error = state.velocity[2] - 0.0;

//         float Kp = 4.0;
//         float Kd = 4.0;

//         //Update the previous position error
//         this->prev_height_error_ = height_error;
//         Eigen::Vector3d attitude_target = compute_attitude(t);

//         double T = (Kp * height_error + Kd * velocity_error + 9.81 )* this->mass_/ std::abs(std::cos(attitude_target[0] * M_PI / 180 + attitude_target[1] * M_PI / 180));
//         // double T = 10.5 * this->mass_ / std::abs(std::cos(attitude_target[0] * M_PI / 180 + attitude_target[1] * M_PI / 180));

//         this->controller_->set_attitude(attitude_target, T, dt);

//         // Set the header (timestamp and frame_id)
//         euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
//         euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

//         // Set the vector values (roll, pitch, yaw)
//         euler_msg.vector.x = attitude_target[0]; // roll
//         euler_msg.vector.y = attitude_target[1]; // pitch
//         euler_msg.vector.z = attitude_target[2]; // yaw

//         // Publish the Vector3Stamped message
//         attitude_target_publisher_->publish(euler_msg);
//     }
//     else {
//         Eigen::Vector3d stop_position = state.position;
//         this->controller_->set_position({stop_position[0], stop_position[1], -1.5}, 0.0, dt);
//         // this->controller_->set_position({0.0, 0.0, -1.5}, 0.0, dt);
//     }

//     this->t += dt; // Increment time
// }

// // Funtion to compute the attitude target to be sent to the controller based on a sinusoidal
// Eigen::Vector3d SystemIdentification::compute_attitude(double t) {

//     // Initialize the unit vector
//     Eigen::Vector3d attitude = {0.0, 0.0, 0.0};       // Set the z component to 1.0 (to ensure hover mode, cos(~=0) = 1)

//     // Calculate the sinusoidal attitude for the specified axis
//     // double sinValue = std::sin(2*M_PI*this->frequency * t);

//     // Original: attitude[this->axis] = sinValue;
//     // attitude[0] = 0.173 * this->axis[0] * sinValue ;
//     attitude[0] = this->amplitude * this->axis[0];
//     attitude[1] = this->amplitude * this->axis[1];
//     attitude[2] = this->amplitude * this->axis[2];
//     return attitude;
// }

Eigen::Vector3d SystemIdentification::compute_position(double t) {

    // Initialize the unit vector
    Eigen::Vector3d position = {0.0, 0.0, -1.5};       // Set the z component to -1.5 (to ensure hover at 1.5m altitude)

    // Calculate the sinusoidal position for the specified axis
    position[0] =  this->axis[0] * this->amplitude * std::sin(2*M_PI*this->frequency * (t - 2.0)); // Start after 2 seconds
    position[1] = this->axis[1] * this->amplitude * std::cos(2*M_PI*this->frequency * (t - 2.0));
    return position;
}

Eigen::Vector3d SystemIdentification::compute_velocity(double t) {

    // Initialize the unit vector
    Eigen::Vector3d velocity = {0.0, 0.0, 0.0};       

    // Calculate the sinusoidal velocity for the specified axis
    velocity[0] = this->axis[0] * 2 * M_PI * this->frequency * this->amplitude * std::cos(2*M_PI*this->frequency * (t - 2.0));
    velocity[1] = this->axis[1] * -2 * M_PI * this->frequency * this->amplitude * std::sin(2*M_PI*this->frequency * (t - 2.0));

    return velocity;
}

Eigen::Vector3d SystemIdentification::compute_acceleration(double t) {

    // Initialize the unit vector
    Eigen::Vector3d acceleration = {0.0, 0.0, 0.0};       

    // Calculate the sinusoidal acceleration for the specified axis
    acceleration[0] = this->axis[0] * -std::pow(2 * M_PI * this->frequency, 2) * this->amplitude * std::sin(2*M_PI*this->frequency * (t - 2.0));
    acceleration[1] = this->axis[1] * -std::pow(2 * M_PI * this->frequency, 2) * this->amplitude * std::cos(2*M_PI*this->frequency * (t - 2.0));
    return acceleration;
}

Eigen::Vector3d SystemIdentification::compute_jerk(double t) {

    // Initialize the unit vector
    Eigen::Vector3d jerk = {0.0, 0.0, 0.0};       

    // Calculate the sinusoidal jerk for the specified axis
    jerk[0] = this->axis[0] * -std::pow(2 * M_PI * this->frequency, 3) * this->amplitude * std::cos(2*M_PI*this->frequency * (t - 2.0));
    jerk[1] = this->axis[1] * std::pow(2 * M_PI * this->frequency, 3) * this->amplitude * std::sin(2*M_PI*this->frequency * (t - 2.0));
    return jerk;
}

void SystemIdentification::generatePathMsg() {
    path_msg_.header.frame_id = "world";
    path_msg_.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    path_msg_.poses.clear();

    for (int i = 0; i <= 50; ++i) {
        double tau = i / 50.0;

        Eigen::Vector3d position = compute_position(tau*4.0 + 2.0); // Scale tau to match the time range of the maneuver
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        pose.pose.position.x = position[0];
        pose.pose.position.y = -position[1];
        pose.pose.position.z = -position[2];
        pose.pose.orientation.w = 1.0;
        path_msg_.poses.push_back(pose);
    }
}

void SystemIdentification::publishDroneMarker() {
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

    // Publish both markers
    drone_marker_pub_->publish(base_marker);
    drone_marker_pub_->publish(z_axis_marker);
}

void SystemIdentification::systemID_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request, const pegasus_msgs::srv::SystemID::Response::SharedPtr response) {
    
    RCLCPP_WARN(this->node_->get_logger(), "Received System Identification settings request.");
    // Set the settings
    this->axis[0] = request->axis[0];
    this->axis[1] = request->axis[1];
    this->axis[2] = request->axis[2];
    this->amplitude = request->amplitude;
    this->frequency = request->frequency;
    this->t = 0.0;          // Reset the time to 0.0


    // Set the settings flag
    this->settings_set_ = true;

    // Return true to indicate that the settings has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Axis oscilation direction set to (%f, %f, %f) with amplitude %f", this->axis[0], this->axis[1], this->axis[2], this->amplitude);
    generatePathMsg();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::SystemIdentification, autopilot::Mode)