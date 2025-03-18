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
    attitude_target_publisher_ = this->node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude_target", 10);

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

void SystemIdentification::update(double dt) {
    // Get the current state of the vehicle
    State state = get_vehicle_state();
    if (t < 2.0) {
        // Get the vehicle to hover at the origin of the referential.
        this->controller_->set_position({state.position[0], state.position[1], -1.5}, 0.0, dt);

        // Generate the Vector3Stamped message
        euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

        euler_msg.vector.x = 0; // roll
        euler_msg.vector.y = 0; // pitch
        euler_msg.vector.z = 0; // yaw

        // Publish the Vector3Stamped message
        attitude_target_publisher_->publish(euler_msg);
    } else if (t < 4.0) {

        // Calculate the position error
        float height_error = state.position[2] - (-1.5);
        

        // Calculate the velocity error using the previous position error
        float velocity_error = state.velocity[2] - 0.0;

        float Kp = 4.0;
        float Kd = 4.0;

        //Update the previous position error
        this->prev_height_error_ = height_error;
        Eigen::Vector3d attitude_target = compute_attitude(t);

        double T = (Kp * height_error + Kd * velocity_error + 9.81 )* this->mass_/ std::abs(std::cos(attitude_target[0] * M_PI / 180 + attitude_target[1] * M_PI / 180));
        // double T = 10.5 * this->mass_ / std::abs(std::cos(attitude_target[0] * M_PI / 180 + attitude_target[1] * M_PI / 180));

        this->controller_->set_attitude(attitude_target, T, dt);

        // Set the header (timestamp and frame_id)
        euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

        // Set the vector values (roll, pitch, yaw)
        euler_msg.vector.x = attitude_target[0]; // roll
        euler_msg.vector.y = attitude_target[1]; // pitch
        euler_msg.vector.z = attitude_target[2]; // yaw

        // Publish the Vector3Stamped message
        attitude_target_publisher_->publish(euler_msg);
    }
    else {
        Eigen::Vector3d stop_position = state.position;
        this->controller_->set_position({stop_position[0], stop_position[1], -1.5}, 0.0, dt);
        // this->controller_->set_position({0.0, 0.0, -1.5}, 0.0, dt);
    }

    this->t += dt; // Increment time
}

// void WaypointMode::update(double dt) {

//     // Get the current state of the vehicle
//     State state = this->get_vehicle_state();

//     // Calculate the position error
//     Eigen::Vector3d position_error = this->target_pos - state.position;

//     // Calculate the velocity error using the previous position error
//     Eigen::Vector3d velocity_error = (position_error - this->prev_pos_error_) / dt;

//     //Update the previous position error
//     this->prev_pos_error_ = position_error;
//     // Compute the desired control output acceleration for each controller
//     Eigen::Vector3d u;
//     const Eigen::Vector3d g(0.0, 0.0, 9.81);
//     for(unsigned int i=0; i < 3; i++) u[i] = compute_output(position_error[i], velocity_error[i], 0.0, dt, i); // (acceleration[i] - g[i])* mass_
    
//     u[2] = u[2] - g(2);

//     // Convert the acceleration to attitude and thrust
//     Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, Pegasus::Rotations::deg_to_rad(this->target_yaw));

//     // Set the control output
//     Eigen::Vector3d attitude_target = Eigen::Vector3d(
//         Pegasus::Rotations::rad_to_deg(attitude_thrust[0]),
//         Pegasus::Rotations::rad_to_deg(attitude_thrust[1]),
//         Pegasus::Rotations::rad_to_deg(attitude_thrust[2]));

//     // Send the attitude and thrust to the attitude controller
//     this->controller_->set_attitude(attitude_target, attitude_thrust[3]);

//     // Set the header (timestamp and frame_id)
//     euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
//     euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

//     // Set the vector values (roll, pitch, yaw)
//     euler_msg.vector.x = attitude_target[0]; // roll
//     euler_msg.vector.y = attitude_target[1]; // pitch
//     euler_msg.vector.z = attitude_target[2]; // yaw

//     // Publish the Vector3Stamped message
//     attitude_target_publisher_->publish(euler_msg);
//     // // Update and publish the PID statistics
//     // update_statistics(position);
//     // statistics_pub_->publish(pid_statistics_msg_);
// }

// Eigen::Vector4d WaypointMode::get_attitude_thrust_from_acceleration(const Eigen::Vector3d & u, double mass, double yaw) {

//     Eigen::Matrix3d RzT;
//     Eigen::Vector3d r3d;
//     Eigen::Vector4d attitude_thrust;

//     /* Compute the normalized thrust and r3d vector */
//     double T = mass * u.norm();

//     /* Compute the rotation matrix about the Z-axis */
//     RzT << cos(yaw), sin(yaw), 0.0,
//           -sin(yaw), cos(yaw), 0.0,
//                 0.0,      0.0, 1.0;

//     /* Compute the normalized rotation */
//     r3d = -RzT * u / u.norm();

//     // Compute the actual attitude and setup the desired thrust to apply to the vehicle
//     attitude_thrust << asin(-r3d[1]), atan2(r3d[0], r3d[2]), yaw, T;
//     return attitude_thrust;
// }

// double WaypointMode::compute_output(double error_p, double error_d, double feed_forward_ref, double dt, unsigned int i) {

//     // Compute the PID terms
//     double p_term = kp_[i] * error_p;
//     double d_term = kd_[i] * error_d;
//     double ff_term = kff_[i] * feed_forward_ref;

//     // Compute the output and saturate it
//     double output = p_term + d_term + ff_term;      //add the integral term later
//     double saturated_ouput = std::max(min_output_, std::min(output, max_output_));

//     // // Update the statistics structure used for extracting the performance of the control loop
//     // stats_.dt = dt;
//     // stats_.error_p = error_p;
//     // stats_.error_d = error_d;
//     // stats_.integral = error_i_;
//     // stats_.ff_ref = feed_forward_ref;
//     // stats_.p_term = p_term;
//     // stats_.d_term = d_term;
//     // stats_.i_term = i_term;
//     // stats_.ff_term = ff_term;
//     // stats_.output_pre_sat = output;
//     // stats_.output = saturated_ouput;

//     return saturated_ouput;
// }

// Funtion to compute the attitude target to be sent to the controabs(ller based on a sinusoidal
Eigen::Vector3d SystemIdentification::compute_attitude(double t) {

    // Initialize the unit vector
    Eigen::Vector3d attitude = {0.0, 0.0, 0.0};       // Set the z component to 1.0 (to ensure hover mode, cos(~=0) = 1)

    // Calculate the sinusoidal attitude for the specified axis
    // double sinValue = std::sin(2*M_PI*this->frequency * t);

    // Original: attitude[this->axis] = sinValue;
    // attitude[0] = 0.173 * this->axis[0] * sinValue ;
    attitude[0] = this->amplitude * this->axis[0];
    attitude[1] = this->amplitude * this->axis[1];
    attitude[2] = this->amplitude * this->axis[2];
    return attitude;
}

void SystemIdentification::systemID_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request, const pegasus_msgs::srv::SystemID::Response::SharedPtr response) {
    
    // Set the settings
    this->axis[0] = request->axis[0];
    this->axis[1] = request->axis[1];
    this->axis[2] = request->axis[2];
    this->amplitude = request->amplitude;
    this->t = 0.0;          // Reset the time to 0.0


    // Set the settings flag
    this->settings_set_ = true;

    // Return true to indicate that the settings has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Axis oscilation direction set to (%f, %f, %f) with amplitude %f", this->axis[0], this->axis[1], this->axis[2], this->amplitude);
}



} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::SystemIdentification, autopilot::Mode)