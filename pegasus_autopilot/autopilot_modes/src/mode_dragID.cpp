#include "autopilot_modes/mode_dragID.hpp"
#include <pegasus_utils/rotations.hpp>


namespace autopilot {

DragIdentification::~DragIdentification() {
    // Terminate the waypoint service
    this->dragID_service_.reset();
}

void DragIdentification::initialize() {

    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.DragIdentification.set_settings_service", "set_settings");
    this->dragID_service_ = this->node_->create_service<pegasus_msgs::srv::SystemID>(node_->get_parameter("autopilot.SystemIdentification.set_settings_service").as_string(), std::bind(&DragIdentification::dragID_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->node_->get_logger(), "DragIdentification initialized");
    velocity_publisher_ = this->node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/velocity", 10);
    drag_publisher_ = this->node_->create_publisher<std_msgs::msg::Float64>("debug/drag", 10);
    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    this->mass_ = vehicle_constansts.mass;
}

bool DragIdentification::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->settings_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Settings not set - cannot enter Drag Identification mode.");
        return false;
    }

    // Reset the settings flag (to make sure we do not enter twice in this mode without setting new settings)
    this->settings_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool DragIdentification::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

// Place inside your SystemIdentification class implementation file

void DragIdentification::update(double dt) {
    // get current state (same as your original)
    State state = get_vehicle_state();

    // publish the velocity each update as Vector3Stamped
    geometry_msgs::msg::Vector3Stamped vel_msg;
    vel_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    vel_msg.header.frame_id = "base_link"; // or "odom" if velocities are in odom frame
    vel_msg.vector.x = state.velocity[0];
    vel_msg.vector.y = state.velocity[1];
    vel_msg.vector.z = state.velocity[2];
    velocity_publisher_->publish(vel_msg);

    // keep a small time series buffer of velocities for identification
    // (we assume you have std::deque<double> vel_buffer_; std::deque<double> time_buffer_; and size_t buffer_size_ defined in the class)
    double v_forward = state.velocity[0]; // forward axis (assumes yaw = 0). Change index if forward is another axis.
    vel_buffer_.push_back(v_forward);
    time_buffer_.push_back(this->t);
    if (vel_buffer_.size() > buffer_size_) {
        vel_buffer_.pop_front();
        time_buffer_.pop_front();
    }

    // Phase schedule:
    //  - t < 2   : hover (zero attitude)
    //  - 2 <= t < 2 + accel_duration : apply fixed attitude (pitch) to accelerate
    //  - else    : return to hover / hold
    const double hover_time = 2.0;
    const double accel_duration = 30.0; // how long we push the fixed attitude (tunable)
    const double pitch_deg = this->pitch_angle_;     // fixed pitch angle to produce forward thrust (degrees)
    const double roll_deg  = this->roll_angle_;
    const double yaw_deg   = this->yaw_angle_;

    if (t < hover_time) {
        // hover attitude (same as your original first phase)
        this->controller_->set_position({state.position[0], state.position[1], -1.5}, 0.0, dt);

        // (optionally) publish a zero velocity or other debug info if wanted
    }
    else if (t < hover_time + accel_duration) {
        // ---- APPLY FIXED ATTITUDE TO FORCE HORIZONTAL ACCELERATION ----
        Eigen::Vector3d attitude_target(roll_deg, pitch_deg, yaw_deg);

        // compute thrust magnitude T that maintains altitude (vertical balance)
        // vertical component of thrust ≈ T * cos(roll) * cos(pitch)
        double roll_rad  = roll_deg  * M_PI / 180.0;
        double pitch_rad = pitch_deg * M_PI / 180.0;
        double cos_term = std::cos(roll_rad) * std::cos(pitch_rad);

        float velocity_error = state.velocity[2] - 0.0;
        double Kd = 2.0;
        double Kp = 4.0;
        float height_error = state.position[2] - (-1.5);
        if (std::abs(cos_term) < 1e-6) cos_term = 1e-6; // guard

        // T chosen so vertical thrust balances weight (mass_ * g)
        double T = (Kp * height_error + Kd * velocity_error + 9.81 )* this->mass_/ std::abs(cos_term);
        // double T = (this->mass_ * 9.81) / std::abs(cos_term);


        // send attitude + thrust to controller (same signature as your original)
        this->controller_->set_attitude(attitude_target, T, dt);

        // Now attempt to estimate drag coefficient online when the velocity seems steady:
        // We'll check velocity change over the buffer. If slope is low, treat it as steady-state.
        if (vel_buffer_.size() == buffer_size_) {
            // compute simple linear slope over buffer (least-squares or simple delta)
            // simple slope: (v_last - v_first) / (t_last - t_first)
            double v_first = vel_buffer_.front();
            double v_last  = vel_buffer_.back();
            double t_first = time_buffer_.front();
            double t_last  = time_buffer_.back();
            double dt_window = t_last - t_first;
            double slope = 0.0;
            if (dt_window > 1e-6) slope = (v_last - v_first) / dt_window;

            // threshold for 'steady' (tunable)
            const double slope_threshold = 0.01; // m/s^2 approx (if slope smaller than this, consider steady)
            if (std::abs(slope) < slope_threshold && std::abs(v_last) > 0.05) {
                // compute v_ss as average of buffer
                double v_sum = 0.0;
                for (double vv : vel_buffer_) v_sum += vv;
                double v_ss = v_sum / static_cast<double>(vel_buffer_.size());

                // horizontal thrust acceleration component (per mass)
                // a_thrust = (T / mass) * sin(pitch)
                double a_thrust = (T / this->mass_) * std::sin(pitch_rad);

                // estimate quadratic drag coefficient: c_d = a_thrust / v_ss^2
                double c_d_est = 0.0;
                if (std::abs(v_ss) > 1e-6) {
                    c_d_est = a_thrust / (v_ss * v_ss);
                } else {
                    c_d_est = 0.0; // can't estimate with near-zero steady velocity
                }

                // Store/publish/log the estimated drag coefficient
                // assume you have rclcpp logger and optional publisher drag_publisher_ of std_msgs::msg::Float64
                RCLCPP_INFO(this->node_->get_logger(),"Drag identification: v_ss=%.3f m/s, a_thrust=%.3f m/s^2, c_d_est=%.6f", v_ss, a_thrust, c_d_est);

                if (drag_publisher_) {
                    std_msgs::msg::Float64 msg;
                    msg.data = c_d_est;
                    drag_publisher_->publish(msg);
                }

                // Optionally, once a good estimate found, you may stop pushing attitude or
                // reduce accel_duration/mark finished. Here we just log the estimate,
                // but do not automatically stop the experiment.
            }
        }
    }
    else {
        // return to hover / hold position at current horizontal location and -1.5 m altitude
        Eigen::Vector3d stop_pos = state.position;
        this->controller_->set_position({stop_pos[0], stop_pos[1], -1.5}, 0.0, dt);
    }

    // increment time (same as your original)
    this->t += dt;
}


// Funtion to compute the attitude target to be sent to the controller based on a sinusoidal
Eigen::Vector3d DragIdentification::compute_attitude(double t) {

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

void DragIdentification::dragID_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request, const pegasus_msgs::srv::SystemID::Response::SharedPtr response) {
    
    // Set the settings
    this->pitch_angle_ = request->axis[0];
    this->roll_angle_ = request->axis[1];
    this->yaw_angle_ = request->axis[2];
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
PLUGINLIB_EXPORT_CLASS(autopilot::DragIdentification, autopilot::Mode)