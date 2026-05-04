#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/through_window.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pegasus_msgs/msg/yaw_less_statistics.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <cmath>
#include <deque>


static constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;

namespace autopilot {

class ThroughWindowMode : public autopilot::Mode {

public:
    ~ThroughWindowMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:
    // Enum for constraint types
    enum ConstraintType { Position, Acceleration, Velocity };

    // Struct to define trajectory constraints
    struct Constraint {
        double time;
        ConstraintType type;
        double value;
    };

    // ========== PREDICTIVE CONTROL ADDITIONS ==========
    
    // Control action structure for queue
    struct ControlAction {
        Eigen::Vector3d a_des;      // Desired acceleration (input to position controller)
        Eigen::Vector3d attitude_rate; // Desired attitude rate (torque equivalent)
        double thrust;              // Thrust magnitude
        
        ControlAction() : 
            a_des(Eigen::Vector3d::Zero()), 
            attitude_rate(Eigen::Vector3d::Zero()), 
            thrust(9.81) {}
        
        ControlAction(const Eigen::Vector3d& a, const Eigen::Vector3d& omega, double T) :
            a_des(a), attitude_rate(omega), thrust(T) {}
    };
    
    // State structure for prediction
    struct PredictedState {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d angular_velocity;
        
        PredictedState() :
            position(Eigen::Vector3d::Zero()),
            velocity(Eigen::Vector3d::Zero()),
            rotation(Eigen::Matrix3d::Identity()),
            angular_velocity(Eigen::Vector3d::Zero()) {}
    };
    
    // ODE function for state prediction
    PredictedState rigid_body_ode(const PredictedState& state, const ControlAction& control, double dt);
    
    // RK4 integration step
    PredictedState rk4_step(const PredictedState& state, const ControlAction& control, double dt);
    
    // Predict state by applying queued controls
    PredictedState predict_state(const State& current_state, double dt_control, int integration_steps);
    
    // Compute control action
    ControlAction compute_control_action(
        const Eigen::Vector3d& position_ref,
        const Eigen::Vector3d& velocity_ref, 
        const Eigen::Vector3d& acceleration_ref,
        const Eigen::Vector3d& jerk_ref,
        const PredictedState& predicted_state,
        double yaw,
        double yaw_rate,
        double dt);
    
    // Convert State to PredictedState
    PredictedState state_to_predicted(const State& state);
    
    // Queue of control actions
    std::deque<ControlAction> queued_control_actions_;
    // Queue of predicted states for visualization
    std::deque<PredictedState> queued_predicted_states_;
    
    // Predictive control parameters
    int K_;                     // Prediction horizon (number of steps)
    double delay_;              // Control delay in seconds
    double dt_control_;         // Control update interval
    bool use_predictive_control_; // Enable/disable predictive control
    
    // Inertia matrix (needed for dynamics prediction)
    Eigen::Matrix3d inertia_;
    
    // Last predicted state for visualization
    PredictedState last_predicted_state_;
    
    // Publishers for predicted states
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr predicted_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr predicted_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr predicted_attitude_pub_;
    
    // ========== END PREDICTIVE CONTROL ADDITIONS ==========


    

    // Helper functions
    Eigen::VectorXd generatePolynomialTrajectory(const std::vector<Constraint>& constraints, int polynomial_order);
    void generatePathMsg();
    void generateTrajectoryMarkers(double dt_marker);
    void publishDroneMarker();
    void setTrajectoryConstraints();
    void debuggingAttitudeRefs(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const State& vehicle_state);
    void debuggingRealAcceleration(const State& vehicle_state, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);
    double compute_output(double pos_error, double vel_error, double external_force, unsigned int i);
    void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, double yaw, double yaw_rate, double dt);



    // Callback for settings service
    void throughWindowCallback(const pegasus_msgs::srv::ThroughWindow::Request::SharedPtr request, const pegasus_msgs::srv::ThroughWindow::Response::SharedPtr response);

    // Polynomial coefficients for flat outputs
    std::vector<Eigen::VectorXd> x_segments;
    std::vector<Eigen::VectorXd> y_segments;
    std::vector<Eigen::VectorXd> z_segments;
    std::vector<double> segment_durations_, segmen_start_times_;
    Eigen::VectorXd x_coeffs_;
    Eigen::VectorXd y_coeffs_;
    Eigen::VectorXd z_coeffs_;
    Eigen::VectorXd yaw_coeffs_;

    // Trajectory parameters
    double window_position[3]{0.0, 0.0, 0.0}; // Window position
    double window_angle{0.0}; // Window angle
    double T_final_{0.0}; // Default trajectory duration
    double t{0.0};        // Internal clock

    State vehicle_state;

    Eigen::Vector3d total_acc;
    const double g = 9.81;
    double u = 0.0;
    const double eps = 1e-9;
    double roll  = 0.0,pitch = 0.0,yaw = 0.0;

    double u_dot = 0.0, theta_dot = 0.0, phi_dot = 0.0, psi_dot = 0.0;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d eulers = Eigen::Vector3d::Zero();
    double psi_real   = 0.0,theta_real =0.0,phi_real   = 0.0;
    double p = 0.0,q = 0.0,r = 0.0;
    double s_phi = 0.0,c_phi = 0.0,s_theta = 0.0,c_theta = 0.0;
    const double SING_TOL = 1e-6;// Guard against cos(theta) near zero (gimbal singularity)
    double qsinr = 0.0;
    

    // Vehicle mass
    double mass_;

    // Gains from parameters
    Eigen::Vector3d kp = Eigen::Vector3d::Zero();
    Eigen::Vector3d kd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ki = Eigen::Vector3d::Zero();
    // Gains for the attitude controller
    Eigen::Matrix3d kr = Eigen::Matrix3d::Zero();
    Eigen::Vector3d min_output = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_output = Eigen::Vector3d::Zero();

    // Check if the waypoint is already set
    bool window_point_set_{false};

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_position_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr drone_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr drag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_no_drag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_rate_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_real_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_rate_real_pub_;
    rclcpp::Publisher<pegasus_msgs::msg::YawLessStatistics>::SharedPtr statistics_pub_{nullptr};
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;

    // ROS service
    rclcpp::Service<pegasus_msgs::srv::ThroughWindow>::SharedPtr window_service_{nullptr};

    // Messages
    geometry_msgs::msg::Vector3Stamped euler_msg;
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::Vector3 acceleration_msg;
    geometry_msgs::msg::Vector3 velocity_msg;
    geometry_msgs::msg::Vector3 position_msg;
    geometry_msgs::msg::Vector3 drag_msg;
    visualization_msgs::msg::Marker marker_msg_;

    pegasus_msgs::msg::YawLessStatistics statistics_msg_;

};

} // namespace autopilot
