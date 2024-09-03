#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class LimoNode : public rclcpp::Node
{
public:
    LimoNode() : Node("limo_node")
    {
        this->declare_parameter("des_x_loc", 3.0);
        this->declare_parameter("des_y_loc", 3.0);
        this->declare_parameter("Kp_lin_velo", 0.7);
        this->declare_parameter("Kp_ang_velo", 0.7);

        velo_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        timer_ = this->create_wall_timer(0.02s, std::bind(&LimoNode::goToGoal, this));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                       10,
                                                                       std::bind(&LimoNode::poseFeedback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Limo Node has been started");
        RCLCPP_INFO(this->get_logger(),
                    "Reminder: Desired (x, y) location can be set using params des_x_loc and des_y_loc");
    }

private:
    float euclidean_dist(float x1, float x2, float y1, float y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    float angle_to_goal(float x1, float x2, float y1, float y2)
    {
        return atan2(y2 - y1, x2 - x1);
    }

    void adj_angle_limo(geometry_msgs::msg::TwistStamped &velo_command)
    {
        velo_command.twist.linear.x = 0.0;
        velo_command.twist.angular.z = Kp_ang_velo * angle_error;
    }

    void move_limo(geometry_msgs::msg::TwistStamped &velo_command)
    {
        velo_command.twist.angular.z = 0.0;
        velo_command.twist.linear.x = Kp_lin_velo * euclidean_dist_val;
    }

    void stop_movement(geometry_msgs::msg::TwistStamped &velo_command)
    {
        velo_command.twist.linear.x = 0.0;
        velo_command.twist.angular.z = 0.0;
    }

    void goToGoal()
    {
        auto velo_pub_msg = geometry_msgs::msg::TwistStamped();

        x2 = static_cast<float>(this->get_parameter("des_x_loc").as_double());
        y2 = static_cast<float>(this->get_parameter("des_y_loc").as_double());
        Kp_lin_velo = static_cast<float>(this->get_parameter("Kp_lin_velo").as_double());
        Kp_ang_velo = static_cast<float>(this->get_parameter("Kp_ang_velo").as_double());

        angle_to_goal_val = this->angle_to_goal(x1, x2, y1, y2);
        // RCLCPP_INFO(this->get_logger(), "Angle to Goal: '%f'", angle_to_goal_val); // for debugging

        // errors for proportional control
        euclidean_dist_val = this->euclidean_dist(x1, x2, y1, y2);
        angle_error = angle_to_goal_val - theta;
        // RCLCPP_INFO(this->get_logger(), "Angle Error: '%f'", angle_error); // for debugging

        // tolerances
        dist_tol = 0.05; // m
        angle_tol = 0.1; // rad

        // First, start by adjusting angle
        if (abs(angle_error) > angle_tol)
        {
            this->adj_angle_limo(velo_pub_msg);
        }
        else
        {
            // Then, move to the goal
            if (euclidean_dist_val > dist_tol)
            {
                this->move_limo(velo_pub_msg);
            }
            else
            {
                this->stop_movement(velo_pub_msg);
                RCLCPP_INFO(this->get_logger(), "Desired loc (%.2f, %.2f) has been reached", x2, y2);
            }
        }

        velo_publisher_->publish(velo_pub_msg);
    }

    void poseFeedback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x1 = msg->pose.pose.position.x;
        y1 = msg->pose.pose.position.y;
        theta = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // RCLCPP_INFO(this->get_logger(), "Theta: '%f'", theta); // for debugging


        // // OLD/EXTRA
        // RCLCPP_INFO(this->get_logger(), "Position-> x: '%f', y: '%f', z: '%f'",
        //             x1,
        //             y1,
        //             msg->pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "Orientation-> x: '%f', y: '%f', z: '%f', w: '%f'",
        //             msg->pose.pose.orientation.x,
        //             msg->pose.pose.orientation.y,
        //             msg->pose.pose.orientation.z,
        //             msg->pose.pose.orientation.w);
        // RCLCPP_INFO(this->get_logger(), "Vel-> Linear Velo: '%f', Theta dot: '%f'",
        //             msg->twist.twist.linear.x,
        //             msg->twist.twist.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velo_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float x1, y1, x2, y2, theta, euclidean_dist_val, dist_tol, 
            angle_tol, angle_to_goal_val, angle_error, Kp_lin_velo, Kp_ang_velo;
};


int main(int argc, char **argv)
{
    rclcpp::sleep_for(10s); // gives time for Gazebo sim to load and then robot moves
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
