/* Hero Turtle!
*/


#include <cmath>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/hero_telem.hpp"
#include "my_robot_interfaces/msg/villain_telem.hpp"

#include "turtlechasehwk/utils.hpp"

using namespace turtlechasehwk;
using std::placeholders::_1;
using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;
using turtlesim::srv::Kill;
using my_robot_interfaces::msg::HeroTelem;
using my_robot_interfaces::msg::Turtle;
using my_robot_interfaces::msg::VillainTelem;


double distanceBetween(Pose &a, Pose &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}


class HeroTurtleNode : public rclcpp::Node
{
public:
    HeroTurtleNode() : Node("hero_turtle"), need_new_villain_(true), terminate_(false), kill_queue_()
    {
        declare_parameter<std::string>("hero_name", "turtle1");
        declare_parameter<double>("controller_freq_hz", 10.0);
        declare_parameter<double>("capture_dist", 0.25);
        declare_parameter<double>("control/pos/P", 1.0);
        declare_parameter<double>("control/pos/min", 0.1);
        declare_parameter<double>("control/pos/max", 5.0);
        declare_parameter<double>("control/ang/P", 5.0);
        declare_parameter<double>("control/ang/min", -100.0);
        declare_parameter<double>("control/ang/max", 100.0);

        kill_thread_ = std::thread(std::bind(&HeroTurtleNode::killThreadLoop, this));

        // Controller Setup
        std::string hero_name = get_parameter("hero_name").as_string();
        std::string input_topic = std::string("/") + hero_name + std::string("/pose");
        std::string output_topic = std::string("/") + hero_name + std::string("/cmd_vel");
        pose_subscription_ = create_subscription<Pose>(
            input_topic, 10, std::bind(&HeroTurtleNode::poseSubscriptionCallback, this, _1)
        );
        control_out_publisher_ = create_publisher<Twist>(output_topic, 10);
        int control_period_ms = freqToPeriodMs(get_parameter("controller_freq_hz").as_double());
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(control_period_ms), std::bind(&HeroTurtleNode::controllerTimerCallback, this)
        );
        RCLCPP_INFO(get_logger(), "Reading from %s", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "Writing to %s", output_topic.c_str());

        telem_publisher_ = create_publisher<HeroTelem>("hero_telem", 10);
        villain_subscription_ = create_subscription<VillainTelem>(
            "villain_telem", 10, std::bind(&HeroTurtleNode::villainSubscriptionCallback, this, _1)
        );
        RCLCPP_INFO(get_logger(), "Starting Hero!");
    }

    ~HeroTurtleNode()
    {
        RCLCPP_DEBUG(get_logger(), "Closing Thread");
        terminate_ = true;
        kill_condition_.notify_all();
        kill_thread_.detach();
    }

private:

    void villainSubscriptionCallback(const VillainTelem::SharedPtr villain_telem)
    {
        if (!need_new_villain_) {
            return;
        }
        double min_dist = INFINITY;
        for (auto villain : villain_telem->villains) {
            auto cur_dist = distanceBetween(villain.pose, cur_telem_.pose);
            if (cur_dist < min_dist) {
                cur_telem_.target = villain;
                need_new_villain_ = false;
            }
        }
        if (!need_new_villain_) {
            RCLCPP_INFO(get_logger(), "Found New Villain: %s", cur_telem_.target.name.c_str());
        } else {
            RCLCPP_DEBUG(get_logger(), "Still searching for the one...");
        }
    }

    void controllerTimerCallback()
    {
        // Convert telem state to eigen
        Eigen::Vector3d pos_robot_in_field(cur_telem_.pose.x, cur_telem_.pose.y, 0.0);
        Eigen::Vector3d pos_target_in_field(cur_telem_.target.pose.x, cur_telem_.target.pose.y, 0.0);
        auto heading = cur_telem_.pose.theta;
        Eigen::Vector3d dir_heading_in_field(std::cos(heading), std::sin(heading), 0.0);  // Boy I hope this is right

        // Compute error terms
        auto pos_error_target_from_robot_in_field = pos_target_in_field - pos_robot_in_field;
        cur_telem_.pos_error = pos_error_target_from_robot_in_field.norm();
        auto dir_target_from_robot_in_field = pos_error_target_from_robot_in_field.normalized();
        // Note turning left is positive error
        auto angle_error_in_field = dir_heading_in_field.cross(dir_target_from_robot_in_field);
        cur_telem_.theta_error = angle_error_in_field(2);

        // Compute controller gains
        auto vel_command = get_parameter("control/pos/P").as_double() * cur_telem_.pos_error;
        vel_command = clipToBounds(
            vel_command, get_parameter("control/pos/min").as_double(), get_parameter("control/pos/max").as_double()
        );
        auto ang_command = get_parameter("control/ang/P").as_double() * cur_telem_.theta_error;
        ang_command = clipToBounds(
            ang_command, get_parameter("control/ang/min").as_double(), get_parameter("control/ang/max").as_double()
        );

        // Publish Results
        auto command = Twist();
        command.linear.x = vel_command;
        command.linear.y = 0.0;
        command.linear.z = 0.0;
        command.angular.x = 0.0;
        command.angular.y = 0.0;
        command.angular.z = ang_command;
        control_out_publisher_->publish(command);

        if (cur_telem_.pos_error < get_parameter("capture_dist").as_double()) {
            auto request = std::make_shared<Kill::Request>();
            request->name = cur_telem_.target.name;
            RCLCPP_INFO(get_logger(), "Got %s!", request->name.c_str());
            queueKillRequest(request);
        }
        telem_publisher_->publish(cur_telem_);
    }

    /**
     * Pose Subscription Callback
     *
     * Did a quick `ros2 topic hz /turtle1/pose` and got about 62.5Hz on my machine.  I don't want to assume the control
     * loop will run at the same rate, but I might create a low level controller that just takes a desired pos/heading
     * and make a high level controller that sets that pos/heading (like a path planner or something), but for now just
     * record the pose.
    */
    void poseSubscriptionCallback(const Pose::SharedPtr pose)
    {
        cur_telem_.pose_timestamp = get_clock()->now();
        cur_telem_.pose.x = pose->x;
        cur_telem_.pose.y = pose->y;
        cur_telem_.pose.theta = pose->theta;
        cur_telem_.pose.angular_velocity = pose->angular_velocity;
        cur_telem_.pose.linear_velocity = pose->linear_velocity;
        RCLCPP_DEBUG(get_logger(), "Got Pose at %d | %d -> (%f, %f, %f, %f, %f)",
            cur_telem_.pose_timestamp.sec,
            cur_telem_.pose_timestamp.nanosec,
            cur_telem_.pose.x,
            cur_telem_.pose.y,
            cur_telem_.pose.theta,
            cur_telem_.pose.angular_velocity,
            cur_telem_.pose.linear_velocity
        );
    }

    void queueKillRequest(Kill::Request::SharedPtr request)
    {
        {
            std::unique_lock<std::mutex> lock(kill_mutex_);
            kill_queue_.push(std::move(request));
        }
        kill_condition_.notify_one();
    }

    void killThreadLoop()
    {
        RCLCPP_INFO(get_logger(), "Staring Kill Thread");
        while (true) {
            Kill::Request::SharedPtr request;
            {
                std::unique_lock<std::mutex> lock(kill_mutex_);
                kill_condition_.wait(lock, [this] {
                    return !kill_queue_.empty() || terminate_;
                });
                if (terminate_) {
                    return;
                }
                request = kill_queue_.front();
                kill_queue_.pop();
            }
            RCLCPP_INFO(get_logger(), "Sending Kill Request: %s", request->name.c_str());
            auto response = sendRequest<Kill, Kill::Request, Kill::Response>(request, "kill_turtle");
            if (!response) {
                RCLCPP_WARN(get_logger(), "Couldn't kill %s, must be invincible", request->name.c_str());
            }
            need_new_villain_ = true;
        }
        RCLCPP_INFO(get_logger(), "Ending Kill Thread");
    }

    template <typename T, typename Request, typename Response>
    std::shared_ptr<Response> sendRequest(const std::shared_ptr<Request> request, const std::string endpoint)
    {
        auto client = create_client<T>(endpoint);
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Waiting for server...");
        }
        auto future = client->async_send_request(request);
        try {
            auto response = future.get();
            return response;
        } catch (const std::exception &e) {
            return nullptr;
        }
    }

    // Mission
    bool need_new_villain_;
    rclcpp::Subscription<VillainTelem>::SharedPtr villain_subscription_;

    // Controller
    rclcpp::Subscription<Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<Twist>::SharedPtr control_out_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // General Telem
    HeroTelem cur_telem_;
    rclcpp::Publisher<HeroTelem>::SharedPtr telem_publisher_;

    // Kill Thread
    bool terminate_;
    std::thread kill_thread_;
    std::mutex kill_mutex_;
    std::condition_variable kill_condition_;
    std::queue<Kill::Request::SharedPtr> kill_queue_;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeroTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
