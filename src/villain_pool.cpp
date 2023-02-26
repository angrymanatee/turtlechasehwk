/* Villain Pool ROS2 Thing
*/


#include <algorithm>
#include <cmath>
#include <queue>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/villain_telem.hpp"
#include "turtlechasehwk/utils.hpp"


using namespace turtlechasehwk;
using std::placeholders::_1;
using std::placeholders::_2;
using turtlesim::srv::Spawn;
using turtlesim::srv::Kill;
using my_robot_interfaces::msg::Turtle;
using my_robot_interfaces::msg::VillainTelem;


class VillainPoolNode : public rclcpp::Node
{
public:
    VillainPoolNode() :  Node("villain_pool"), n_created_(0), n_killed_(0), rng_(), terminate_(false), villain_list_(),
        spawn_queue_(), kill_queue_()
    {
        declare_parameter<double>("spawn_frequency_hz", 1.0);
        declare_parameter<double>("min_x", 0.0);
        declare_parameter<double>("max_x", 10.0);
        declare_parameter<double>("min_y", 0.0);
        declare_parameter<double>("max_y", 10.0);
        declare_parameter<int>("max_villains", 10);
        declare_parameter<int>("random_seed", 0);

        // Create distributions
        auto seed = get_parameter("random_seed").as_int();
        if (seed != 0) {
            rng_.seed(seed);
        } else {
            std::random_device rd;
            rng_.seed(rd());
        }

        telem_publisher_ = create_publisher<VillainTelem>("villain_telem", 10);

        // Setup spawn clock
        int spawn_period_ms = freqToPeriodMs(get_parameter("spawn_frequency_hz").as_double());
        spawn_thread_ = std::thread(std::bind(&VillainPoolNode::spawnThreadLoop, this));
        kill_thread_ = std::thread(std::bind(&VillainPoolNode::killThreadLoop, this));
        kill_service_ = create_service<Kill>(
            "kill_turtle", std::bind(&VillainPoolNode::killServiceCallback, this, _1, _2)
        );
        spawn_timer_ = create_wall_timer(
            std::chrono::milliseconds(spawn_period_ms),  std::bind(&VillainPoolNode::spawnTimerLoop, this)
        );
        RCLCPP_INFO(get_logger(), "VillainPool node started!  Muahahaha!");
    }

    ~VillainPoolNode()
    {
        RCLCPP_DEBUG(get_logger(), "Closing Thread");
        terminate_ = true;
        spawn_condition_.notify_all();
        kill_condition_.notify_all();
        spawn_thread_.detach();
        kill_thread_.detach();
    }

private:

    void spawnTimerLoop()
    {
        int n_villains;
        {
            std::unique_lock<std::mutex> lock(villain_list_mutex_);
            n_villains = villain_list_.size();
        }
        if (n_villains >= get_parameter("max_villains").as_int()) {
            RCLCPP_WARN(get_logger(), "Too many villains (have %d)!  Not spawning.", n_villains);
        } else {
            auto request = std::make_shared<Spawn::Request>();
            std::uniform_real_distribution<> x_dist(
                get_parameter("min_x").as_double(), get_parameter("max_x").as_double());
            std::uniform_real_distribution<> y_dist(
                get_parameter("min_x").as_double(), get_parameter("max_x").as_double());
            std::uniform_real_distribution<> theta_dist(0.0, 2 * M_PIl);
            request->x = x_dist(rng_);
            request->y = y_dist(rng_);
            request->theta = theta_dist(rng_);
            queueSpawnRequest(std::move(request));
        }
        // Publish Telemetry
        publishTelemNow();
    }

    void queueSpawnRequest(Spawn::Request::SharedPtr request)
    {
        RCLCPP_DEBUG(get_logger(), "Queued Spawn Request (queue len=%ld): (%f, %f, %f)",
            spawn_queue_.size(), request->x, request->y, request->theta);
        {
            std::unique_lock<std::mutex> lock(spawn_mutex_);
            spawn_queue_.push(std::move(request));
        }
        spawn_condition_.notify_one();
    }

    void spawnThreadLoop()
    {
        RCLCPP_INFO(get_logger(), "Staring Spawn Thread");
        while (true) {
            Spawn::Request::SharedPtr request;
            {
                std::unique_lock<std::mutex> lock(spawn_mutex_);
                spawn_condition_.wait(lock, [this] {
                    return !spawn_queue_.empty() || terminate_;
                });
                if (terminate_) {
                    return;
                }
                request = spawn_queue_.front();
                spawn_queue_.pop();
            }
            RCLCPP_DEBUG(get_logger(), "Sending Spawn Request: (%f, %f, %f)", request->x, request->y, request->theta);
            auto response = sendRequest<Spawn, Spawn::Request, Spawn::Response>(request, "spawn");
            auto turtle = Turtle();
            turtle.pose.x = request->x;
            turtle.pose.y = request->y;
            turtle.pose.theta = request->theta;
            if (response) {
                turtle.name = response->name;
                RCLCPP_INFO(get_logger(), "Spawned turtle %s (%f, %f, %f)",
                    turtle.name.c_str(), request->x, request->y, request->theta);
                std::unique_lock<std::mutex> lock(villain_list_mutex_);
                villain_list_.push_back(turtle);
            } else {
                RCLCPP_WARN(get_logger(), "Failed to spawn turtle!");
            }
        }
        RCLCPP_INFO(get_logger(), "Ending Spawn Thread");
    }

    void killServiceCallback(const Kill::Request::SharedPtr request, Kill::Response::SharedPtr)
    {
        RCLCPP_INFO(get_logger(), "Received Kill Request for %s", request->name.c_str());
        size_t old_size;
        size_t new_size;
        {
            std::unique_lock<std::mutex> lock(villain_list_mutex_);
            old_size = villain_list_.size();
            auto new_start = std::remove_if(
                villain_list_.begin(),
                villain_list_.end(),
                [&request](Turtle &t){ return t.name == request->name; }
            );
            villain_list_.erase(new_start, villain_list_.end());
            new_size = villain_list_.size();
        }
        if (new_size < old_size) {
            ++n_killed_;
        } else {
            RCLCPP_WARN(get_logger(), "Kill request %s rejected, no turtle by that name", request->name.c_str());
        }
        publishTelemNow();
        queueKillRequest(request);  // Kill loop doesn't touch villain list, do last to avoid delays
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
            auto response = sendRequest<Kill, Kill::Request, Kill::Response>(request, "kill");
            if (!response) {
                RCLCPP_WARN(get_logger(), "Couldn't kill %s, must be invincible", request->name.c_str());
            }
        }
        RCLCPP_INFO(get_logger(), "Ending Kill Thread");
    }

    void publishTelemNow()
    {
        RCLCPP_DEBUG(get_logger(), "Publishing Telem");
        auto villain_telem = VillainTelem();
        villain_telem.created = n_created_;
        villain_telem.killed = n_killed_;
        {
            std::unique_lock<std::mutex> lock(villain_list_mutex_);
            villain_telem.villains.resize(villain_list_.size());
            std::copy(villain_list_.begin(), villain_list_.end(), villain_telem.villains.begin());
        }
        telem_publisher_->publish(villain_telem);
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

    // Telemetry and stats
    int n_created_;
    int n_killed_;
    rclcpp::Publisher<VillainTelem>::SharedPtr telem_publisher_;

    // Spawning Timer and RNGs
    std::mt19937 rng_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::Service<Kill>::SharedPtr kill_service_;

    // Thread Stuff
    bool terminate_;
    // Villain List Stuff
    std::mutex villain_list_mutex_;
    std::vector<Turtle> villain_list_;
    // Spawn Thread Stuff
    std::thread spawn_thread_;
    std::mutex spawn_mutex_;
    std::condition_variable spawn_condition_;
    std::queue<Spawn::Request::SharedPtr> spawn_queue_;
    // Kill Thread Stuff
    std::thread kill_thread_;
    std::mutex kill_mutex_;
    std::condition_variable kill_condition_;
    std::queue<Kill::Request::SharedPtr> kill_queue_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VillainPoolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
