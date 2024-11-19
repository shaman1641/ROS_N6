#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <algorithm>

class CircularMover : public rclcpp::Node
{
public:
    CircularMover() : Node("circular_mover"), is_obstacle(false), min_distance(1.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CircularMover::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "robot/scan", 10, std::bind(&CircularMover::laser_callback, this, std::placeholders::_1));

        publish_tf();
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        // Проверка на наличие препятствий
        if (is_obstacle)
        {
            // Остановка робота
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stopping robot.");
        }
        else
        {
            // Движение по кругу
            msg.linear.x = 0.2;  // Скорость вперед
            msg.angular.z = 0.2;  // Угловая скорость
            RCLCPP_INFO(this->get_logger(), "Moving in a circle.");
        }
        
        publisher_->publish(msg);
        
        // Вывод текущего минимального расстояния до препятствия
        RCLCPP_INFO(this->get_logger(), "Current minimum distance to obstacle: %f", min_distance);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Проверка на наличие препятствий
        min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        is_obstacle = (min_distance < 0.5); // Если есть препятствие ближе 0.5 м

        // Вывод отладочной информации
        RCLCPP_INFO(this->get_logger(), "Minimum distance to obstacle updated: %f", min_distance);
    
        // Вывод всех расстояний
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Range[%zu]: %f", i, msg->ranges[i]);
        }
    }


    void publish_tf()
    {
        // Публикация трансформации для робота
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "robot";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        // Создание кватерниона для начальной ориентации
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = 1.0; // Начальная ориентация (без поворота)
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = 0.0;

        transformStamped.transform.rotation = quaternion;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool is_obstacle; // Флаг, указывающий на наличие препятствия
    float min_distance; // Минимальное расстояние до препятствия
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularMover>());
    rclcpp::shutdown();
    return 0;
}
