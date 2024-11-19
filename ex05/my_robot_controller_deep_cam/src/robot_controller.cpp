#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <deque>
#include <limits>

class CircularMover : public rclcpp::Node
{
public:
    CircularMover() : Node("circular_mover"), is_obstacle(false), min_distance(1.0), window_size(5), sum(0.0), previous_distance(1.0), outlier_threshold(0.1)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CircularMover::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth/points", 10, std::bind(&CircularMover::point_cloud_callback, this, std::placeholders::_1));

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

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Преобразование PointCloud2 в pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Проверка на наличие препятствий
        float current_distance = 10;
        for (const auto& point : cloud.points)
        {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            {
                // Вычисляем расстояние до точки
                float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                // Игнорируем точки, находящиеся ниже порога 0.5 метра
                if (distance < current_distance && distance >= 0.5)
                {
                    current_distance = distance;
                }
            }
        }

        // Обновление сглаженного значения
        update_distance(current_distance);

        // Устанавливаем флаг наличия препятствия
        is_obstacle = (min_distance < 1.0); // Если есть препятствие ближе 1 м

        // Вывод отладочной информации
        RCLCPP_INFO(this->get_logger(), "Minimum distance to obstacle updated: %f", min_distance);
    }

    void update_distance(float current_distance)
    {
        // Фильтрация выбросов
        if (std::abs(current_distance - previous_distance) > outlier_threshold)
        {
            RCLCPP_WARN(this->get_logger(), "Outlier detected: %f, ignoring.", current_distance);
            return; // Игнорируем выброс
        }

        // Обновление окна для сглаживания
        if (distance_window.size() == window_size) {
            sum -= distance_window.front(); // Удаляем старое значение из суммы
            distance_window.pop_front(); // Удаляем старое значение из окна
        }

        distance_window.push_back(current_distance); // Добавляем новое значение в окно
        sum += current_distance; // Обновляем сумму

        // Вычисляем сглаженное значение
        min_distance = sum / distance_window.size();
        previous_distance = min_distance; // Обновляем предыдущее значение
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool is_obstacle; // Флаг, указывающий на наличие препятствия
    float min_distance; // Минимальное расстояние до препятствия
    std::deque<float> distance_window; // Дек для хранения значений расстояний
    int window_size; // Размер окна для сглаживания
    float sum; // Сумма значений в окне
    float previous_distance; // Предыдущее сглаженное расстояние
    float outlier_threshold; // Порог для определения выбросов
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularMover>());
    rclcpp::shutdown();
    return 0;
}
