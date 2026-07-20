/*
 * File: fk_subscriber.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2026-07-16
 * Last Modified: 2026-07-16
 *
 * Description: Test node that subscribes to /fk_pose (forward kinematics result),
 *              logs the received poses, and saves them to a CSV file on shutdown.
 *              Use scripts/plot_fk.py to visualize the saved data.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

class FkSubscriber : public rclcpp::Node
{
public:
    FkSubscriber()
        : Node("fk_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fk_pose", 10,
            std::bind(&FkSubscriber::fkCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "FK Subscriber started. Listening on /fk_pose. "
            "Data will be saved to /tmp/fk_poses.csv on shutdown (Ctrl-C).");
    }

    ~FkSubscriber() override
    {
        saveToCsv();
    }

private:
    void fkCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        PoseRecord record;
        record.sec  = msg->header.stamp.sec;
        record.nsec = msg->header.stamp.nanosec;
        record.x  = msg->pose.position.x;
        record.y  = msg->pose.position.y;
        record.z  = msg->pose.position.z;
        record.qx = msg->pose.orientation.x;
        record.qy = msg->pose.orientation.y;
        record.qz = msg->pose.orientation.z;
        record.qw = msg->pose.orientation.w;

        records_.push_back(record);

        RCLCPP_INFO(this->get_logger(),
            "[%zu] FK pose: pos=(%.4f, %.4f, %.4f)  ori=(%.4f, %.4f, %.4f, %.4f)",
            records_.size(),
            record.x, record.y, record.z,
            record.qx, record.qy, record.qz, record.qw);
    }

    void saveToCsv()
    {
        if (records_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "No FK data received — nothing to save.");
            return;
        }

        const std::string path = "/tmp/fk_poses.csv";
        std::ofstream out(path);
        if (!out.is_open()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open %s for writing.", path.c_str());
            return;
        }

        // Write CSV header
        out << "sec,nsec,x,y,z,qx,qy,qz,qw\n";
        out << std::fixed << std::setprecision(6);

        for (const auto &r : records_) {
            out << r.sec  << ',' << r.nsec << ','
                << r.x   << ',' << r.y    << ',' << r.z << ','
                << r.qx  << ',' << r.qy   << ',' << r.qz << ',' << r.qw << '\n';
        }

        out.close();
        RCLCPP_INFO(this->get_logger(),
            "Saved %zu FK poses to %s", records_.size(), path.c_str());
    }

    struct PoseRecord
    {
        int32_t sec;
        uint32_t nsec;
        double x, y, z;
        double qx, qy, qz, qw;
    };

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::vector<PoseRecord> records_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FkSubscriber>());
    rclcpp::shutdown();
    return 0;
}
