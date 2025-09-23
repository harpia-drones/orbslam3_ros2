#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <orbslam3/System.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mutex>
#include <queue>

using namespace std::placeholders;
using namespace std::chrono_literals;


class RgbdInertialSlamNode 
: public rclcpp::Node
{
    public:
    RgbdInertialSlamNode(const std::string& node_name)
    : Node(node_name), last_timestamp(0.0), timestamp(0.00001)
    {
        RCLCPP_INFO(this->get_logger(), "Node do RGBDInertial começou a rodar");

        // Parameters
        this->declare_parameter<std::string>("orb_voc_path");
        this->declare_parameter<std::string>("settings_path");
        this->declare_parameter<int>("camera_fps");

        std::string orb_voc_path_ = this->get_parameter("orb_voc_path").as_string();
        std::string settings_path_ = this->get_parameter("settings_path").as_string();
        int camera_fps = this->get_parameter("camera_fps").as_int();

        // ORB_SLAM3
        SLAM = std::make_unique<ORB_SLAM3::System>(orb_voc_path_, settings_path_, ORB_SLAM3::System::IMU_RGBD, true);
        image_scale = SLAM->GetImageScale();

        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb", 10, std::bind(&RgbdInertialSlamNode::rgb_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth", 10, std::bind(&RgbdInertialSlamNode::depth_callback, this, std::placeholders::_1)
        );
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 100, std::bind(&RgbdInertialSlamNode::imu_callback, this, std::placeholders::_1)
        );


        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        path_msg_.header.frame_id = "odom";  // Initialize Path

        // Timer to loop
        period_ = std::chrono::milliseconds(1000/camera_fps);
        timer_ = this->create_wall_timer(
            period_,
            [this]() {
                slam_loop();
            }
        );
    }

    ~RgbdInertialSlamNode() 
    {
        SLAM->Shutdown();
        RCLCPP_INFO(this->get_logger(), "Node do RGBDInertial parou de rodar");
    }

private:
    // Timer
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Path path_msg_; // path message

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_; 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_; 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; 
    
    // SLAM
    std::unique_ptr<ORB_SLAM3::System> SLAM;
    float image_scale;
    bool slam_threads_started = false;
    
    std::mutex mutex_;
    double timestamp, last_timestamp;
    rclcpp::Time header_timestamp; 
    cv::Mat rgb_frame, depth_frame;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    // Os timestamps são puxados somente do rgb
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        cv_bridge::CvImagePtr rgb_cv_ptr;
        try {
            last_timestamp = timestamp;
            rgb_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rgb_frame = rgb_cv_ptr->image;
            header_timestamp = msg->header.stamp;

            timestamp = header_timestamp.seconds();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_INFO(this->get_logger(), "erro no cv_bridge: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        cv_bridge::CvImagePtr depth_cv_ptr;
        try {
            depth_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_frame = depth_cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_INFO(this->get_logger(), "erro no cv_bridge: %s", e.what());
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        imu_buffer_.push(msg);
    }


    void slam_loop() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (rgb_frame.empty() || depth_frame.empty() || imu_buffer_.empty()) {
            return;
        }

        std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
        imu_msgs = imu_buffer_;
        imu_buffer_ = std::queue<sensor_msgs::msg::Imu::SharedPtr>();

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        while(!imu_msgs.empty()) {
            auto imu_msg = imu_msgs.front();
            imu_msgs.pop();
            double imu_timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
            vImuMeas.emplace_back(-imu_msg->linear_acceleration.y, -imu_msg->linear_acceleration.z, imu_msg->linear_acceleration.x,
                                  -imu_msg->angular_velocity.y, -imu_msg->angular_velocity.z, imu_msg->angular_velocity.x,
                                  imu_timestamp);
        }

        if (image_scale != 1.f) {
            int width = rgb_frame.cols * image_scale;
            int height = rgb_frame.rows * image_scale;
            cv::resize(rgb_frame, rgb_frame, cv::Size(width, height));
            cv::resize(depth_frame, depth_frame, cv::Size(width, height));
        }

        Sophus::SE3f Tcw = SLAM->TrackRGBD(rgb_frame, depth_frame, timestamp, vImuMeas);


        if (Tcw.matrix().isIdentity()) {
            RCLCPP_WARN(this->get_logger(), "Inicializando o sistema de SLAM...");
            return;
        } else {
            slam_threads_started = true; // marks that slam is actually running
            RCLCPP_INFO(this->get_logger(), "SLAM iniciado");
        }

        if (slam_threads_started) {
            get_pose(Tcw, header_timestamp);
        }
    }

    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time time)
    {
        // Get inverse of Tcw : Twc
        Sophus::SE3f Twc = Tcw.inverse();

        // Get rotation matriz and translation vector
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

        // Convert to PoseStamped
        auto pose_msg = geometry_msgs::msg::PoseStamped();

        pose_msg.header.stamp = header_timestamp;
        pose_msg.header.frame_id = "odom";

        pose_msg.pose.position.x = t(0);
        pose_msg.pose.position.y = t(1);
        pose_msg.pose.position.z = t(2);

        Eigen::Quaternionf q(R);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub_->publish(pose_msg);

        // Acumulate trajectory
        path_msg_.header.stamp = time;
        path_msg_.poses.push_back(pose_msg);
        
        path_pub_->publish(path_msg_);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdInertialSlamNode>("slam_rgbd_inertial_loop");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
