#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <orbslam3/System.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <mutex>
#include <queue>
#include <chrono>


using namespace std::placeholders;
using namespace std::chrono_literals;
using px4_msgs::msg::VehicleOdometry;

class RgbdInertialSlamNode 
: public rclcpp::Node
{
    public:
    RgbdInertialSlamNode(const std::string& node_name)
    : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Node do RGBDInertial começou a rodar");

        for(int i=0; i < 15; i++) {
            pos_buffer_.push(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            time_buffer_.push(0.0);
        }

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


        // Publisher
        vo_pub_ = this->create_publisher<VehicleOdometry>("vehicle_odometry", 10);

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
    std::queue<Eigen::Vector3f> pos_buffer_;
    std::queue<double> time_buffer_;

    geometry_msgs::msg::PoseStamped last_published_pose;

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

            last_published_pose.stamp = this->get_clock()->now();
            pose_pub_->publish(last_published_pose);

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

            last_published_pose.stamp = this->get_clock()->now();
            pose_pub_->publish(last_published_pose);

            return;
        } else {
            slam_threads_started = true; // marks that slam is actually running
            RCLCPP_INFO(this->get_logger(), "SLAM iniciado");
        }

        if (slam_threads_started) {
            get_pose(Tcw, header_timestamp, vImuMeas.back().angular_velocity);
        }
    }

    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time time, const geometry_msgs::msg::Vector3 av)
    {
        // Get inverse of Tcw : Twc
        Sophus::SE3f Twc = Tcw.inverse();

        // Get rotation matriz and translation vector
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

        auto fifteen_ago = pos_buffer_.pop();
        pos_buffer_.push(t);

        auto fifteen_ago_seconds = time_buffer_.pop();
        time_buffer_.push(header_timestamp.seconds());

        double dt = header_timestamp.seconds() - fifteen_ago_seconds;
        Eigen::Vector3f v = (t - fifteen_ago) / dt;

        VehicleOdometry msg;
        msg.timestamp = static_cast<uint64_t>( header_timestamp.nanoseconds() / 1000 );;
        msg.timestamp_sample = msg.timestamp
        

        msg.pose_frame = VehicleOdometry::POSE_FRAME_FRD;

        msg.position = {t(0), t(1), t(2)};
        msg.q = {q.w(), q.x(), q.y(), q.z()};

        msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_FRD;
        msg.velocity = {v.x, v.y, v.z};
        msg.angular_velocity = {av.x, av.y, av.z};

        msg.position_variance     = {NaN, NaN, NaN};
        msg.orientation_variance  = {NaN, NaN, NaN};
        msg.velocity_variance     = {NaN, NaN, NaN};

        msg.reset_counter = 0;
        msg.quality = -1;

        vo_pub_->publish(msg);
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
