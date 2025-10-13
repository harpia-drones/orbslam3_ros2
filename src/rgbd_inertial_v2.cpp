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
        RCLCPP_INFO(this->get_logger(), "Inicializando node do RGBDInertialV2");

        // Parameters
        this->declare_parameter<std::string>("orb_voc_path");
        this->declare_parameter<std::string>("settings_path");
        this->declare_parameter<int>("camera_fps");

        std::string orb_voc_path_ = this->get_parameter("orb_voc_path").as_string();
        std::string settings_path_ = this->get_parameter("settings_path").as_string();
        int camera_fps = this->get_parameter("camera_fps").as_int();

        for(int i=0; i < camera_fps; i++) {
            pos_buffer_.push(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            time_buffer_.push(0.0);
        }

        // ORB_SLAM3
        SLAM = std::make_unique<ORB_SLAM3::System>(orb_voc_path_, settings_path_, ORB_SLAM3::System::IMU_RGBD, false);
        image_scale = SLAM->GetImageScale();

        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb", 10, std::bind(&RgbdInertialSlamNode::rgb_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth", 10, std::bind(&RgbdInertialSlamNode::depth_callback, this, std::placeholders::_1)
        );
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::SensorDataQoS(), std::bind(&RgbdInertialSlamNode::imu_callback, this, std::placeholders::_1)
        );


        // Publisher
        vo_pub_ = this->create_publisher<VehicleOdometry>("/vehicle_odometry", 10);

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
        RCLCPP_INFO(this->get_logger(), "Node do RGBDInertialV2 parou de rodar");
    }

private:
    // Timer
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Publisher
    rclcpp::Publisher<VehicleOdometry>::SharedPtr vo_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_; 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_; 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; 
    
    // SLAM
    std::unique_ptr<ORB_SLAM3::System> SLAM;
    float image_scale;
    
    std::mutex mutex_;
    
    rclcpp::Time header_timestamp; 
    cv::Mat rgb_frame, depth_frame;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    std::queue<Eigen::Vector3f> pos_buffer_;
    std::queue<double> time_buffer_;


    // Os timestamps são puxados somente do rgb
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        cv_bridge::CvImagePtr rgb_cv_ptr;
        try {
            rgb_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rgb_frame = rgb_cv_ptr->image;
            header_timestamp = msg->header.stamp;
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
            /*
            last_published_pose.stamp = this->get_clock()->now();
            pose_pub_->publish(last_published_pose);
            */
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
            vImuMeas.emplace_back(-imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.z,
                                  -imu_msg->angular_velocity.y, imu_msg->angular_velocity.x, imu_msg->angular_velocity.z,
                                  imu_timestamp);
        }

        if (image_scale != 1.f) {
            int width = rgb_frame.cols * image_scale;
            int height = rgb_frame.rows * image_scale;
            cv::resize(rgb_frame, rgb_frame, cv::Size(width, height));
            cv::resize(depth_frame, depth_frame, cv::Size(width, height));
        }

        Sophus::SE3f Tcw = SLAM->TrackRGBD(rgb_frame, depth_frame, header_timestamp.seconds(), vImuMeas);

        /*
        if (Tcw.matrix().isIdentity()) {
            RCLCPP_INFO(this->get_logger(), "O drone está parado...");
        } else {
            RCLCPP_INFO(this->get_logger(), "o drone está andando...");
        }
        */

        if (vImuMeas.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Sem dados da IMU");
            return;
        }

        get_pose(Tcw, header_timestamp, vImuMeas.back());
    }

    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time time, const ORB_SLAM3::IMU::Point ip) {
        // Get inverse of Tcw : Twc
        Sophus::SE3f Twc = Tcw.inverse();

        // Get rotation matriz and translation vector
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

        Eigen::Vector3f fifteen_ago = pos_buffer_.front();
        pos_buffer_.pop();
        pos_buffer_.push(t);

        double fifteen_ago_seconds = time_buffer_.front();
        time_buffer_.pop();
        time_buffer_.push(header_timestamp.seconds());

        double dt = header_timestamp.seconds() - fifteen_ago_seconds;
        Eigen::Vector3f v;
        if (dt - 1.0 > 0.7)
            RCLCPP_WARN(this->get_logger(), "Intervalo estranho para cálculo da velocidade");
        else
            v = (t - fifteen_ago) / dt;


        VehicleOdometry msg;
        msg.timestamp = static_cast<uint64_t>(header_timestamp.nanoseconds() * 1000);
        msg.timestamp_sample = msg.timestamp;
        
        msg.pose_frame = VehicleOdometry::POSE_FRAME_FRD;

        msg.position[0] = t(0);
        msg.position[1] = t(1);
        msg.position[2] = t(2);

        Eigen::Quaternionf q(R);
        msg.q[0] = q.w();
        msg.q[1] = q.x();
        msg.q[2] = q.y();
        msg.q[3] = q.z();

        msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_FRD;
        msg.velocity[0] = v.x();
        msg.velocity[1] = v.y();
        msg.velocity[2] = v.z();

        msg.angular_velocity[0] = ip.w[0];
        msg.angular_velocity[1] = ip.w[1];
        msg.angular_velocity[2] = ip.w[2];

        msg.position_variance[0] = NAN;
        msg.position_variance[1] = NAN;
        msg.position_variance[2] = NAN;

        msg.orientation_variance[0] = NAN;
        msg.orientation_variance[1] = NAN;
        msg.orientation_variance[2] = NAN;

        msg.velocity_variance[0] = NAN;
        msg.velocity_variance[1] = NAN;
        msg.velocity_variance[2] = NAN;

        msg.reset_counter = 0;
        msg.quality = -1;

        vo_pub_->publish(msg);
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdInertialSlamNode>("slam_rgbd_inertial_loop");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}