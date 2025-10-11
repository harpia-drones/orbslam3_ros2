#include <queue>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <orbslam3/System.h>


using namespace std::placeholders;
using namespace std::chrono_literals;



class RgbdInertialSlamNode 
: public rclcpp::Node
{
    public:
    RgbdInertialSlamNode(const std::string& node_name)
    : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "SLAM Node has been started in RGBD-Inertial mode.");

        // =================================================
        //     PARAMETERS
        // =================================================

        this->declare_parameter<std::string>("orb_voc_path");
        this->declare_parameter<std::string>("settings_path");
        this->declare_parameter<int>("camera_fps");
        this->declare_parameter<bool>("visualization");

        std::string orb_voc_path_ = this->get_parameter("orb_voc_path").as_string();
        std::string settings_path_ = this->get_parameter("settings_path").as_string();
        int camera_fps = this->get_parameter("camera_fps").as_int();
        bool visualization = this->get_parameter("visualization").as_bool();

        // =================================================
        //     ORB SLAM3 SYSTEM
        // =================================================

        SLAM = std::make_unique<ORB_SLAM3::System>(orb_voc_path_, settings_path_, ORB_SLAM3::System::IMU_RGBD, visualization);
        image_scale = SLAM->GetImageScale();

        // =================================================
        //     SUBSCRIBERS
        // =================================================

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb", 10, std::bind(&RgbdInertialSlamNode::rgb_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth", 10, std::bind(&RgbdInertialSlamNode::depth_callback, this, std::placeholders::_1)
        );
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 100, std::bind(&RgbdInertialSlamNode::imu_callback, this, std::placeholders::_1)
        );

        // =================================================
        //     PUBLISHERS
        // =================================================

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        path_msg_.header.frame_id = "odom";  // Initialize Path

        // =================================================
        //     TIMERS
        // =================================================

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
        RCLCPP_INFO(this->get_logger(), "SLAM threads stopped!");
    }

private:

    // =================================================
    //     ATTRIBUTES
    // =================================================

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
    
    // mutexes
    std::mutex rgb_mtx;
    std::mutex depth_mtx;
    std::mutex imu_mtx;

    // Image callback
    cv::Mat gray_buffer_;
    cv::Mat depth_buffer_;
    rclcpp::Time timestamp_buffer_;

    // Imu callback
    std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;


    // =================================================
    //     METHODS
    // =================================================

    
    /**
    * @brief Callback to process incoming RGB images from a ROS topic.
    *
    * Converts the received image to grayscale, optionally resizes it,
    * and stores the result in a shared buffer protected by a mutex,
    * along with the message timestamp.
    *
    * @param msg Pointer to the received image message (sensor_msgs::msg::Image).
    */
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            // Get timestamp
            rclcpp::Time timestamp = msg->header.stamp;   

            // Get rgb frame
            cv_bridge::CvImagePtr cv_ptr;                 
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat color_frame = cv_ptr->image;  
            
            // Convert from bgr to gray scale
            cv::Mat gray_frame;

            if (color_frame.channels() == 3)
            {
                cv::cvtColor(color_frame, gray_frame, cv::COLOR_BGR2GRAY);
            } 
            else if (color_frame.channels() == 1)
            {
                gray_frame = color_frame.clone();
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Frame channel is neither 3 nor 1.");
                return;
            }
            
            // Resize image
            if (image_scale != 1.f)
            {
                int width = static_cast<int>(gray_frame.cols * image_scale);
                int height = static_cast<int>(gray_frame.rows * image_scale);
                cv::resize(gray_frame, gray_frame, cv::Size(width, height));
            }
            
            {
                std::lock_guard<std::mutex> lock(rgb_mtx);
                
                gray_buffer_ = gray_frame.clone();
                timestamp_buffer_ = timestamp;
            }
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }


    /**
    * @brief Callback to process incoming depth images from a ROS topic.
    *
    * Converts the received depth image into an OpenCV matrix with 16-bit
    * unsigned single-channel format (TYPE_16UC1) and stores it in a
    * shared buffer protected by a mutex.
    *
    * @param msg Pointer to the received depth image message (sensor_msgs::msg::Image).
    *
    * @throws cv_bridge::Exception If the image conversion fails.
    */
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {        
        try 
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat depth_frame = cv_ptr->image;

            {
                std::lock_guard<std::mutex> lock(depth_mtx);
                depth_buffer_ = depth_frame.clone();
            }
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_INFO(this->get_logger(), "erro no cv_bridge: %s", e.what());
        }
    }


    /**
    * @brief Callback to process incoming IMU measurements from a ROS topic.
    *
    * Extracts linear acceleration and angular velocity from the IMU message,
    * converts them into OpenCV 3D points, and packages them into an
    * ORB_SLAM3::IMU::Point structure with a timestamp. The result is then
    * appended to a shared IMU buffer protected by a mutex.
    *
    * @param msg Pointer to the received IMU message (sensor_msgs::msg::Imu).
    */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        rclcpp::Time header_stamp = msg->header.stamp;
        double timestamp = header_stamp.seconds() + header_stamp.nanoseconds() * 1e-9;
        
        cv::Point3f Acc = cv::Point3f(
            msg->linear_acceleration.x, 
            msg->linear_acceleration.y, 
            msg->linear_acceleration.z
        );

        cv::Point3f Gyro= cv::Point3f(
            msg->angular_velocity.x, 
            msg->angular_velocity.y, 
            msg->angular_velocity.z
        );
        
        ORB_SLAM3::IMU::Point imu_data(Acc, Gyro, timestamp);

        {
            std::lock_guard<std::mutex> lock(imu_mtx);
            imu_buffer_.push_back(imu_data);
        }
    }

    /**
    * @brief Main SLAM processing loop.
    *
    * Retrieves the latest grayscale, depth, and IMU measurements from their
    * respective buffers (protected by mutexes), clears the buffers, and passes
    * the data to the ORB-SLAM3 tracking function. If the system is not yet fully
    * initialized, it logs a warning; otherwise, it marks the SLAM threads as started
    * and publishes the current pose.
    */
    void slam_loop()
    {
        cv::Mat gray_frame;
        cv::Mat depth_frame;
        rclcpp::Time timestamp;
        double tframe;
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

        {
            std::lock_guard lock(rgb_mtx);

            // Copy data from buffers
            gray_frame = gray_buffer_.clone();
            timestamp = timestamp_buffer_;
            tframe = timestamp.seconds() + timestamp.nanoseconds() * 1e-9;
        }

        {
            std::lock_guard lock(depth_mtx);

            // Copy data from buffers
            depth_frame = depth_buffer_.clone();
        }

        {
            std::lock_guard lock(imu_mtx);

            // Copy data from buffer
            vImuMeas = std::move(imu_buffer_);

            // Clear buffer
            imu_buffer_.clear();
        }
    
        RCLCPP_INFO(this->get_logger(), 
            "Calling TrackRGBD: gray=%dx%d depth=%dx%d imu=%zu t=%.6f",
            gray_frame.cols, gray_frame.rows,
            depth_frame.cols, depth_frame.rows,
            vImuMeas.size(), tframe);


        // Verification
        if (vImuMeas.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "IMU not ready yet, skipping iteration");
            return;
        }
        if (gray_frame.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "RGB frames not ready yet, skipping iteration");
            return;
        }
        if (depth_frame.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "Depth frames not ready yet, skipping iteration");
            return;
        }


        // Submit frame_gray and timestamp to SLAM track
        Sophus::SE3f Tcw = SLAM->TrackRGBD(gray_frame, depth_frame, tframe, vImuMeas);

        // Check if slam is actually running 
        if (Tcw.matrix().isIdentity()) 
        {
            RCLCPP_WARN(this->get_logger(), "Starting the SLAM system... This may take a while.");
            return;
        }
        else
        {
            slam_threads_started = true; // marks that slam is actually running
            RCLCPP_INFO(this->get_logger(), "All SLAM threads has started!");
        }

        get_pose(Tcw, timestamp);
    }


    /**
    * @brief Publishes the current camera pose estimated by SLAM.
    *
    * Converts the transformation matrix from camera-to-world (Tcw) into world-to-camera (Twc),
    * extracts translation and rotation, and publishes the result as a PoseStamped message.
    * Also accumulates the trajectory into a Path message for visualization.
    *
    * @param Tcw Camera-to-world transformation matrix estimated by SLAM.
    * @param time Timestamp associated with the pose.
    */
    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time time)
    {
        // Get inverse of Tcw : Twc
        Sophus::SE3f Twc = Tcw.inverse();

        // Get rotation matriz and translation vector
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

        // Convert to PoseStamped
        auto pose_msg = geometry_msgs::msg::PoseStamped();

        pose_msg.header.stamp = time;
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

        // TODO: Convert o px4_msgs/msg/VehicleOdometry
    }
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdInertialSlamNode>("slam_rgbd_inertial_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}