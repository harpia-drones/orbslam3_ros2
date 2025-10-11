#include <rclcpp/rclcpp.hpp>

#include <orbslam3/System.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <deque>
#include <Eigen/Dense>
#include <sophus/so3.hpp>


using namespace std::placeholders;
using namespace std::chrono_literals;



class MonoCamSlamNode 
: public rclcpp::Node
{
public:
    MonoCamSlamNode(const std::string& node_name)
    : Node(node_name), last_timestamp(0.0), timestamp(0.00001)
    {
        RCLCPP_INFO(this->get_logger(), "SLAM Node has been started!");

        // ========================================
        //     Parameters
        // ========================================

        this->declare_parameter<std::string>("orb_voc_path");
        this->declare_parameter<std::string>("settings_path");
        this->declare_parameter<int>("camera_fps");

        std::string orb_voc_path_ = this->get_parameter("orb_voc_path").as_string();
        std::string settings_path_ = this->get_parameter("settings_path").as_string();
        int camera_fps = this->get_parameter("camera_fps").as_int();

        // ========================================
        //     ORB_SLAM3
        // ========================================

        SLAM = std::make_unique<ORB_SLAM3::System>(orb_voc_path_, settings_path_, ORB_SLAM3::System::MONOCULAR, false);
        image_scale = SLAM->GetImageScale();

        // ========================================
        //     Subscribers
        // ========================================

        color_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("color", 10, std::bind(&MonoCamSlamNode::subcriber_callback, this, _1));

        // ========================================
        //     Publishers
        // ========================================

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        path_msg.header.frame_id = "odom";  // Initialize Path
        
        vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);

        // ========================================
        //     Timers
        // ========================================
        
        period_ = std::chrono::milliseconds(1000/camera_fps);
        timer_ = this->create_wall_timer(
            period_,
            [this]() {
                slam_loop();
            }
        );
    }

    ~MonoCamSlamNode() 
    {
        SLAM->Shutdown();
        RCLCPP_INFO(this->get_logger(), "SLAM threads stopped!");
    }

private:

    // ========================================
    //     Attributes
    // ========================================

    // Timer
    std::chrono::milliseconds period_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;

    // Messages
    nav_msgs::msg::Path path_msg; 

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr color_sub_; 
    
    // SLAM
    std::unique_ptr<ORB_SLAM3::System> SLAM;
    float image_scale;
    bool slam_threads_started = false;
    
    double timestamp, last_timestamp;
    rclcpp::Time header_timestamp; 

    cv::Mat frame, frame_gray;
    int width, height;

    // parâmetros configuráveis
    const size_t COV_WINDOW = 30;        // número de poses para janela (ajuste: 10-100)
    const double VAR_FLOOR = 1e-6;       // mínimo para variâncias

    // ========================================
    //     Methods
    // ========================================

    void subcriber_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
        
        // cv bridge object
        cv_bridge::CvImagePtr cv_ptr;

        try 
        {
            last_timestamp = timestamp;

            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            frame = cv_ptr->image;
            // encoding = cv_ptr->encoding;
            header_timestamp = msg->header.stamp;

            timestamp = header_timestamp.seconds();
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void slam_loop()
    {
        // Check if frame is empty
        if (frame.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Empty frame received.");
            return;
        }

        // Convert from bgr to gray scale
        if (frame.channels() == 3)
        {
            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        } else
        if (frame.channels() == 1)
        {
            frame_gray = frame.clone();
        } else 
        {
            RCLCPP_INFO(this->get_logger(), "Frame channel is neither 3 nor 1.");
            return;
        }

        // Resize image
        if (image_scale != 1.f)
        {
            width = frame_gray.cols * image_scale;
            height = frame_gray.rows * image_scale;
            cv::resize(frame_gray, frame_gray, cv::Size(width, height));
        }

        // Submit frame_gray and timestamp to SLAM track
        Sophus::SE3f Tcw = SLAM->TrackMonocular(frame_gray, timestamp);

        // Check if slam is actually running 
        if (Tcw.matrix().isIdentity()) 
        {
            RCLCPP_WARN(this->get_logger(), "Starting the SLAM system... This may take a while.");
            return;
        }
        else
        {
            slam_threads_started = true; // marks that slam is actually running
        }

        if (slam_threads_started) 
        {
            RCLCPP_INFO(this->get_logger(), "All SLAM threads has started!");
        }

        get_pose(Tcw, header_timestamp);
    }

    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time time)
    {
        // Get inverse of Tcw : Twc
        Sophus::SE3f Twc = Tcw.inverse();

        // Get rotation matriz and translation vector
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

        // Pose Stamped
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

        // Path
        path_msg.header.stamp = time;
        path_msg.poses.push_back(pose_msg);
        path_pub_->publish(path_msg);


        // // Vehicle Odometry
        // auto vo_msg = px4_msgs::msg::VehicleOdometry();

        // vo_msg.timestamp = ...;
        // vo_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD; // ajustar conforme seu frame
        
        // vo_msg.position[0] = t(0); 
        // vo_msg.position[1] = t(1); 
        // vvo_msgo.position[2] = t(2);
        
        // vo_msg.q[0] = q.w(); 
        // vo_msg.q[1] = q.x(); 
        // vo_msg.q[2] = q.y(); 
        // vo_msg.q[3] = q.z();

        // // position_variance e orientation_variance são arrays de 3 floats
        // vo_msg.position_variance[0] = pos_var(0);
        // vo_msg.position_variance[1] = pos_var(1);
        // vo_msg.position_variance[2] = pos_var(2);

        // // orientation_variance: aqui usamos as var da representação so3 (rx,ry,rz)
        // vo_msg.orientation_variance[0] = ori_var(0);
        // vo_msg.orientation_variance[1] = ori_var(1);
        // vo_msg.orientation_variance[2] = ori_var(2);

        // vo_msg.quality = some_quality_value; // 0..100
        // vehicle_odometry_pub_->publish(vo);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonoCamSlamNode>("slam_mono_cam");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}