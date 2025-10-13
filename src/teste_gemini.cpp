#include <queue>
#include <mutex>
#include <vector>
#include <thread> // Para a thread de processamento

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <orbslam3/System.h>

using namespace std::chrono_literals;

// Estrutura para facilitar o armazenamento de imagens com timestamp
struct ImageData {
    cv::Mat image;
    rclcpp::Time stamp;
};

class RgbdInertialSlamNode : public rclcpp::Node {
public:
    RgbdInertialSlamNode(const std::string& node_name)
    : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "SLAM Node has been started in RGBD-Inertial mode.");

        this->declare_parameter<std::string>("orb_voc_path", "");
        this->declare_parameter<std::string>("settings_path", "");
        this->declare_parameter<bool>("visualization", true);

        std::string orb_voc_path_ = this->get_parameter("orb_voc_path").as_string();
        std::string settings_path_ = this->get_parameter("settings_path").as_string();
        bool visualization = this->get_parameter("visualization").as_bool();

        SLAM = std::make_unique<ORB_SLAM3::System>(orb_voc_path_, settings_path_, ORB_SLAM3::System::IMU_RGBD, visualization);
        image_scale_ = SLAM->GetImageScale();

        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb", 100, std::bind(&RgbdInertialSlamNode::rgb_callback, this, std::placeholders::_1)
        );
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth", 100, std::bind(&RgbdInertialSlamNode::depth_callback, this, std::placeholders::_1)
        );
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(), std::bind(&RgbdInertialSlamNode::imu_callback, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        path_msg_.header.frame_id = "odom";

        // Inicia a thread de processamento
        processing_thread_ = std::thread(&RgbdInertialSlamNode::sync_and_process, this);
    }

    ~RgbdInertialSlamNode() {
        processing_thread_.join(); // Garante que a thread termine
        SLAM->Shutdown();
        RCLCPP_INFO(this->get_logger(), "SLAM threads stopped!");
    }

private:
    std::unique_ptr<ORB_SLAM3::System> SLAM;
    float image_scale_;
    std::thread processing_thread_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Buffers para sincronização
    std::queue<ImageData> rgb_buffer_;
    std::queue<ImageData> depth_buffer_;
    std::queue<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex buffer_mutex_;

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat gray_frame;
            if (cv_ptr->image.channels() == 3) {
                cv::cvtColor(cv_ptr->image, gray_frame, cv::COLOR_BGR2GRAY);
            } else {
                gray_frame = cv_ptr->image.clone();
            }

            if (image_scale_ != 1.f) {
                cv::resize(gray_frame, gray_frame, cv::Size(), image_scale_, image_scale_);
            }
            
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            rgb_buffer_.push({gray_frame, msg->header.stamp});
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat depth_frame = cv_ptr->image;

            if (depth_frame.type() != CV_32F) {
                depth_frame.convertTo(depth_frame, CV_32F, 1.0 / 1000.0); // Exemplo para depth em mm
            }

            if (image_scale_ != 1.f) {
                cv::resize(depth_frame, depth_frame, cv::Size(), image_scale_, image_scale_, cv::INTER_NEAREST);
            }

            std::lock_guard<std::mutex> lock(buffer_mutex_);
            depth_buffer_.push({depth_frame, msg->header.stamp});
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        
        // CUIDADO: Verifique se essa transformação de eixos é a correta para o seu sensor!
        cv::Point3f Acc(-msg->linear_acceleration.y, msg->linear_acceleration.x, msg->linear_acceleration.z);
        cv::Point3f Gyro(-msg->angular_velocity.y, msg->angular_velocity.x, msg->angular_velocity.z);

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        imu_buffer_.push(ORB_SLAM3::IMU::Point(Acc, Gyro, timestamp));
    }

    void sync_and_process() {
        while (rclcpp::ok()) {
            cv::Mat gray_frame, depth_frame;
            rclcpp::Time frame_stamp;
            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);

                // Sincroniza RGB e Depth (procura o par mais antigo)
                if (!rgb_buffer_.empty() && !depth_buffer_.empty()) {
                    double t_rgb = rclcpp::Time(rgb_buffer_.front().stamp).seconds();
                    double t_depth = rclcpp::Time(depth_buffer_.front().stamp).seconds();
                    
                    // Sincroniza por tempo, descartando frames mais antigos para alinhar os buffers
                    double max_diff = 0.01; // Tolerância de 10ms
                    if (t_rgb > t_depth + max_diff) {
                        depth_buffer_.pop();
                        continue;
                    } else if (t_depth > t_rgb + max_diff) {
                        rgb_buffer_.pop();
                        continue;
                    }

                    // Se sincronizado, pega os dados
                    gray_frame = rgb_buffer_.front().image;
                    depth_frame = depth_buffer_.front().image;
                    frame_stamp = rgb_buffer_.front().stamp;
                    rgb_buffer_.pop();
                    depth_buffer_.pop();

                    // Coleta todas as medições da IMU ANTERIORES ao frame da imagem
                    double frame_time_sec = rclcpp::Time(frame_stamp).seconds();
                    while (!imu_buffer_.empty() && imu_buffer_.front().t <= frame_time_sec) {
                        vImuMeas.push_back(imu_buffer_.front());
                        imu_buffer_.pop();
                    }
                }
            } // Mutex é liberado aqui

            if (!gray_frame.empty() && !depth_frame.empty()) {
                if (vImuMeas.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No IMU data available for this frame, skipping.");
                } else {
                    Sophus::SE3f Tcw = SLAM->TrackRGBD(gray_frame, depth_frame, rclcpp::Time(frame_stamp).seconds(), vImuMeas);
                    if (!Tcw.matrix().isIdentity()) {
                        get_pose(Tcw, frame_stamp);
                    }
                }
            }

            std::this_thread::sleep_for(1ms); // Evita uso de 100% da CPU
        }
    }

    void get_pose(const Sophus::SE3f& Tcw, const rclcpp::Time& time) {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f R = Twc.rotationMatrix();
        Eigen::Vector3f t = Twc.translation();

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

        path_msg_.header.stamp = time;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdInertialSlamNode>("slam_rgbd_inertial_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}