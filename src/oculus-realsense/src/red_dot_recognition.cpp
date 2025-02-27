#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "wozniak_interfaces/srv/coord.hpp"

class RealsenseImageAnalyzer : public rclcpp::Node
{
public:
    RealsenseImageAnalyzer() : Node("realsense_image_analyzer")
    {
        this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        camera_frame_ = this->get_parameter("camera_frame").as_string();

        this->declare_parameter<std::string>("detected_object_frame", "red_dot");
        detected_object_frame_ = this->get_parameter("detected_object_frame").as_string();

        // Declare and retrieve parameters
        this->declare_parameter<std::string>("camera_name", "camera");
        std::string camera_name = this->get_parameter("camera_name").as_string();

        this->declare_parameter<std::string>("camera_namespace", "camera");
        std::string camera_namespace = this->get_parameter("camera_namespace").as_string();

        // Create subscription to the camera topic
        sub_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
            "/" + camera_namespace + "/" + camera_name + "/color/image_raw",
            10,
            // std::bind(&RealsenseImageAnalyzer::image_callback, this, std::placeholders::_1)
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                image_raw_ = msg;                
            }
        );

        sub_aligned_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/" + camera_namespace + "/" + camera_name + "/aligned_depth_to_color/image_raw",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                aligned_depth_image_ = msg;
            }
        );

        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/" + camera_namespace + "/" + camera_name + "/aligned_depth_to_color/camera_info",
            10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                camera_info_ = msg;
            }
        );

        // Create publisher for the modified image
        pub_image_result = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        pub_image_mask = this->create_publisher<sensor_msgs::msg::Image>("mask_image", 10);

        // Create a transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RealsenseImageAnalyzer::timer_callback, this)
        );

        client = this->create_client<wozniak_interfaces::srv::Coord>("Coord");

        RCLCPP_INFO(this->get_logger(), "RealsenseImageAnalyzer node started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_aligned_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_result;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_mask;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string camera_frame_;
    std::string detected_object_frame_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    sensor_msgs::msg::Image::SharedPtr image_raw_;
    sensor_msgs::msg::Image::SharedPtr aligned_depth_image_;

    rclcpp::Client<wozniak_interfaces::srv::Coord>::SharedPtr client;


    void timer_callback()
    {
        if (image_raw_ == nullptr || aligned_depth_image_ == nullptr || camera_info_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for image and camera info...");
            return;
        }

        // Convert ROS2 Image to OpenCV Mat
        cv::Mat frame = cv_bridge::toCvCopy(image_raw_, "bgr8")->image;

        // Perform image processing
        std::pair<int, int> result = find_red_dot(frame);
        if (result.first == -1 || result.second == -1)
        {
            RCLCPP_WARN(this->get_logger(), "No red dot detected in the image.");
            return;
        }

        // Convert the modified OpenCV Mat back to ROS2 Image and publish
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_image_result->publish(*image_msg);

        geometry_msgs::msg::Vector3::SharedPtr red_dot_position = get_3d_position(result.first, result.second);
        if (!red_dot_position)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to compute the 3D position of the red dot.");
            return;
        }

        // Broadcast the position of the red dot as a transform
        publish_transform(red_dot_position);

        // Get red_dot_position with respect to oculus_link

        // Call Service Client
        call_service(red_dot_position);
    }

    geometry_msgs::msg::Vector3::SharedPtr get_3d_position(int x, int y)
    {
        RCLCPP_INFO(this->get_logger(), "Getting 3D position for pixel coordinates: x=%d, y=%d", x, y);

        if (!camera_info_ || !aligned_depth_image_)
        {
            RCLCPP_ERROR(this->get_logger(), "Camera info or depth image not available.");
            return nullptr;
        }

        double fx = camera_info_->k[0];  // Focal length in x direction
        double fy = camera_info_->k[4];  // Focal length in y direction
        double cx = camera_info_->k[2];  // Principal point in x direction
        double cy = camera_info_->k[5];  // Principal point in y direction

        cv::Mat depth_frame = cv_bridge::toCvCopy(aligned_depth_image_, aligned_depth_image_->encoding)->image;

        // Ensure coordinates are within bounds
        if (x < 0 || x >= depth_frame.cols || y < 0 || y >= depth_frame.rows)
        {
            RCLCPP_ERROR(this->get_logger(), "Pixel coordinates out of bounds: x=%d, y=%d", x, y);
            return nullptr;
        }

        // Get the depth value
        double depth = depth_frame.at<uint16_t>(y, x) * 0.001;  // Use (row, column) indexing
        if (depth == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Depth value is invalid at x=%d, y=%d", x, y);
            return nullptr;
        }

        // Calculate 3D position
        geometry_msgs::msg::Vector3::SharedPtr point(new geometry_msgs::msg::Vector3());
        point->x = (x - cx) * depth / fx;
        point->y = (y - cy) * depth / fy;
        point->z = depth;  // Convert millimeters to meters

        return point;
    }

    std::pair<int, int> find_red_dot(cv::Mat &frame)
    {
        // Convert to HSV for color detection
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Define red color range (adjust as needed)
        cv::Scalar lower_red(0, 120, 70);
        cv::Scalar upper_red(10, 255, 255);

        // Threshold the image to get only red colors
        cv::Mat mask;
        cv::inRange(hsv, lower_red, upper_red, mask);

        auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        this->pub_image_mask->publish(*mask_msg);

        // Detect contours (objects)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int max_area = 0;
        int x = -1;
        int y = -1;
        for (const auto& contour : contours)
        {
            if (cv::contourArea(contour) < max_area)
                continue;

            // Get the center of the contour (using moments)
            max_area = cv::contourArea(contour);
            cv::Moments moments = cv::moments(contour);
            x = static_cast<int>(moments.m10 / moments.m00);
            y = static_cast<int>(moments.m01 / moments.m00);
        }

        if (x != -1 && y != -1)
        {
            cv::circle(frame, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);  // Blue circle
        }

        return std::make_pair(x, y);
    }

    void publish_transform(geometry_msgs::msg::Vector3::SharedPtr position)
    {
        // Create a transform message for the red dot position
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = camera_frame_;  // Camera frame
        transform_stamped.child_frame_id = detected_object_frame_;  // Frame for the red dot

        // Set translation to the red dot position (cx, cy)
        transform_stamped.transform.translation.x = position->x;
        transform_stamped.transform.translation.y = position->y;
        transform_stamped.transform.translation.z = position->z;
        // transform_stamped.transform.translation.x = 0.0;
        // transform_stamped.transform.translation.y = 0.0;
        // transform_stamped.transform.translation.z = 1.0;

        // No rotation (we assume no rotation for simplicity)
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        // Publish the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void call_service(geometry_msgs::msg::Vector3::SharedPtr position)
    {
        auto request = std::make_shared<wozniak_interfaces::srv::Coord::Request>();
        request->x = position->x;
        request->y = position->y;
        request->z = position->z;

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending request...");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f", request->x, request->y, request->z);
        auto result_future = client->async_send_request(request,
        [this](rclcpp::Client<wozniak_interfaces::srv::Coord>::SharedFuture response) {
            if (response.get()->success) {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Success: %d", response.get()->success);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Coord");
            }
        });
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseImageAnalyzer>());
    rclcpp::shutdown();
    return 0;
}


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/msg/point.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <opencv4/opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include "wozniak_interfaces/srv/coord.hpp"

// class RealsenseImageAnalyzer : public rclcpp::Node
// {
// public:
//     RealsenseImageAnalyzer() : Node("realsense_image_analyzer")
//     {
//         this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
//         camera_frame_ = this->get_parameter("camera_frame").as_string();

//         this->declare_parameter<std::string>("detected_object_frame", "red_dot");
//         detected_object_frame_ = this->get_parameter("detected_object_frame").as_string();

//         this->declare_parameter<std::string>("oculus_frame", "oculus_link");
//         oculus_frame_ = this->get_parameter("oculus_frame").as_string();

//         // Declare and retrieve parameters
//         this->declare_parameter<std::string>("camera_name", "camera");
//         std::string camera_name = this->get_parameter("camera_name").as_string();

//         this->declare_parameter<std::string>("camera_namespace", "camera");
//         std::string camera_namespace = this->get_parameter("camera_namespace").as_string();

//         // Create subscription to the camera topic
//         sub_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
//             "/" + camera_namespace + "/" + camera_name + "/color/image_raw",
//             10,
//             // std::bind(&RealsenseImageAnalyzer::image_callback, this, std::placeholders::_1)
//             [this](const sensor_msgs::msg::Image::SharedPtr msg) {
//                 image_raw_ = msg;                
//             }
//         );

//         sub_aligned_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/" + camera_namespace + "/" + camera_name + "/aligned_depth_to_color/image_raw",
//             10,
//             [this](const sensor_msgs::msg::Image::SharedPtr msg) {
//                 aligned_depth_image_ = msg;
//             }
//         );

//         sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
//             "/" + camera_namespace + "/" + camera_name + "/aligned_depth_to_color/camera_info",
//             10,
//             [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
//                 camera_info_ = msg;
//             }
//         );

//         // Create publisher for the modified image
//         pub_image_result = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
//         pub_image_mask = this->create_publisher<sensor_msgs::msg::Image>("mask_image", 10);

//         // Create a transform objects
//         tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&RealsenseImageAnalyzer::timer_callback, this)
//         );

//         client = this->create_client<wozniak_interfaces::srv::Coord>("Coord");

//         RCLCPP_INFO(this->get_logger(), "RealsenseImageAnalyzer node started.");
//     }

// private:
//     rclcpp::TimerBase::SharedPtr timer_;

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_aligned_depth_image_;
//     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_result;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_mask;
    
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     std::string camera_frame_;
//     std::string detected_object_frame_;
//     std::string oculus_frame_;
//     sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
//     sensor_msgs::msg::Image::SharedPtr image_raw_;
//     sensor_msgs::msg::Image::SharedPtr aligned_depth_image_;

//     rclcpp::Client<wozniak_interfaces::srv::Coord>::SharedPtr client;


//     void timer_callback()
//     {
//         if (image_raw_ == nullptr || aligned_depth_image_ == nullptr || camera_info_ == nullptr)
//         {
//             RCLCPP_INFO(this->get_logger(), "Waiting for image and camera info...");
//             return;
//         }

//         // Convert ROS2 Image to OpenCV Mat
//         cv::Mat frame = cv_bridge::toCvCopy(image_raw_, "bgr8")->image;

//         // Perform image processing
//         std::pair<int, int> result = find_red_dot(frame);
//         if (result.first == -1 || result.second == -1)
//         {
//             RCLCPP_WARN(this->get_logger(), "No red dot detected in the image.");
//             return;
//         }

//         // Convert the modified OpenCV Mat back to ROS2 Image and publish
//         auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
//         pub_image_result->publish(*image_msg);

//         geometry_msgs::msg::Vector3::SharedPtr red_dot_position = get_3d_position(result.first, result.second);
//         if (!red_dot_position)
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to compute the 3D position of the red dot.");
//             return;
//         }

//         // Broadcast the position of the red dot as a transform
//         publish_transform(red_dot_position);

//         // Get red_dot_position with respect to oculus_link
//         geometry_msgs::msg::Vector3::SharedPtr position = get_red_dot_position_with_respect_to_camera();

//         if (!position)
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to get the red dot position with respect to oculus_link.");
//             return;
//         }
//         // Call Service Client
//         call_service(position);
//     }

//     geometry_msgs::msg::Vector3::SharedPtr get_3d_position(int x, int y)
//     {
//         RCLCPP_INFO(this->get_logger(), "Getting 3D position for pixel coordinates: x=%d, y=%d", x, y);

//         if (!camera_info_ || !aligned_depth_image_)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Camera info or depth image not available.");
//             return nullptr;
//         }

//         double fx = camera_info_->k[0];  // Focal length in x direction
//         double fy = camera_info_->k[4];  // Focal length in y direction
//         double cx = camera_info_->k[2];  // Principal point in x direction
//         double cy = camera_info_->k[5];  // Principal point in y direction

//         cv::Mat depth_frame = cv_bridge::toCvCopy(aligned_depth_image_, aligned_depth_image_->encoding)->image;

//         // Ensure coordinates are within bounds
//         if (x < 0 || x >= depth_frame.cols || y < 0 || y >= depth_frame.rows)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Pixel coordinates out of bounds: x=%d, y=%d", x, y);
//             return nullptr;
//         }

//         // Get the depth value
//         double depth = depth_frame.at<uint16_t>(y, x) * 0.001;  // Use (row, column) indexing
//         if (depth == 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Depth value is invalid at x=%d, y=%d", x, y);
//             return nullptr;
//         }

//         // Calculate 3D position
//         geometry_msgs::msg::Vector3::SharedPtr point(new geometry_msgs::msg::Vector3());
//         point->x = (x - cx) * depth / fx;
//         point->y = (y - cy) * depth / fy;
//         point->z = depth;  // Convert millimeters to meters

//         return point;
//     }

//     std::pair<int, int> find_red_dot(cv::Mat &frame)
//     {
//         // Convert to HSV for color detection
//         cv::Mat hsv;
//         cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

//         // Define red color range (adjust as needed)
//         cv::Scalar lower_red(0, 120, 70);
//         cv::Scalar upper_red(10, 255, 255);

//         // Threshold the image to get only red colors
//         cv::Mat mask;
//         cv::inRange(hsv, lower_red, upper_red, mask);

//         auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
//         this->pub_image_mask->publish(*mask_msg);

//         // Detect contours (objects)
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         int max_area = 0;
//         int x = -1;
//         int y = -1;
//         for (const auto& contour : contours)
//         {
//             if (cv::contourArea(contour) < max_area)
//                 continue;

//             // Get the center of the contour (using moments)
//             max_area = cv::contourArea(contour);
//             cv::Moments moments = cv::moments(contour);
//             x = static_cast<int>(moments.m10 / moments.m00);
//             y = static_cast<int>(moments.m01 / moments.m00);
//         }

//         if (x != -1 && y != -1)
//         {
//             cv::circle(frame, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);  // Blue circle
//         }

//         return std::make_pair(x, y);
//     }

//     void publish_transform(geometry_msgs::msg::Vector3::SharedPtr position)
//     {
//         // Create a transform message for the red dot position
//         geometry_msgs::msg::TransformStamped transform_stamped;
//         transform_stamped.header.stamp = this->get_clock()->now();
//         transform_stamped.header.frame_id = camera_frame_;  // Camera frame
//         transform_stamped.child_frame_id = detected_object_frame_;  // Frame for the red dot

//         // Set translation to the red dot position (cx, cy)
//         transform_stamped.transform.translation.x = position->x;
//         transform_stamped.transform.translation.y = position->y;
//         transform_stamped.transform.translation.z = position->z;
//         // transform_stamped.transform.translation.x = 0.0;
//         // transform_stamped.transform.translation.y = 0.0;
//         // transform_stamped.transform.translation.z = 1.0;

//         // No rotation (we assume no rotation for simplicity)
//         transform_stamped.transform.rotation.x = 0.0;
//         transform_stamped.transform.rotation.y = 0.0;
//         transform_stamped.transform.rotation.z = 0.0;
//         transform_stamped.transform.rotation.w = 1.0;

//         // Publish the transform
//         tf_broadcaster_->sendTransform(transform_stamped);
//     }

//     geometry_msgs::msg::Vector3::SharedPtr get_red_dot_position_with_respect_to_camera()
//     {
//         geometry_msgs::msg::TransformStamped transformStamped;
//         try {
//             // Alterando o frame de destino para o 'camera_frame_'
//             transformStamped = tf_buffer_->lookupTransform(camera_frame_, detected_object_frame_, tf2::TimePointZero);
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
//                         camera_frame_.c_str(), detected_object_frame_.c_str(), ex.what());
//             return nullptr;
//         }
//         return std::make_shared<geometry_msgs::msg::Vector3>(transformStamped.transform.translation);
//     }


//     void call_service(geometry_msgs::msg::Vector3::SharedPtr position)
//     {
//         auto request = std::make_shared<wozniak_interfaces::srv::Coord::Request>();
//         request->x = position->x;
//         request->y = position->y;
//         request->z = position->z;

//         while (!client->wait_for_service(std::chrono::seconds(1))) {
//             if (!rclcpp::ok()) {
//                 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//                 return;
//             }
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//         }

//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending request...");
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f", request->x, request->y, request->z);
//         auto result_future = client->async_send_request(request,
//         [this](rclcpp::Client<wozniak_interfaces::srv::Coord>::SharedFuture response) {
//             if (response.get()->success) {
//                 RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Success: %d", response.get()->success);
//             } else {
//                 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Coord");
//             }
//         });
//     }
// };


// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<RealsenseImageAnalyzer>());
//     rclcpp::shutdown();
//     return 0;
// }
