#include <kinect_recorder/kinect_recorder.hpp>

using std::placeholders::_1;

KinectRecorder::KinectRecorder()
: Node("KinectRecorder")
{
  subscription_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/rgb/image_raw", 5, std::bind(&KinectRecorder::rgb_raw_callback, this, _1));

  subscription_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/depth/image_raw", 5, std::bind(&KinectRecorder::depth_raw_callback, this, _1));

}

void KinectRecorder::rgb_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // Save the image as PNG
  std::string filename = "data/rgb/image_" + std::to_string(msg->header.stamp.sec) + ".png";
  cv::imwrite(filename, cv_ptr->image);
}

void KinectRecorder::depth_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

  // Convert 32FC1 image to 8-bit grayscale for display
  cv::Mat image_8bit;
  cv_ptr->image.convertTo(image_8bit, CV_8U, 255.0);


  // Save the image as PNG
  std::string filename = "data/depth/image_" + std::to_string(msg->header.stamp.sec) + ".png";
  cv::imwrite(filename, image_8bit);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<KinectRecorder>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
