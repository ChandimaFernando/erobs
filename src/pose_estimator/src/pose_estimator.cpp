#include <pose_estimator/pose_estimator.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

PoseEstimator::PoseEstimator()
: Node("PoseEstimator")
{
  subscription_rgb_.subscribe(this, "/rgb/image_raw");
  // subscription_depth_.subscribe(this, "/depth/image_raw");
  subscription_depth_.subscribe(this, "/depth_to_rgb/image_raw");

  // Synchronize messages from the two topics
  sync_ =
    std::make_shared<message_filters::Synchronizer<sync_policy>>(
    sync_policy(
      5), subscription_rgb_, subscription_depth_);
  sync_->registerCallback(&PoseEstimator::image_raw_callback, this);

  parameters_ = cv::aruco::DetectorParameters::create();
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  dictionary_ = cv::aruco::getPredefinedDictionary(
    cv::aruco::DICT_APRILTAG_36h11);

  RCLCPP_INFO(this->get_logger(), "Seg here");

  this->declare_parameter<int>("number_of_observations", 5);

  median_filtered_rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  median_filter_->configure(
    6, "", "number_of_observations",
    this->get_node_logging_interface(), this->get_node_parameters_interface());

}

void PoseEstimator::image_raw_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{

  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr_rgb =
    cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

  cv::aruco::detectMarkers(
    cv_ptr_rgb->image, dictionary_,
    markerCorners_, markerIds_, parameters_,
    rejectedCandidates_);

  try {
    cv::Mat outputImage = cv_ptr_rgb->image.clone();
    cv::aruco::drawDetectedMarkers(
      outputImage, markerCorners_,
      markerIds_);

    if (!markerIds_.empty()) {
      // rvecs: rotational vector
      // tvecs: translation vector
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        markerCorners_, 0.05, cameraMatrix_, distCoeffs_, rvecs, tvecs);

      cv::Mat R;
      std::cout << "P3P solutions found: " << rvecs.size() << std::endl;
      for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Rodrigues(rvecs[i], R);
        std::cout << "Solution " << i + 1 << ":\n";
        // std::cout << "Rotation vector: " << rvecs[i].t() << "\n";
        // std::cout << "Rotation matrix:\n" << R << "\n";
        std::cout << "Translation vector: " << tvecs[i].t() << "\n";
      }

      // Save the image as PNG
      std::string filename_rgb = "data/rgb_pose/" + std::to_string(rgb_msg->header.stamp.sec) +
        ".png";

      double r11 = R.at<double>(0, 0);
      double r21 = R.at<double>(1, 0);
      double r31 = R.at<double>(2, 0);
      double r32 = R.at<double>(2, 1);
      double r33 = R.at<double>(2, 2);

      double roll, pitch, yaw;

      // Get the filtered value
      // double filtered_value = kalman_filter_.getState();

      roll = std::atan2(r32, r33) * (180.0 / 3.141592653589793238463);
      pitch = std::asin(-1 * r31) * (180.0 / 3.141592653589793238463);
      // Calculate roll (phi)
      yaw = std::atan2(r21, r11) * (180.0 / 3.141592653589793238463);


      rclcpp::Time now = this->get_clock()->now();
      if (!last_time_flag_) {
        last_time_flag_ = true;
        dt = 0.0000000001;
      } else {
        dt = (now - last_time_).seconds();
      }
      last_time_ = now;

      Eigen::Vector3d z;
      z << roll, pitch, yaw;

      auto tranlsation = tvecs[0] ;
      std::vector<double> raw_rpy = {roll, pitch, yaw, tranlsation[0], tranlsation[1] , tranlsation[2] };

      kalman_filter_.predict(Eigen::Vector3d::Zero(), dt);   // Assuming no control input (u)
      kalman_filter_.update(z);

      Eigen::Vector3d state = kalman_filter_.getState();

      // ######### implements mean filter ##########
      median_filter_->update(raw_rpy, median_filtered_rpy);

      RCLCPP_INFO(
        this->get_logger(), "rpy: %f \t %f \t %f before ekf", roll, pitch, yaw);
      RCLCPP_INFO(
        this->get_logger(), "rpy: %f \t %f \t %f after ekf", state[0], state[1], state[2]);

      RCLCPP_INFO(
        this->get_logger(), "rpy: %f \t %f \t %f after mean filter", median_filtered_rpy[0],
        median_filtered_rpy[1],
        median_filtered_rpy[2]);

      appendVectorsToCSV("sesnor_data_raw.csv", raw_rpy);
      appendVectorsToCSV("sesnor_data_filtered.csv", median_filtered_rpy);

    } else {
      // std::string filename_rgb = "data/depth/" + std::to_string(depth_msg->header.stamp.sec) + ".png";
    }

    // Inner try-catch
  } catch (const std::invalid_argument & e) {
    std::cerr << "Invalid argument in inner try: " << e.what() << std::endl;
    throw;
  }

  // // Convert ROS image message to cv::Mat
  // cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(
  //   depth_msg,
  //   sensor_msgs::image_encodings::TYPE_32FC1);

  // // Convert 32FC1 image to CV_16UC1
  // cv_ptr_depth->image.convertTo(cv_ptr_depth->image, CV_16UC1);

  // cv::imwrite(filename_rgb, cv_ptr_rgb->image);
  // cv::imwrite(filename_rgb, outputImage);

  // // Save the image as PNG
  // std::string filename_depth = "data/depth/" + std::to_string(depth_msg->header.stamp.sec) + ".png";
  // cv::imwrite(filename_depth, cv_ptr_depth->image);

}

void PoseEstimator::appendVectorsToCSV(const std::string& filename, std::vector<double> data) {
    std::ofstream file(filename, std::ios::app); // Open in append mode

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << data[0] ; file << ","; 
    file << data[1] ; file << ",";     
    file << data[2] ; file << ",";     
    file << data[3] ; file << ",";     
    file << data[4] ; file << ",";     
    file << data[5] ; file << "\n";

    file.close();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<PoseEstimator>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
