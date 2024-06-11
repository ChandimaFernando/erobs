#include <pose_estimator/pose_estimator.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

PoseEstimator::PoseEstimator()
: Node("PoseEstimator")
{
  subscription_rgb_.subscribe(this, "/rgb/image_raw");
  subscription_depth_.subscribe(this, "/depth_to_rgb/image_raw");

  // Synchronize messages from the two topics
  sync_ =
    std::make_shared<message_filters::Synchronizer<sync_policy>>(
    sync_policy(
      5), subscription_rgb_, subscription_depth_);
  sync_->registerCallback(&PoseEstimator::image_raw_callback, this);

  // Assign parameters for ArUco tag detection
  parameters_ = cv::aruco::DetectorParameters::create();
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  dictionary_ = cv::aruco::getPredefinedDictionary(
    cv::aruco::DICT_APRILTAG_36h11);

  // This sets the moving window size for the mean filter
  this->declare_parameter<int>("number_of_observations", moving_window_median_);

  // Initial estimates 
  median_filtered_rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Configure the mean filter. 6 refers to the number of channels in the multi-channel filter
  median_filter_->configure(
    6, "", "number_of_observations",
    this->get_node_logging_interface(), this->get_node_parameters_interface());

  rclcpp::Service<pdf_beamtime_interfaces::srv::EstimatedPoseMsg>::SharedPtr service =
    this->create_service<pdf_beamtime_interfaces::srv::EstimatedPoseMsg>(
    "pose_service",
    std::bind(
      &PoseEstimator::get_pose, this, _1, _2));

}

void PoseEstimator::image_raw_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{

  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr_rgb =
    cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

  // Detect the markers from the incoming image
  cv::aruco::detectMarkers(
    cv_ptr_rgb->image, dictionary_,
    markerCorners_, markerIds_, parameters_,
    rejectedCandidates_);

  try {
    // Make a copy of the image for saving + visualizing
    cv::Mat outputImage = cv_ptr_rgb->image.clone();

    if (!markerIds_.empty()) {

      // rvecs: rotational vector
      // tvecs: translation vector
      std::vector<cv::Vec3d> rvecs, tvecs;

      // Pose estimation happens here
      cv::aruco::estimatePoseSingleMarkers(
        markerCorners_, physical_marker_size_, cameraMatrix_, distCoeffs_, rvecs, tvecs);

      cv::Mat R;
      //  Convert the rvecs to a rotation matrix
      for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Rodrigues(rvecs[i], R);
      }

      // Access each elements of the R matrix for easy rpy calculations
      double r11 = R.at<double>(0, 0);
      double r21 = R.at<double>(1, 0);
      double r31 = R.at<double>(2, 0);
      double r32 = R.at<double>(2, 1);
      double r33 = R.at<double>(2, 2);

      double roll, pitch, yaw;

      // rpy calculation
      roll = std::atan2(r32, r33) * (180.0 / 3.141592653589793238463);
      pitch = std::asin(-1 * r31) * (180.0 / 3.141592653589793238463);
      yaw = std::atan2(r21, r11) * (180.0 / 3.141592653589793238463);


      auto tranlsation = tvecs[0] ;
      std::vector<double> raw_rpyxyz = {roll, pitch, yaw, tranlsation[0], tranlsation[1] , tranlsation[2] };

      // Median filter gets applied
      median_filter_->update(raw_rpyxyz, median_filtered_rpy);

    }

    // Inner try-catch
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(this->get_logger(), "Invalid argument in inner try: %s " ,e.what() );
    throw;
  }
}

void PoseEstimator::get_pose(
  const std::shared_ptr<pdf_beamtime_interfaces::srv::EstimatedPoseMsg::Request> request,
  std::shared_ptr<pdf_beamtime_interfaces::srv::EstimatedPoseMsg::Response> response)
  {
    for (size_t i = 0; i < median_filtered_rpy.size(); ++i) {
      response->pose[i] = median_filtered_rpy[i];
    }
  }


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<PoseEstimator>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
