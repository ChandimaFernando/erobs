#include <pose_estimator/pose_estimator.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

PoseEstimator::PoseEstimator()
: Node("PoseEstimator"), static_broadcaster_(this), tf_broadcaster_(this)
{

  // tf2::Quaternion camera_qua;
  // tf2::Quaternion holder;

  // // Rotation matrix to quaternion conversion
  // tf2::Matrix3x3 rotation_matrix(
  //   0, -1, 0,
  //   0, 0, -1,
  //   1, 0, 0
  // );

  // rotation_matrix.getRotation(camera_qua);
  // camera_qua.setRPY();
  // // Define the transform
  // geometry_msgs::msg::TransformStamped transformStamped;
  // transformStamped.header.stamp = this->now();
  // transformStamped.header.frame_id = "world";
  // transformStamped.child_frame_id = "camera_base";
  // transformStamped.transform.translation.x = 0.576;
  // transformStamped.transform.translation.y = -0.675;
  // transformStamped.transform.translation.z = 0.263;
  // transformStamped.transform.rotation.x = camera_qua.x();
  // transformStamped.transform.rotation.y = camera_qua.y();
  // transformStamped.transform.rotation.z = camera_qua.z();
  // transformStamped.transform.rotation.w = camera_qua.w();

  // static_broadcaster_.sendTransform(transformStamped);


  // geometry_msgs::msg::TransformStamped transformStamped_holder;
  // transformStamped_holder.header.stamp = this->now();
  // transformStamped_holder.header.frame_id = "world";
  // transformStamped_holder.child_frame_id = "sample";
  // transformStamped_holder.transform.translation.x = 0.046727;
  // transformStamped_holder.transform.translation.y = 0.152187;
  // transformStamped_holder.transform.translation.z = 0.679315;
  // // transformStamped_holder.transform.rotation.x = 0;
  // // transformStamped_holder.transform.rotation.y = 0;
  // // transformStamped_holder.transform.rotation.z = 0;
  // // transformStamped_holder.transform.rotation.w = 1;

  // static_broadcaster_.sendTransform(transformStamped_holder);


  service = this->create_service<pdf_beamtime_interfaces::srv::EstimatedPoseMsg>(
    "pose_service",
    std::bind(
      &PoseEstimator::get_pose, this, std::placeholders::_1, std::placeholders::_2));

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

  RCLCPP_INFO(this->get_logger(), "Pose estimator node started!");

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

      auto tranlsation = tvecs[0];
      std::vector<double> raw_rpyxyz =
      {roll, pitch, yaw, tranlsation[0], tranlsation[1], tranlsation[2]};

      // Median filter gets applied
      median_filter_->update(raw_rpyxyz, median_filtered_rpy);

      object_pose_in_world =
      {median_filtered_rpy[2],
        median_filtered_rpy[0],
        median_filtered_rpy[1],
        0.576 - median_filtered_rpy[5],
        -0.675 + median_filtered_rpy[3],
        0.263 - median_filtered_rpy[4]
      };

      // tf2::Matrix3x3 rotation_matrix(
      //   0, -1, 0,
      //   0, 0, -1,
      //   -1, 0, 0
      // );

      // stf2::Vector3 world_to_camera_translation(0.576, -0.675, 0.263);

      // stf2::Vector3 filtered_xyz(median_filtered_rpy[3],
      //   median_filtered_rpy[4], median_filtered_rpy[5]);
      // tf2::Vector3 filtered_rpy(median_filtered_rpy[0],
      //   median_filtered_rpy[1], median_filtered_rpy[2]);

      // tf2::Vector3 world_xyz = rotation_matrix * filtered_xyz + world_to_camera_translation;
      // tf2::Vector3 world_rpy = rotation_matrix * filtered_rpy;
      // // Add to the tf server here
      // geometry_msgs::msg::TransformStamped transformStamped_tag;

      // transformStamped_tag.header.stamp = this->now();
      // transformStamped_tag.header.frame_id = "world";
      // transformStamped_tag.child_frame_id = "tag_1";

      // transformStamped_tag.transform.translation.x = 0.576 - median_filtered_rpy[5];
      // transformStamped_tag.transform.translation.y = -0.675 + median_filtered_rpy[3];
      // transformStamped_tag.transform.translation.z = 0.263 - median_filtered_rpy[4];
      // // transformStamped_tag.transform.rotation = toQuaternion(roll, pitch, yaw);
      // transformStamped_tag.transform.rotation = toQuaternion(
      //   median_filtered_rpy[2],
      //   median_filtered_rpy[0],
      //   median_filtered_rpy[1]);

      // tf_broadcaster_.sendTransform(transformStamped_tag);

      RCLCPP_INFO(
        this->get_logger(), "Pose RPY XYZ: %f %f %f  \t %f %f %f ", median_filtered_rpy[0],
        median_filtered_rpy[1], median_filtered_rpy[2], median_filtered_rpy[3],
        median_filtered_rpy[4], median_filtered_rpy[5]);

    }

    // Inner try-catch
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(this->get_logger(), "Invalid argument in inner try: %s ", e.what() );
    throw;
  }
}

void PoseEstimator::get_pose(
  const std::shared_ptr<pdf_beamtime_interfaces::srv::EstimatedPoseMsg::Request> request,
  std::shared_ptr<pdf_beamtime_interfaces::srv::EstimatedPoseMsg::Response> response)
{
  int a = request->a;
  a++;
  for (size_t i = 0; i < object_pose_in_world.size(); ++i) {
    response->pose.push_back(object_pose_in_world[i]);
  }
}

geometry_msgs::msg::Quaternion PoseEstimator::toQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Quaternion msg_quat;
  msg_quat.x = quaternion.x();
  msg_quat.y = quaternion.y();
  msg_quat.z = quaternion.z();
  msg_quat.w = quaternion.w();

  return msg_quat;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<PoseEstimator>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
