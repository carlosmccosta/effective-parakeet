#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureFrameConfig.h>

#include <Zivid/Version.h>

#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class ZividNodeTest : public testing::Test
{
protected:
  ros::NodeHandle nh_;
public:
  const ros::Duration defaultWaitDuration{ 5 };

  void sleepAndSpin(ros::Duration duration)
  {
    duration.sleep();
    ros::spinOnce();
  }
};

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  ASSERT_TRUE(ros::service::waitForService("/zivid_camera/capture", defaultWaitDuration));
  zivid_camera::CameraInfoModelName modelName;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", modelName));
  ASSERT_EQ(modelName.response.model_name, std::string("FileCamera-") + ZIVID_VERSION);
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  ASSERT_TRUE(ros::service::waitForService("/zivid_camera/capture", defaultWaitDuration));
  zivid_camera::CameraInfoSerialNumber serialNumber;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serialNumber));
  ASSERT_EQ(serialNumber.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  ASSERT_TRUE(ros::service::waitForService("/zivid_camera/capture", defaultWaitDuration));

  std::size_t numPointsReceived = 0;
  boost::function<void(const boost::shared_ptr<sensor_msgs::PointCloud2 const>&)> onPoints = [&](const auto&) {
    numPointsReceived++;
  };
  auto pointsSub = nh_.subscribe("/zivid_camera/depth/points", 1, onPoints);

  std::size_t numRgbImagesReceived = 0;
  boost::function<void(const boost::shared_ptr<sensor_msgs::Image const>&)> onRgbImage = [&](const auto&) {
    numRgbImagesReceived++;
  };
  auto rgbImageSub = nh_.subscribe("/zivid_camera/rgb/image_rect_color", 1, onRgbImage);

  std::size_t numDepthImagesReceived = 0;
  boost::function<void(const boost::shared_ptr<sensor_msgs::Image const>&)> onDepthImage = [&](const auto&) {
    numDepthImagesReceived++;
  };
  auto depthImageSub = nh_.subscribe("/zivid_camera/depth/image_rect", 1, onDepthImage);

  auto verifyNumTopicsReceived = [&](std::size_t numTopics) {
    ASSERT_EQ(numPointsReceived, numTopics);
    ASSERT_EQ(numRgbImagesReceived, numTopics);
    ASSERT_EQ(numDepthImagesReceived, numTopics);
  };

  sleepAndSpin(ros::Duration(1));
  verifyNumTopicsReceived(0);

  zivid_camera::Capture capture;
  // Capture fails when no frames are enabled
  ASSERT_FALSE(ros::service::call("/zivid_camera/capture", capture));
  sleepAndSpin(ros::Duration(1));
  verifyNumTopicsReceived(0);

  // Enable first frame
  dynamic_reconfigure::Client<zivid_camera::CaptureFrameConfig> frame0Client("/zivid_camera/capture_frame/frame_0/");
  sleepAndSpin(ros::Duration(1));
  zivid_camera::CaptureFrameConfig frame0Cfg;
  ASSERT_TRUE(frame0Client.getDefaultConfiguration(frame0Cfg, defaultWaitDuration));
  frame0Cfg.enabled = true;
  ASSERT_TRUE(frame0Client.setConfiguration(frame0Cfg));

  ASSERT_TRUE(ros::service::call("/zivid_camera/capture", capture));
  sleepAndSpin(ros::Duration(1));
  verifyNumTopicsReceived(1);

  ASSERT_TRUE(ros::service::call("/zivid_camera/capture", capture));
  sleepAndSpin(ros::Duration(1));
  verifyNumTopicsReceived(2);

  ASSERT_TRUE(ros::service::call("/zivid_camera/capture", capture));
  sleepAndSpin(ros::Duration(1));
  verifyNumTopicsReceived(3);

  sleepAndSpin(ros::Duration(3));
  verifyNumTopicsReceived(3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_zivid_camera");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
