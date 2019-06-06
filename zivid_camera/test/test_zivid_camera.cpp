#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureFrameConfig.h>

#include <Zivid/Version.h>

#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class ZividNodeTest : public testing::Test
{
protected:
  ros::NodeHandle nh_;

  const ros::Duration defaultWaitDuration{ 5 };
  static constexpr auto captureServiceName = "/zivid_camera/capture";
  static constexpr auto colorCameraInfoTopicName = "/zivid_camera/color/camera_info";
  static constexpr auto colorCameraImageColorTopicName = "/zivid_camera/color/image_color";
  static constexpr auto depthCameraInfoTopicName = "/zivid_camera/depth/camera_info";
  static constexpr auto depthImageRawTopicName = "/zivid_camera/depth/image_raw";
  static constexpr auto depthPointsTopicName = "/zivid_camera/depth/points";

  class SubscriptionWrapper
  {
  public:
    template <class Type, class Fn>
    static SubscriptionWrapper make(ros::NodeHandle& nh, const std::string& name, Fn&& fn)
    {
      SubscriptionWrapper w;
      boost::function<void(const boost::shared_ptr<const Type>&)> cb = [ptr = w.numMessages_.get(),
                                                                        fn = std::move(fn)](const auto& v) mutable {
        (*ptr)++;
        fn(v);
      };
      w.subscriber_ = nh.subscribe<Type>(name, 1, cb);
      return w;
    }

    std::size_t numMessages() const
    {
      return *numMessages_;
    }

  private:
    SubscriptionWrapper() : numMessages_(std::make_unique<std::size_t>(0))
    {
    }
    ros::Subscriber subscriber_;
    std::unique_ptr<std::size_t> numMessages_;
  };

  void sleepAndSpin(ros::Duration duration)
  {
    duration.sleep();
    ros::spinOnce();
  }

  void waitForReady()
  {
    ASSERT_TRUE(ros::service::waitForService(captureServiceName, defaultWaitDuration));
  }

  void enableFirstFrame()
  {
    dynamic_reconfigure::Client<zivid_camera::CaptureFrameConfig> frame0Client("/zivid_camera/capture_frame/frame_0/");
    sleepAndSpin(ros::Duration(1));
    zivid_camera::CaptureFrameConfig frame0Cfg;
    ASSERT_TRUE(frame0Client.getDefaultConfiguration(frame0Cfg, defaultWaitDuration));
    frame0Cfg.enabled = true;
    ASSERT_TRUE(frame0Client.setConfiguration(frame0Cfg));
  }

  template <class Type, class Fn>
  SubscriptionWrapper subscribe(const std::string& name, Fn&& callback)
  {
    return SubscriptionWrapper::make<Type>(nh_, name, callback);
  }

  template <class Type>
  SubscriptionWrapper subscribe(const std::string& name)
  {
    return subscribe<Type>(name, [](const auto& v) {});
  }

  template <class A, class B>
  void assertArrayFloatEq(const A& actual, const B& expected)
  {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); i++)
    {
      ASSERT_FLOAT_EQ(actual[i], expected[i]);
    }
  }
};

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  waitForReady();
  zivid_camera::CameraInfoModelName modelName;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", modelName));
  ASSERT_EQ(modelName.response.model_name, std::string("FileCamera-") + ZIVID_VERSION);
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  waitForReady();
  zivid_camera::CameraInfoSerialNumber serialNumber;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serialNumber));
  ASSERT_EQ(serialNumber.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  waitForReady();

  auto colorCameraInfoSub = subscribe<sensor_msgs::CameraInfo>(colorCameraInfoTopicName);
  auto colorCameraImageColorSub = subscribe<sensor_msgs::Image>(colorCameraImageColorTopicName);
  auto depthCameraInfoSub = subscribe<sensor_msgs::CameraInfo>(depthCameraInfoTopicName);
  auto depthImageRawSub = subscribe<sensor_msgs::Image>(depthImageRawTopicName);
  auto depthPointsSub = subscribe<sensor_msgs::PointCloud2>(depthPointsTopicName);

  auto assertNumTopicsReceived = [&](std::size_t numTopics) {
    ASSERT_EQ(colorCameraInfoSub.numMessages(), numTopics);
    ASSERT_EQ(colorCameraImageColorSub.numMessages(), numTopics);
    ASSERT_EQ(depthCameraInfoSub.numMessages(), numTopics);
    ASSERT_EQ(depthImageRawSub.numMessages(), numTopics);
    ASSERT_EQ(depthPointsSub.numMessages(), numTopics);
  };

  sleepAndSpin(ros::Duration(1));
  assertNumTopicsReceived(0);

  zivid_camera::Capture capture;
  // Capture fails when no frames are enabled
  ASSERT_FALSE(ros::service::call(captureServiceName, capture));
  sleepAndSpin(ros::Duration(1));
  assertNumTopicsReceived(0);

  enableFirstFrame();

  ASSERT_TRUE(ros::service::call(captureServiceName, capture));
  sleepAndSpin(ros::Duration(1));
  assertNumTopicsReceived(1);

  ASSERT_TRUE(ros::service::call(captureServiceName, capture));
  sleepAndSpin(ros::Duration(1));
  assertNumTopicsReceived(2);

  ASSERT_TRUE(ros::service::call(captureServiceName, capture));
  sleepAndSpin(ros::Duration(1));
  assertNumTopicsReceived(3);

  sleepAndSpin(ros::Duration(3));
  assertNumTopicsReceived(3);
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
  waitForReady();

  std::optional<sensor_msgs::CameraInfo> colorCameraInfo;
  auto colorCameraInfoSub =
      subscribe<sensor_msgs::CameraInfo>(colorCameraInfoTopicName, [&](const auto& r) { colorCameraInfo = *r; });

  std::optional<sensor_msgs::CameraInfo> depthCameraInfo;
  auto depthCameraInfoSub =
      subscribe<sensor_msgs::CameraInfo>(depthCameraInfoTopicName, [&](const auto& r) { depthCameraInfo = *r; });

  auto assertCameraInfoForFileCamera = [this](const sensor_msgs::CameraInfo& ci) {
    ASSERT_EQ(ci.width, 1920);
    ASSERT_EQ(ci.height, 1200);
    ASSERT_EQ(ci.distortion_model, "plumb_bob");

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    assertArrayFloatEq(
        ci.K, std::array<double, 9>{ 2759.12329102, 0, 958.78460693, 0, 2758.73681641, 634.94018555, 0, 0, 1 });

    // R = I
    assertArrayFloatEq(ci.R, std::array<double, 9>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    assertArrayFloatEq(ci.P, std::array<double, 12>{ 2759.12329102, 0, 958.78460693, 0, 0, 2758.73681641, 634.94018555,
                                                     0, 0, 0, 1, 0 });
  };

  enableFirstFrame();
  zivid_camera::Capture capture;
  ASSERT_TRUE(ros::service::call(captureServiceName, capture));
  sleepAndSpin(ros::Duration(1));

  ASSERT_EQ(colorCameraInfoSub.numMessages(), 1);
  ASSERT_EQ(depthCameraInfoSub.numMessages(), 1);

  ASSERT_TRUE(colorCameraInfo);
  assertCameraInfoForFileCamera(*colorCameraInfo);
  ASSERT_TRUE(depthCameraInfo);
  assertCameraInfoForFileCamera(*depthCameraInfo);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_zivid_camera");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
