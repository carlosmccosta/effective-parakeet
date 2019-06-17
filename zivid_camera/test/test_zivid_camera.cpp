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

  const ros::Duration default_wait_duration{ 5 };
  static constexpr auto capture_service_name = "/zivid_camera/capture";
  static constexpr auto color_camera_info_topic_name = "/zivid_camera/color/camera_info";
  static constexpr auto color_image_color_topic_name = "/zivid_camera/color/image_color";
  static constexpr auto depth_camera_info_topic_name = "/zivid_camera/depth/camera_info";
  static constexpr auto depth_image_raw_topic_name = "/zivid_camera/depth/image_raw";
  static constexpr auto depth_points_topic_name = "/zivid_camera/depth/points";

  class SubscriptionWrapper
  {
  public:
    template <class Type, class Fn>
    static SubscriptionWrapper make(ros::NodeHandle& nh, const std::string& name, Fn&& fn)
    {
      SubscriptionWrapper w;
      boost::function<void(const boost::shared_ptr<const Type>&)> cb = [ptr = w.num_messages_.get(),
                                                                        fn = std::move(fn)](const auto& v) mutable {
        (*ptr)++;
        fn(v);
      };
      w.subscriber_ = nh.subscribe<Type>(name, 1, cb);
      return w;
    }

    std::size_t numMessages() const
    {
      return *num_messages_;
    }

  private:
    SubscriptionWrapper() : num_messages_(std::make_unique<std::size_t>(0))
    {
    }
    ros::Subscriber subscriber_;
    std::unique_ptr<std::size_t> num_messages_;
  };

  void sleepAndSpin(ros::Duration duration)
  {
    duration.sleep();
    ros::spinOnce();
  }

  void waitForReady()
  {
    ASSERT_TRUE(ros::service::waitForService(capture_service_name, default_wait_duration));
  }

  void enableFirstFrame()
  {
    dynamic_reconfigure::Client<zivid_camera::CaptureFrameConfig> frame_0_client("/zivid_camera/capture_frame/"
                                                                                 "frame_0/");
    sleepAndSpin(ros::Duration(1));
    zivid_camera::CaptureFrameConfig frame_0_cfg;
    ASSERT_TRUE(frame_0_client.getDefaultConfiguration(frame_0_cfg, default_wait_duration));
    frame_0_cfg.enabled = true;
    ASSERT_TRUE(frame_0_client.setConfiguration(frame_0_cfg));
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
  zivid_camera::CameraInfoModelName model_name;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", model_name));
  ASSERT_EQ(model_name.response.model_name, std::string("FileCamera-") + ZIVID_VERSION);
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  waitForReady();
  zivid_camera::CameraInfoSerialNumber serial_number;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serial_number));
  ASSERT_EQ(serial_number.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  waitForReady();

  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto color_image_color_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  auto depth_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name);
  auto depth_image_raw_sub = subscribe<sensor_msgs::Image>(depth_image_raw_topic_name);
  auto depth_points_sub = subscribe<sensor_msgs::PointCloud2>(depth_points_topic_name);

  auto assert_num_topics_received = [&](std::size_t numTopics) {
    ASSERT_EQ(color_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(color_image_color_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_image_raw_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_points_sub.numMessages(), numTopics);
  };

  sleepAndSpin(ros::Duration(1));
  assert_num_topics_received(0);

  zivid_camera::Capture capture;
  // Capture fails when no frames are enabled
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  sleepAndSpin(ros::Duration(1));
  assert_num_topics_received(0);

  enableFirstFrame();

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  sleepAndSpin(ros::Duration(1));
  assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  sleepAndSpin(ros::Duration(1));
  assert_num_topics_received(2);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  sleepAndSpin(ros::Duration(1));
  assert_num_topics_received(3);

  sleepAndSpin(ros::Duration(3));
  assert_num_topics_received(3);
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
  waitForReady();

  std::optional<sensor_msgs::CameraInfo> color_camera_info;
  auto color_camera_info_sub =
      subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name, [&](const auto& r) { color_camera_info = *r; });

  std::optional<sensor_msgs::CameraInfo> depth_camera_info;
  auto depth_camera_info_sub =
      subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name, [&](const auto& r) { depth_camera_info = *r; });

  auto assert_camera_info_for_file_camera = [this](const sensor_msgs::CameraInfo& ci) {
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
  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  sleepAndSpin(ros::Duration(1));

  ASSERT_EQ(color_camera_info_sub.numMessages(), 1);
  ASSERT_EQ(depth_camera_info_sub.numMessages(), 1);

  ASSERT_TRUE(color_camera_info.has_value());
  assert_camera_info_for_file_camera(*color_camera_info);
  ASSERT_TRUE(depth_camera_info.has_value());
  assert_camera_info_for_file_camera(*depth_camera_info);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_zivid_camera");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
