#include "zivid_camera/zivid_camera.hpp"

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>

#include "zivid_camera_gen.hpp"
TEST(ParameterPathRename, testAlreadyCorrectString)
{
  EXPECT_EQ("filters_outlier_enabled", convertSettingsPathToConfigPath("filters_outlier_enabled"));
}
TEST(ParameterPathRename, testStartingCapitalLetter)
{
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter_test_enabled"));
}
TEST(ParameterPathRename, testCapitalLetters)
{
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("FilterTestEnabled"));
}
TEST(ParameterPathRename, testSlashes)
{
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("filter/test/enabled"));
}
TEST(ParameterPathRename, testSlashesAndCapitals)
{
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter/Test/Enabled"));
}
TEST(ParameterPathRename, testOnlySingleUnderscore)
{
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter/Test_Enabled"));
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter/Test_/Enabled"));
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter//Test_Enabled"));
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter/Test_Enabled"));
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter/_Test_Enabled"));
  EXPECT_EQ("filter_test_enabled", convertSettingsPathToConfigPath("Filter_/_Test/_/Enabled"));
}

// Some initialization is required to setup the ros node in test mode
class ZividCameraClassTest : public testing::Test
{
public:
  ZividCameraClassTest(void)
  {
    // param_setup.setParam("zivid_test_mode_file", "test.zdf");

    // system("rosrun dynamic_reconfigure dynparam set_from_parameters zivid_camera capture_mode \"capture\"");
  }

  ~ZividCameraClassTest(void)
  {
  }

protected:
  virtual void SetUp(void)
  {
  }
  virtual void TearDown(void)
  {
  }
};

TEST_F(ZividCameraClassTest, testConstructor)
{
  // This crashes because of underlying zivid application
  zivid_camera::ZividCamera wrapper;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
