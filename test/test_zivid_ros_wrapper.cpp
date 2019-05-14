#include "zivid_ros_wrapper/zivid_ros_wrapper.hpp"

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>

#include "zivid_ros_wrapper_gen.hpp"
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
class ZividRosWrapperClassTest : public testing::Test
{
public:
  ZividRosWrapperClassTest(void)
  {
    // param_setup.setParam("zivid_test_mode_file", "test.zdf");

    // system("rosrun dynamic_reconfigure dynparam set_from_parameters zivid_ros_wrapper capture_mode \"capture\"");
  }

  ~ZividRosWrapperClassTest(void)
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

TEST_F(ZividRosWrapperClassTest, testConstructor)
{
  // This crashes because of underlying zivid application
  zivid_ros_wrapper::ZividRosWrapper wrapper;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
