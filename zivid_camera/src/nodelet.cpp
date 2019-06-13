#include "zivid_camera.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory>
#include <exception>

namespace zivid_camera
{
class ZividNodelet : public nodelet::Nodelet
{
private:
  virtual void onInit()
  {
    try
    {
      camera = std::make_unique<zivid_camera::ZividCamera>(getNodeHandle());
    }
    catch (const std::exception& e)
    {
      NODELET_ERROR_STREAM("Failed to initialize camera driver. Exception: \"" << e.what() << "\"");
    }
  };

  std::unique_ptr<zivid_camera::ZividCamera> camera;
};

}  // namespace zivid_camera

PLUGINLIB_EXPORT_CLASS(zivid_camera::ZividNodelet, nodelet::Nodelet)