#ifndef ZIVID_SERVICES_GENERATED_M
#define ZIVID_SERVICES_GENERATED_M

#include <zivid_ros_wrapper_gen.hpp>
#include <zivid_ros_wrapper/Available.h>
#include <zivid_ros_wrapper/Connected.h>
#include <zivid_ros_wrapper/Live.h>
#include <zivid_ros_wrapper/TemperatureDMD.h>
#include <zivid_ros_wrapper/TemperatureGeneral.h>
#include <zivid_ros_wrapper/TemperatureLED.h>
#include <zivid_ros_wrapper/TemperatureLens.h>
#include <zivid_ros_wrapper/TemperaturePCB.h>

bool srvAvailableCallback(zivid_ros_wrapper::Available::Request&, zivid_ros_wrapper::Available::Response& res,
                          const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Available::ValueType, decltype(res.value)>::stateValueToSrvValue(
          camera.state().isAvailable().value());
  return true;
}

bool srvConnectedCallback(zivid_ros_wrapper::Connected::Request&, zivid_ros_wrapper::Connected::Response& res,
                          const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Connected::ValueType, decltype(res.value)>::stateValueToSrvValue(
          camera.state().isConnected().value());
  return true;
}

bool srvLiveCallback(zivid_ros_wrapper::Live::Request&, zivid_ros_wrapper::Live::Response& res,
                     const Zivid::Camera& camera)
{
  res.value = StateSrvValueConversion<Zivid::CameraState::Live::ValueType, decltype(res.value)>::stateValueToSrvValue(
      camera.state().live().value());
  return true;
}

bool srvTemperatureDMDCallback(zivid_ros_wrapper::TemperatureDMD::Request&,
                               zivid_ros_wrapper::TemperatureDMD::Response& res, const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Temperature::DMD::ValueType,
                              decltype(res.value)>::stateValueToSrvValue(camera.state().temperature().dmd().value());
  return true;
}

bool srvTemperatureGeneralCallback(zivid_ros_wrapper::TemperatureGeneral::Request&,
                                   zivid_ros_wrapper::TemperatureGeneral::Response& res, const Zivid::Camera& camera)
{
  res.value = StateSrvValueConversion<Zivid::CameraState::Temperature::General::ValueType, decltype(res.value)>::
      stateValueToSrvValue(camera.state().temperature().general().value());
  return true;
}

bool srvTemperatureLEDCallback(zivid_ros_wrapper::TemperatureLED::Request&,
                               zivid_ros_wrapper::TemperatureLED::Response& res, const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Temperature::LED::ValueType,
                              decltype(res.value)>::stateValueToSrvValue(camera.state().temperature().led().value());
  return true;
}

bool srvTemperatureLensCallback(zivid_ros_wrapper::TemperatureLens::Request&,
                                zivid_ros_wrapper::TemperatureLens::Response& res, const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Temperature::Lens::ValueType,
                              decltype(res.value)>::stateValueToSrvValue(camera.state().temperature().lens().value());
  return true;
}

bool srvTemperaturePCBCallback(zivid_ros_wrapper::TemperaturePCB::Request&,
                               zivid_ros_wrapper::TemperaturePCB::Response& res, const Zivid::Camera& camera)
{
  res.value =
      StateSrvValueConversion<Zivid::CameraState::Temperature::PCB::ValueType,
                              decltype(res.value)>::stateValueToSrvValue(camera.state().temperature().pcb().value());
  return true;
}

void setupStatusServers(ros::NodeHandle& nh, const Zivid::Camera& camera, std::vector<ros::ServiceServer>& server_list)
{
  ROS_INFO("Registering Available service at %s", "available");
  boost::function<bool(zivid_ros_wrapper::Available::Request&, zivid_ros_wrapper::Available::Response&)>
      callback_srvAvailableCallback = boost::bind(&srvAvailableCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("available", callback_srvAvailableCallback));

  ROS_INFO("Registering Connected service at %s", "connected");
  boost::function<bool(zivid_ros_wrapper::Connected::Request&, zivid_ros_wrapper::Connected::Response&)>
      callback_srvConnectedCallback = boost::bind(&srvConnectedCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("connected", callback_srvConnectedCallback));

  ROS_INFO("Registering Live service at %s", "live");
  boost::function<bool(zivid_ros_wrapper::Live::Request&, zivid_ros_wrapper::Live::Response&)>
      callback_srvLiveCallback = boost::bind(&srvLiveCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("live", callback_srvLiveCallback));

  ROS_INFO("Registering TemperatureDMD service at %s", "temperature/dmd");
  boost::function<bool(zivid_ros_wrapper::TemperatureDMD::Request&, zivid_ros_wrapper::TemperatureDMD::Response&)>
      callback_srvTemperatureDMDCallback = boost::bind(&srvTemperatureDMDCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("temperature/dmd", callback_srvTemperatureDMDCallback));

  ROS_INFO("Registering TemperatureGeneral service at %s", "temperature/general");
  boost::function<bool(zivid_ros_wrapper::TemperatureGeneral::Request&,
                       zivid_ros_wrapper::TemperatureGeneral::Response&)>
      callback_srvTemperatureGeneralCallback = boost::bind(&srvTemperatureGeneralCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("temperature/general", callback_srvTemperatureGeneralCallback));

  ROS_INFO("Registering TemperatureLED service at %s", "temperature/led");
  boost::function<bool(zivid_ros_wrapper::TemperatureLED::Request&, zivid_ros_wrapper::TemperatureLED::Response&)>
      callback_srvTemperatureLEDCallback = boost::bind(&srvTemperatureLEDCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("temperature/led", callback_srvTemperatureLEDCallback));

  ROS_INFO("Registering TemperatureLens service at %s", "temperature/lens");
  boost::function<bool(zivid_ros_wrapper::TemperatureLens::Request&, zivid_ros_wrapper::TemperatureLens::Response&)>
      callback_srvTemperatureLensCallback = boost::bind(&srvTemperatureLensCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("temperature/lens", callback_srvTemperatureLensCallback));

  ROS_INFO("Registering TemperaturePCB service at %s", "temperature/pcb");
  boost::function<bool(zivid_ros_wrapper::TemperaturePCB::Request&, zivid_ros_wrapper::TemperaturePCB::Response&)>
      callback_srvTemperaturePCBCallback = boost::bind(&srvTemperaturePCBCallback, _1, _2, camera);
  server_list.push_back(nh.advertiseService("temperature/pcb", callback_srvTemperaturePCBCallback));
}

#endif  // ZIVID_SERVICES_GENERATED_M
