#include "parse_settings.hpp"

#include <Zivid/Zivid.h>

#include <dynamic_reconfigure/config_tools.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <string>
#include <cctype>
#include <chrono>

namespace
{
template <class T>
using NormalizedType = std::remove_const_t<std::remove_reference_t<T>>;

template <class T, class SettingType>
constexpr bool isSettingType = std::is_same<NormalizedType<T>, SettingType>::value;

template <class T>
constexpr bool isFilters = isSettingType<T, Zivid::Settings::Filters>;

template <class T>
constexpr bool isColorBalance =
    isSettingType<T, Zivid::Settings::BlueBalance> || isSettingType<T, Zivid::Settings::RedBalance>;

template <class T>
constexpr bool isGlobalSetting = isFilters<T> || isColorBalance<T>;

// Function is used to get the text from a ranged value typically specified with a
// default, min and max. The function requires a string conversion on the << operator.
template <typename ValueType>
std::string getRangeValue(ValueType default_value, ValueType min_value, ValueType max_value)
{
  std::stringstream default_value_ss;
  std::stringstream min_value_ss;
  std::stringstream max_value_ss;

  default_value_ss << default_value;
  min_value_ss << min_value;
  max_value_ss << max_value;

  return default_value_ss.str() + ", " + min_value_ss.str() + ", " + max_value_ss.str();
}

template <typename ZividSetting, typename ValueType>
std::string getRangeValueText(const ZividSetting& zividSetting)
{
  ValueType default_value =
      ConfigSettingsValueConversion<typename ZividSetting::ValueType, ValueType>::settingsValueToConfigValue(
          zividSetting.value());
  ValueType min_value =
      ConfigSettingsValueConversion<typename ZividSetting::ValueType, ValueType>::settingsValueToConfigValue(
          zividSetting.range().min());
  ValueType max_value =
      ConfigSettingsValueConversion<typename ZividSetting::ValueType, ValueType>::settingsValueToConfigValue(
          zividSetting.range().max());

  return getRangeValue<ValueType>(default_value, min_value, max_value);
}

template <typename ZividSetting, typename ValueType>
std::string getTypeNameText(void)
{
  return ConfigSettingsValueConversion<typename ZividSetting::ValueType, ValueType>::configValueType;
}

template <typename ZividSetting>
std::string generateSettingCode(const ZividSetting& zividSetting, const std::string& parent_variable)
{
  std::string setting_name = convertSettingsPathToConfigPath(zividSetting.path);
  std::string type_name = "None";
  std::string some_number = "0";
  std::string description = "description placeholder";
  std::string default_text = "default placeholder";

  // description = zividSetting.description;

  if constexpr (std::is_same_v<typename ZividSetting::ValueType, bool>)
  {
    default_text =
        ConfigSettingsValueConversion<bool, bool>::settingsValueToConfigValue(zividSetting.value()) ? "True" : "False";
    type_name = getTypeNameText<ZividSetting, bool>();
  }
  else if constexpr (std::is_same_v<typename ZividSetting::ValueType, double>)
  {
    default_text = getRangeValueText<ZividSetting, double>(zividSetting);
    type_name = getTypeNameText<ZividSetting, double>();
  }
  else if constexpr (std::is_same_v<typename ZividSetting::ValueType, size_t>)
  {
    default_text = getRangeValueText<ZividSetting, int>(zividSetting);
    type_name = getTypeNameText<ZividSetting, int>();
  }
  else if constexpr (std::is_same_v<typename ZividSetting::ValueType, std::chrono::microseconds>)
  {
    default_text = getRangeValueText<ZividSetting, double>(zividSetting);
    type_name = getTypeNameText<ZividSetting, double>();
  }
  else
  {
    static_assert(AssertValue<typename ZividSetting::ValueType>::value, "A default text and type name is required for "
                                                                        "type "
                                                                        "ZividSetting::ValueType");
  }

  return parent_variable + ".add(\"" + setting_name + "\", " + type_name + ", " + some_number + ", " + "\"" +
         description + "\", " + default_text + ")\n";
}

template <typename ZividSetting>
std::string generateGroupCode(const ZividSetting& zividSetting, const std::string& parent_variable,
                              const std::string& groupVariable)
{
  std::stringstream ss;
  ss << groupVariable << " = " << parent_variable << ".add_group(\"" << zividSetting.name << "\")\n";
  return ss.str();
}

template <typename ZividSetting>
std::string frameSettingsConfig(const ZividSetting& zividSetting, const std::string& parent_variable)
{
  if constexpr (isGlobalSetting<ZividSetting>)
  {
    // ignore
  }
  else if constexpr (ZividSetting::isContainer)
  {
    std::stringstream ss;
    zividSetting.forEach([&](const auto& member) { ss << frameSettingsConfig(member, parent_variable); });
    return ss.str();
  }
  else
  {
    return generateSettingCode(zividSetting, parent_variable);
  }
  return "";
}

template <typename ZividSetting>
std::string generalSettingsConfig(const ZividSetting& zividSetting, const std::string& parent_variable,
                                  bool anyParentNodeIsGlobal = false)
{
  if constexpr (ZividSetting::isContainer)
  {
    std::string group_variable = convertSettingsPathToConfigPath(zividSetting.path);
    if (group_variable.empty())
      group_variable = parent_variable;

    std::stringstream ss2;
    zividSetting.forEach([&](const auto& member) {
      ss2 << generalSettingsConfig(member, group_variable, anyParentNodeIsGlobal || isGlobalSetting<ZividSetting>);
    });

    if (!ss2.str().empty())
    {
      std::stringstream ss;
      if (group_variable != parent_variable)
        ss << generateGroupCode(zividSetting, parent_variable, group_variable);
      ss << ss2.str();
      return ss.str();
    }
  }
  else if (!ZividSetting::isContainer && (isGlobalSetting<ZividSetting> || anyParentNodeIsGlobal))
  {
    return generateSettingCode(zividSetting, parent_variable);
  }
  return "";
}

/*


  class ZividFrameSettingsAdapter
  {
    public:
      void populate(Zivid::Settings &);
    private:

  }

*/

void writeCFGToFile(const std::string& file_name, const std::string& text)
{
  std::ofstream cfg_file;
  cfg_file.open(file_name);
  cfg_file << text;
  cfg_file.close();
}

void writeCFGFile(void)
{
  std::string file_name = "ZividSettings.cfg";

  std::stringstream ss;

  ss << "#!/usr/bin/env python\n\n"
        "#GENERATED FILE\n\n"
        "PACKAGE = \"zivid_camera\"\n"
        "import roslib\n"
        "roslib.load_manifest(PACKAGE);\n"
        "from dynamic_reconfigure.parameter_generator_catkin import *\n\n";

  ss << "frame_settings_gen = ParameterGenerator()\n";
  ss << frameSettingsConfig(Zivid::Settings{}, "frame_settings_gen");
  ss << "frame_settings_gen.generate(PACKAGE, \"zivid_camera\", \"CaptureFrameSettings\")\n\n";

  ss << "general_settings_gen = ParameterGenerator()\n";
  ss << generalSettingsConfig(Zivid::Settings{}, "general_settings_gen");
  ss << "general_settings_gen.generate(PACKAGE, \"zivid_camera\", \"CaptureGeneralSettings\")\n";

  writeCFGToFile(file_name, ss.str());
}
}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
  writeCFGFile();
  return 0;
}
