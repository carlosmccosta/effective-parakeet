#include "zivid_ros_wrapper_gen.hpp"

#include <Zivid/Zivid.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <string>

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

template <typename Target, typename ValueType>
std::string getRangeValueText(const Target& target)
{
  ValueType default_value =
      ConfigSettingsValueConversion<typename Target::ValueType, ValueType>::settingsValueToConfigValue(target.value());
  ValueType min_value =
      ConfigSettingsValueConversion<typename Target::ValueType, ValueType>::settingsValueToConfigValue(
          target.range().min());
  ValueType max_value =
      ConfigSettingsValueConversion<typename Target::ValueType, ValueType>::settingsValueToConfigValue(
          target.range().max());

  return getRangeValue<ValueType>(default_value, min_value, max_value);
}

template <typename Target, typename ValueType>
std::string getTypeNameText(void)
{
  return ConfigSettingsValueConversion<typename Target::ValueType, ValueType>::getConfigValueType();
}

template <typename Target>
std::string generateSettingCode(const Target& target, const std::string& parent_variable)
{
  std::string return_string;

  std::string setting_name = convertSettingsPathToConfigPath(target.path);
  std::string type_name = "None";
  std::string some_number = "0";
  std::string description = "description placeholder";
  std::string default_text = "default placeholder";

  // description = target.description;

  if constexpr (std::is_same_v<typename Target::ValueType, bool>)
  {
    default_text =
        ConfigSettingsValueConversion<bool, bool>::settingsValueToConfigValue(target.value()) ? "True" : "False";
    type_name = getTypeNameText<Target, bool>();
  }
  else if constexpr (std::is_same_v<typename Target::ValueType, double>)
  {
    default_text = getRangeValueText<Target, double>(target);
    type_name = getTypeNameText<Target, double>();
  }
  else if constexpr (std::is_same_v<typename Target::ValueType, size_t>)
  {
    default_text = getRangeValueText<Target, int>(target);
    type_name = getTypeNameText<Target, int>();
  }
  else if constexpr (std::is_same_v<typename Target::ValueType, std::chrono::microseconds>)
  {
    default_text = getRangeValueText<Target, double>(target);
    type_name = getTypeNameText<Target, double>();
  }
  else
  {
    static_assert(AssertValue<typename Target::ValueType>::value, "A default text and type name is required for type "
                                                                  "Target::ValueType");
  }

  return_string = parent_variable + ".add(\"" + setting_name + "\", " + type_name + ", " + some_number + ", " + "\"" +
                  description + "\", " + default_text + ")\n";

  return return_string;
}

template <typename Target>
std::string generateGroupCode(const Target& target, const std::string& parent_variable, std::string& new_variable)
{
  std::string return_string;

  std::string group_name = convertSettingsPathToConfigPath(target.path);
  // If no path is available, it's in the root and a main Zivid settings group is added.
  if (group_name == "")
    group_name = "zivid_settings";

  new_variable = group_name + "_group_gen";
  std::string group_code = new_variable + " = " + parent_variable + "." + "add_group(\"" + group_name + "\")\n";

  return group_code;
}

template <typename Target>
std::string getGeneratedSettingsConfigContent(const Target& target, const std::string& parent_variable)
{
  std::string return_string;

  if constexpr (Target::isContainer)
  {
    // This is a container, so it's added to a group
    std::string group_variable;
    std::string group_code = generateGroupCode(target, parent_variable, group_variable);
    target.forEach([&](const auto& member) {
      std::string subcode = getGeneratedSettingsConfigContent(member, group_variable);
      group_code += subcode;
    });

    return_string += group_code;
  }
  else
  {
    return_string = generateSettingCode(target, parent_variable);
  }

  return return_string;
}

void writeCFGToFile(const std::string& text, const std::string& file_name)
{
  std::ofstream cfg_file;
  cfg_file.open(file_name);
  cfg_file << text;
  cfg_file.close();
}

void writeCFGFile(void)
{
  std::string file_name = "ZividSettings.cfg";

  std::string start_text = "#!/usr/bin/env python\n\n#GENERATED FILE\n\nPACKAGE = \"zivid_ros_wrapper\"\nimport "
                           "roslib; roslib.load_manifest(PACKAGE);\nfrom "
                           "dynamic_reconfigure.parameter_generator_catkin import *\ngen = ParameterGenerator()\n\n";
  std::string zivid_settings = getGeneratedSettingsConfigContent(Zivid::Settings{}, "gen") + "\n";
  std::string end_text = "exit(gen.generate(PACKAGE, \"zivid_ros_wrapper\", \"ZividSettings\"))\n";

  std::string all_code = start_text + zivid_settings + end_text;

  writeCFGToFile(all_code, file_name);
}

template <typename Target>
void writeStateSrvFile(const Target& target)
{
  std::string file_name = "state_srv_files/" + getFileNameFromZividStatePath(target.path);
  // TODO find a cleaner solution to this.

  std::string type_name = getTypeNameFromZividState<typename Target::ValueType>();

  std::ofstream srv_file;
  srv_file.open(file_name);
  srv_file << "---" << std::endl;
  srv_file << type_name << " "
           << "value" << std::endl;
  srv_file.close();
}

void writeStateSrvFiles(void)
{
  Zivid::CameraState state;
  state.traverseValues([&](const auto& state_entry) { writeStateSrvFile(state_entry); });
}

int main(int /*argc*/, char** /*argv*/)
{
  writeCFGFile();
  writeStateSrvFiles();

  return 0;
}
