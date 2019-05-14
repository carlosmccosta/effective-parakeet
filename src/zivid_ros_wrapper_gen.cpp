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

template <typename Target>
std::string generateSrvInclude(const Target& target)
{
  std::string file_name = getFileNameFromZividStatePath(target.path);

  // Replace .srv file ending.
  std::string file_ending = ".srv";
  std::string file_ending_replacement = ".h";
  int file_ending_index = file_name.rfind(file_ending);
  std::string header_file_name = file_name.substr(0, file_ending_index) + file_ending_replacement;

  std::string include_text = "#include <zivid_ros_wrapper/" + header_file_name + ">";

  return include_text;
}

template <typename Target>
std::string getSrvClassName(const Target& target)
{
  std::string file_name = getFileNameFromZividStatePath(target.path);

  std::string file_ending = ".srv";
  std::string file_ending_replacement = "";
  int file_ending_index = file_name.rfind(file_ending);
  std::string srv_class_name = file_name.substr(0, file_ending_index) + file_ending_replacement;

  return srv_class_name;
}

template <typename Target>
std::string getSrvCallbackName(const Target& target)
{
  std::string srv_class_name = getSrvClassName(target);
  std::string srv_callback_name = "srv" + srv_class_name + "Callback";

  return srv_callback_name;
}

std::string getSrvNamespace(void)
{
  return "zivid_ros_wrapper";
}

template <typename Target>
std::string stateTypePathToColonNotation(const Target& target)
{
  std::string path = target.path;
  std::string dot_notated = "";

  for (int i = 0; i < path.size(); i++)
  {
    std::string c;
    c += path[i];
    if (c == "/")
      c = "::";

    dot_notated += c;
  }

  return dot_notated;
}

template <typename Target>
std::string stateTypePathToDotNotation(const Target& target)
{
  std::string path = target.path;
  std::string dot_notated = "";

  for (int i = 0; i < path.size(); i++)
  {
    std::string c;
    c += path[i];
    if (c == "/")
      c = "().";

    dot_notated += c;
  }

  std::transform(dot_notated.begin(), dot_notated.end(), dot_notated.begin(), ::tolower);
  return dot_notated;
}
template <>
std::string stateTypePathToDotNotation(const Zivid::CameraState::Available&)
{
  return "isAvailable";
}
template <>
std::string stateTypePathToDotNotation(const Zivid::CameraState::Connected&)
{
  return "isConnected";
}

template <typename Target>
std::string generateSrvCallback(const Target& target)
{
  std::string callback_type = "bool";
  std::string callback_name = getSrvCallbackName(target);
  std::string srv_class_name = getSrvClassName(target);
  std::string req_type = getSrvNamespace() + "::" + srv_class_name + "::Request&";
  std::string res_type = getSrvNamespace() + "::" + srv_class_name + "::Response&";

  // Now the content of the function
  std::string response_member_name = "res.value";
  std::string state_member_name = "camera.state()." + stateTypePathToDotNotation(target) + "().value()";
  std::string state_member_value_type = stateTypePathToColonNotation(target);
  std::string conversion_class_name = "StateSrvValueConversion";
  std::string conversion_function_name = "stateValueToSrvValue";
  std::string state_template_type = "Zivid::CameraState::" + state_member_value_type + "::ValueType";
  std::string srv_template_type = "decltype(" + response_member_name + ")";

  std::string return_value = "true";

  std::string callback_function_string =
      callback_type + " " + callback_name + "(" + req_type + ", " + res_type + " res, " +
      "const Zivid::Camera& camera" + ")\n" + "{\n\t" + response_member_name + " = " + conversion_class_name + "<" +
      state_template_type + ", " + srv_template_type + ">::" + conversion_function_name + "(" + state_member_name +
      ");\n\t" + "return " + return_value + ";\n" + "}\n";

  return callback_function_string;
}

template <typename Target>
std::string generateSrvServerSetup(const Target& target, const std::string& node_handle_name,
                                   const std::string zivid_camera_name, const std::string& server_list_name)
{
  std::string service_name = getSrvClassName(target);
  std::string service_path = target.path;
  std::transform(service_path.begin(), service_path.end(), service_path.begin(), ::tolower);
  std::string info_string = "\tROS_INFO(\"Registering " + service_name + " service at %s\", \"" + service_path + "\")";

  std::string callback_function_name = getSrvCallbackName(target);
  std::string srv_namespace = getSrvNamespace();
  std::string srv_class_name = getSrvClassName(target);
  std::string function_callback_variable_name = "callback_" + callback_function_name;
  std::string callback_template_req_type = srv_namespace + "::" + srv_class_name + "::Request&";
  std::string callback_template_res_type = srv_namespace + "::" + srv_class_name + "::Response&";
  std::string callback_camera_type = "const Zivid::Camera& camera";
  std::string callback_variable_text = "boost::function<bool(" + callback_template_req_type + ", " +
                                       callback_template_res_type + ")> " + function_callback_variable_name +
                                       " = boost::bind(& " + callback_function_name + ", _1, _2, " + zivid_camera_name +
                                       ");";

  std::string callback_function_variable_string = "& " + callback_function_name;
  std::string service_server_type = "ros::ServiceServer";
  std::string service_server_name = "server_" + getSrvCallbackName(target);

  std::string srv_setup_string = info_string + ";\n\t" + callback_variable_text + "\n\t" + server_list_name +
                                 ".push_back(" + node_handle_name + ".advertiseService(\"" + service_path + "\", " +
                                 function_callback_variable_name + "));\n";

  return srv_setup_string;
}

void writeTextToFile(const std::string& file_content, const std::string& file_path)
{
  std::ofstream file_stream;
  file_stream.open(file_path);
  file_stream << file_content;
  file_stream.close();
}

void writeZividSrvHeader(const std::string& file_name)
{
  std::string include_guard_start = "#ifndef ZIVID_SERVICES_GENERATED_M\n#define ZIVID_SERVICES_GENERATED_M";
  std::string include_guard_end = "#endif //ZIVID_SERVICES_GENERATED_M";
  std::string node_handle_name = "nh";
  std::string zivid_camera_name = "camera";
  std::string server_list_name = "server_list";
  std::string main_function_name = "void setupStatusServers(ros::NodeHandle&" + node_handle_name +
                                   ", const Zivid::Camera&" + zivid_camera_name +
                                   ", std::vector<ros::ServiceServer>& " + server_list_name + ")";
  std::string extra_includes = "#include <zivid_ros_wrapper_gen.hpp>";

  std::vector<std::string> include_file_list;
  std::vector<std::string> srv_callback_file_list;
  std::vector<std::string> server_setup_list;

  Zivid::CameraState state;
  state.traverseValues([&](const auto& state_entry) {
    std::string srv_include = generateSrvInclude(state_entry);
    std::string srv_callback = generateSrvCallback(state_entry);
    std::string srv_setup = generateSrvServerSetup(state_entry, node_handle_name, zivid_camera_name, server_list_name);

    include_file_list.push_back(srv_include);
    srv_callback_file_list.push_back(srv_callback);
    server_setup_list.push_back(srv_setup);
  });

  std::string file_text = include_guard_start + "\n\n";

  file_text += extra_includes + "\n";
  for (const std::string& include_text : include_file_list)
  {
    file_text += include_text + "\n";
  }
  file_text += "\n";
  for (const std::string& callback_function : srv_callback_file_list)
  {
    file_text += callback_function + "\n";
  }
  file_text += "\n";
  file_text += main_function_name + "\n";
  file_text += "{\n";

  for (const std::string& server_setup : server_setup_list)
  {
    file_text += server_setup + "\n";
  }

  file_text += "}\n\n";
  file_text += include_guard_end + "\n";

  writeTextToFile(file_text, file_name);
}

int main(int /*argc*/, char** /*argv*/)
{
  writeCFGFile();
  writeStateSrvFiles();
  writeZividSrvHeader("zivid_generated_headers/zivid_services_generated.h");

  return 0;
}
