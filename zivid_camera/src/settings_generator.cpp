#include <Zivid/Zivid.h>

#include <dynamic_reconfigure/config_tools.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

namespace
{
template <class T>
using NormalizedType = std::remove_const_t<std::remove_reference_t<T>>;

template <class T, class ZividSetting>
constexpr bool isZividSetting = std::is_same<NormalizedType<T>, ZividSetting>::value;

template <class T>
constexpr bool isFilters = isZividSetting<T, Zivid::Settings::Filters>;

template <class T>
constexpr bool isColorBalance =
    isZividSetting<T, Zivid::Settings::BlueBalance> || isZividSetting<T, Zivid::Settings::RedBalance>;

template <class T>
constexpr bool isGeneralSetting = isFilters<T> || isColorBalance<T>;

template <typename C, typename = void>
struct HasRange : std::false_type
{
};

template <typename C>
struct HasRange<C, std::void_t<decltype(std::declval<C>().range())>> : std::true_type
{
};

template <typename ZividSetting>
struct AssertValue
{
  static constexpr bool value = false;
};

std::string convertSettingsPathToZividClassName(const std::string& path)
{
  return "Zivid::Settings::" + boost::replace_all_copy<std::string>(path, "/", "::");
}

std::string convertSettingsPathToConfigPath(std::string path)
{
  path = boost::replace_all_copy<std::string>(path, "/", "_");
  const std::regex re("([^_^])([A-Z])");
  path = std::regex_replace(path, re, "$1_$2");  // Convert fex ExposureTime to Exposure_Time
  return boost::algorithm::to_lower_copy(path);
}

class DynamicReconfigureCfgGenerator
{
public:
  DynamicReconfigureCfgGenerator(const std::string& className) : className_(className), insertEnabled_(false)
  {
  }

  template <class ValueType>
  auto convertValueToRosValue(ValueType value)
  {
    // Convert from our own setting value types to the types that ROS params supports.
    if constexpr (std::is_same_v<ValueType, bool> || std::is_same_v<ValueType, double>)
    {
      return value;
    }
    else if constexpr (std::is_same_v<ValueType, std::size_t>)
    {
      return static_cast<int>(value);
    }
    else if constexpr (std::is_same_v<ValueType, std::chrono::microseconds>)
    {
      return std::chrono::duration_cast<std::chrono::duration<double>>(value).count();
    }
    else
    {
      static_assert(AssertValue<ValueType>::value, "Could not convert ValueType to ROS type.");
    }
  }

  template <class RosType>
  std::string rosTypeName()
  {
    if constexpr (std::is_same_v<RosType, bool>)
    {
      return "bool_t";
    }
    else if constexpr (std::is_same_v<RosType, double>)
    {
      return "double_t";
    }
    else if constexpr (std::is_same_v<RosType, int>)
    {
      return "int_t";
    }
    else
    {
      static_assert(AssertValue<RosType>::value, "Could not convert RosType to a ROS typename string.");
    }
  }

  template <class RosType>
  std::string rosTypeToString(RosType v)
  {
    if constexpr (std::is_same_v<RosType, bool>)
    {
      return v ? "True" : "False";
    }
    else if constexpr (std::is_same_v<RosType, double> || std::is_same_v<RosType, int>)
    {
      return std::to_string(v);
    }
    else
    {
      static_assert(AssertValue<RosType>::value, "Could not convert RosType to a string value.");
    }
  }

  template <class ValueType>
  std::string valueTypeToRosTypeString(ValueType v)
  {
    return rosTypeToString(convertValueToRosValue(v));
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    const auto setting_name = convertSettingsPathToConfigPath(s.path);
    const auto level = "0";
    // Newlines must be converted to \\n so that the auto-generated files end up being correct
    const auto description = boost::replace_all_copy<std::string>(s.description, "\n", R"(\\n)");
    const auto type_name = rosTypeName<decltype(convertValueToRosValue(s.value()))>();
    const auto default_value = valueTypeToRosTypeString(s.value());

    ss_ << "gen.add(\"" << setting_name << "\", " << type_name << ", " << level << ", "
        << "\"" << description << "\", " << default_value;

    if constexpr (HasRange<NormalizedType<ZividSetting>>::value)
    {
      ss_ << ", " << valueTypeToRosTypeString(s.range().min()) << ", " << valueTypeToRosTypeString(s.range().max());
    }
    ss_ << ")\n";
  }

  void insertEnabled()
  {
    insertEnabled_ = true;
  }

  std::string str()
  {
    std::stringstream res;
    res << "#!/usr/bin/env python\n\n"
           "# This is an auto-generated cfg file. Do not edit!\n\n"
           "PACKAGE = \"zivid_camera\"\n"
           "import roslib\n"
           "roslib.load_manifest(PACKAGE);\n"
           "from dynamic_reconfigure.parameter_generator_catkin import *\n\n";

    res << "gen = ParameterGenerator()\n";
    if (insertEnabled_)
    {
      res << "gen.add(\"enabled\", bool_t, 0, \"When frame is enabled it will be included in captures\", "
             "False)\n";
    }
    res << ss_.str();
    res << "gen.generate(PACKAGE, \"zivid_camera\", \"" + className_ + "\")\n";
    return res.str();
  }

private:
  std::string className_;
  bool insertEnabled_;
  std::stringstream ss_;
};

class ApplyConfigToZividSettingsGenerator
{
public:
  ApplyConfigToZividSettingsGenerator(const std::string& className) : className_(className)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    using T = NormalizedType<decltype(s)>;
    using VT = typename T::ValueType;

    const auto cfgId = "cfg." + convertSettingsPathToConfigPath(s.path);
    const auto zividClassName = convertSettingsPathToZividClassName(s.path);
    ss_ << "  s.set(" + zividClassName + "{ ";

    if constexpr (std::is_same_v<VT, std::size_t>)
    {
      ss_ << "static_cast<std::size_t>(" + cfgId + ")";
    }
    else if constexpr (std::is_same_v<VT, std::chrono::microseconds>)
    {
      ss_ << "std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(" + cfgId + "))";
    }
    else
    {
      ss_ << cfgId;
    }
    ss_ << " });\n";
  }

  std::string str()
  {
    std::stringstream res;
    res << "void apply" << className_
        << "ConfigToZividSettings(const zivid_camera::" + className_ + "Config& cfg, Zivid::Settings& s)\n";
    res << "{\n";
    res << ss_.str();
    res << "}\n";
    return res.str();
  }

private:
  std::string className_;
  std::stringstream ss_;
};

class GetConfigMinMaxDefFromZividSettingsGenerator
{
public:
  enum class Type
  {
    Min,
    Max,
    Default
  };

  GetConfigMinMaxDefFromZividSettingsGenerator(const std::string& className, Type type)
    : className_(className), type_(type)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    using T = NormalizedType<decltype(s)>;
    using VT = typename T::ValueType;
    const auto cfgId = "cfg." + convertSettingsPathToConfigPath(s.path);
    const auto zividClassName = convertSettingsPathToZividClassName(s.path);

    const auto valueStr = [&]() {
      if (type_ == Type::Min || type_ == Type::Max)
      {
        return "s.get<" + zividClassName + ">().range()." + type() + "()";
      }
      else
      {
        return "s.get<" + zividClassName + ">().value()";
      }
    }();

    if (type_ == Type::Default || HasRange<T>::value)
    {
      if constexpr (std::is_same_v<VT, std::chrono::microseconds>)
      {
        ss_ << "  " + cfgId + " = std::chrono::duration_cast<std::chrono::duration<double>>(" + valueStr +
                   ").count();\n";
      }
      else
      {
        ss_ << "  " + cfgId + " = " + valueStr + ";\n";
      }
    }
  }

  std::string typeUcFirst()
  {
    switch (type_)
    {
      case Type::Min:
        return "Min";
      case Type::Max:
        return "Max";
      case Type::Default:
        return "Default";
    }
    return "";
  }

  std::string type()
  {
    auto _type = typeUcFirst();
    _type[0] = std::tolower(_type[0]);
    return _type;
  }

  std::string str()
  {
    const auto fullClassName = "zivid_camera::" + className_ + "Config";
    std::stringstream res;
    res << fullClassName << " get" << className_
        << "Config" + typeUcFirst() + "FromZividSettings(const Zivid::Settings& s)\n";
    res << "{\n";
    res << "  auto cfg = " + fullClassName + "::__get" + typeUcFirst() + "__();\n";
    res << ss_.str();
    res << "  return cfg;\n";
    res << "}\n";
    return res.str();
  }

private:
  std::string className_;
  Type type_;
  std::stringstream ss_;
};

class ConfigUtilsHeaderGenerator
{
public:
  ConfigUtilsHeaderGenerator(const std::string& className)
    : apply_config_zivid_settings_gen(className)
    , get_config_min_from_zivid_settings_gen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Min)
    , get_config_max_from_zivid_settings_gen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Max)
    , get_config_def_from_zivid_settings_gen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Default)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    apply_config_zivid_settings_gen.apply(s);
    get_config_min_from_zivid_settings_gen.apply(s);
    get_config_max_from_zivid_settings_gen.apply(s);
    get_config_def_from_zivid_settings_gen.apply(s);
  }

  std::string str()
  {
    std::stringstream res;
    res << "#pragma once\n\n";
    res << "// This is an auto-generated header. Do not edit.\n\n";
    res << apply_config_zivid_settings_gen.str() << "\n";
    res << get_config_min_from_zivid_settings_gen.str() << "\n";
    res << get_config_max_from_zivid_settings_gen.str() << "\n";
    res << get_config_def_from_zivid_settings_gen.str() << "\n";
    return res.str();
  }

  ApplyConfigToZividSettingsGenerator apply_config_zivid_settings_gen;
  GetConfigMinMaxDefFromZividSettingsGenerator get_config_min_from_zivid_settings_gen;
  GetConfigMinMaxDefFromZividSettingsGenerator get_config_max_from_zivid_settings_gen;
  GetConfigMinMaxDefFromZividSettingsGenerator get_config_def_from_zivid_settings_gen;
};

class Generator
{
public:
  Generator(const std::string& className) : dynamic_reconfigure_cfg_gen(className), config_utils_header_gen(className)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    dynamic_reconfigure_cfg_gen.apply(s);
    config_utils_header_gen.apply(s);
  }

  DynamicReconfigureCfgGenerator dynamic_reconfigure_cfg_gen;
  ConfigUtilsHeaderGenerator config_utils_header_gen;
};

template <typename ZividSetting>
void traverseGeneralSettingsTree(const ZividSetting& s, Generator& general_settings)
{
  if constexpr (ZividSetting::isContainer)
  {
    s.forEach([&](const auto& c) { traverseGeneralSettingsTree(c, general_settings); });
  }
  else
  {
    general_settings.apply(s);
  }
}

template <typename ZividSetting>
void traverseSettingsTree(const ZividSetting& s, Generator& general_settings, Generator& frame_settings)
{
  if constexpr (isGeneralSetting<ZividSetting>)
  {
    traverseGeneralSettingsTree(s, general_settings);
  }
  else if constexpr (ZividSetting::isContainer)
  {
    s.forEach([&](const auto& c) { traverseSettingsTree(c, general_settings, frame_settings); });
  }
  else
  {
    frame_settings.apply(s);
  }
}

void writeToFile(const std::string& file_name, const std::string& text)
{
  std::ofstream cfg_file;
  cfg_file.open(file_name);
  cfg_file << text;
  cfg_file.close();
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
  Generator capture_general_gen("CaptureGeneral");
  Generator capture_frame_gen("CaptureFrame");

  const auto settings = Zivid::Settings{};
  traverseSettingsTree(settings, capture_general_gen, capture_frame_gen);
  capture_frame_gen.dynamic_reconfigure_cfg_gen.insertEnabled();

  writeToFile("CaptureGeneral.cfg", capture_general_gen.dynamic_reconfigure_cfg_gen.str());
  writeToFile("CaptureFrame.cfg", capture_frame_gen.dynamic_reconfigure_cfg_gen.str());

  writeToFile("generated_headers/CaptureGeneralConfigUtils.h", capture_general_gen.config_utils_header_gen.str());
  writeToFile("generated_headers/CaptureFrameConfigUtils.h", capture_frame_gen.config_utils_header_gen.str());

  return 0;
}
