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

template <typename ZividSetting, typename ConfigType>
struct AssertPair
{
  static constexpr bool value = false;
};

// Template for type conversion. The Config files in the dynamic_reconfigure package in ros
// only support the types bool, int, double and string. The Zivid settings use different
// base types, and must therefore be converted. Each Zivid setting base type have a
// corresponding config type which is converted through a specialization of this class.
template <typename SettingsType, typename ConfigType>
class ConfigSettingsValueConversion
{
public:
  static ConfigType settingsValueToConfigValue(const SettingsType& settings_value)
  {
    static_assert(AssertPair<SettingsType, ConfigType>::value, "A settings to config value conversion must "
                                                               "be added for this setting type. It must be converted "
                                                               "from either int, bool, double or std::string");
  }
};

template <>
class ConfigSettingsValueConversion<bool, bool>
{
public:
  static bool settingsValueToConfigValue(const bool& settings_value)
  {
    return settings_value;
  }
  static constexpr const char* configValueType = "bool_t";
};

template <>
class ConfigSettingsValueConversion<double, double>
{
public:
  static double settingsValueToConfigValue(const double& settings_value)
  {
    return settings_value;
  }
  static constexpr const char* configValueType = "double_t";
};

template <>
class ConfigSettingsValueConversion<std::chrono::microseconds, double>
{
public:
  static double settingsValueToConfigValue(const std::chrono::microseconds& settings_value)
  {
    return std::chrono::duration_cast<std::chrono::duration<double>>(settings_value).count();
  }
  static constexpr const char* configValueType = "double_t";
};

template <>
class ConfigSettingsValueConversion<size_t, int>
{
public:
  static int settingsValueToConfigValue(const size_t& settings_value)
  {
    return (int)settings_value;
  }
  static constexpr const char* configValueType = "int_t";
};

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

std::string convertSettingsPathToZividTypeName(const std::string& path)
{
  const auto typeName = boost::replace_all_copy<std::string>(path, "/", "::");
  return "Zivid::Settings::" + typeName;
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

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    const auto setting_name = convertSettingsPathToConfigPath(s.path);
    const auto level = "0";
    std::string description = "";

    std::string type_name;
    std::string default_text;

    if constexpr (std::is_same_v<typename ZividSetting::ValueType, bool>)
    {
      default_text =
          ConfigSettingsValueConversion<bool, bool>::settingsValueToConfigValue(s.value()) ? "True" : "False";
      type_name = getTypeNameText<ZividSetting, bool>();
    }
    else if constexpr (std::is_same_v<typename ZividSetting::ValueType, double>)
    {
      default_text = getRangeValueText<ZividSetting, double>(s);
      type_name = getTypeNameText<ZividSetting, double>();
    }
    else if constexpr (std::is_same_v<typename ZividSetting::ValueType, size_t>)
    {
      default_text = getRangeValueText<ZividSetting, int>(s);
      type_name = getTypeNameText<ZividSetting, int>();
    }
    else if constexpr (std::is_same_v<typename ZividSetting::ValueType, std::chrono::microseconds>)
    {
      default_text = getRangeValueText<ZividSetting, double>(s);
      type_name = getTypeNameText<ZividSetting, double>();
    }
    else
    {
      static_assert(AssertValue<typename ZividSetting::ValueType>::value, "A default text and type name is required "
                                                                          "for "
                                                                          "type "
                                                                          "ZividSetting::ValueType");
    }

    ss_ << "gen.add(\"" + setting_name + "\", " + type_name + ", " + level + ", " + "\"" + description + "\", " +
               default_text + ")\n";
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
    const auto zividClassName = convertSettingsPathToZividTypeName(s.path);
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
    const auto zividClassName = convertSettingsPathToZividTypeName(s.path);

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

  std::string typeUc()
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
    auto _type = typeUc();
    _type[0] = std::tolower(_type[0]);
    return _type;
  }

  std::string str()
  {
    const auto fullClassName = "zivid_camera::" + className_ + "Config";
    std::stringstream res;
    res << fullClassName << " get" << className_
        << "Config" + typeUc() + "FromZividSettings(const Zivid::Settings& s)\n";
    res << "{\n";
    res << "  auto cfg = " + fullClassName + "::__get" + typeUc() + "__();\n";
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
    : applyConfigToZividSettingsGen(className)
    , getConfigMinFromZividSettingsGen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Min)
    , getConfigMaxFromZividSettingsGen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Max)
    , getConfigDefFromZividSettingsGen(className, GetConfigMinMaxDefFromZividSettingsGenerator::Type::Default)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    applyConfigToZividSettingsGen.apply(s);
    getConfigMinFromZividSettingsGen.apply(s);
    getConfigMaxFromZividSettingsGen.apply(s);
    getConfigDefFromZividSettingsGen.apply(s);
  }

  std::string str()
  {
    std::stringstream res;
    res << "#pragma once\n\n";
    res << "// This is an auto-generated header. Do not edit.\n\n";
    res << applyConfigToZividSettingsGen.str() << "\n";
    res << getConfigMinFromZividSettingsGen.str() << "\n";
    res << getConfigMaxFromZividSettingsGen.str() << "\n";
    res << getConfigDefFromZividSettingsGen.str() << "\n";
    return res.str();
  }

  ApplyConfigToZividSettingsGenerator applyConfigToZividSettingsGen;
  GetConfigMinMaxDefFromZividSettingsGenerator getConfigMinFromZividSettingsGen;
  GetConfigMinMaxDefFromZividSettingsGenerator getConfigMaxFromZividSettingsGen;
  GetConfigMinMaxDefFromZividSettingsGenerator getConfigDefFromZividSettingsGen;
};

class Generator
{
public:
  Generator(const std::string& className) : dynamicReconfigureCfgGenerator(className), configUtilsHeaderGen(className)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    dynamicReconfigureCfgGenerator.apply(s);
    configUtilsHeaderGen.apply(s);
  }

  DynamicReconfigureCfgGenerator dynamicReconfigureCfgGenerator;
  ConfigUtilsHeaderGenerator configUtilsHeaderGen;
};

template <typename ZividSetting>
void traverseGeneralSettingsTree(const ZividSetting& s, Generator& generalSettings)
{
  if constexpr (ZividSetting::isContainer)
  {
    s.forEach([&](const auto& c) { traverseGeneralSettingsTree(c, generalSettings); });
  }
  else
  {
    generalSettings.apply(s);
  }
}

template <typename ZividSetting>
void traverseSettingsTree(const ZividSetting& s, Generator& generalSettings, Generator& frameSettings)
{
  if constexpr (isGeneralSetting<ZividSetting>)
  {
    traverseGeneralSettingsTree(s, generalSettings);
  }
  else if constexpr (ZividSetting::isContainer)
  {
    s.forEach([&](const auto& c) { traverseSettingsTree(c, generalSettings, frameSettings); });
  }
  else
  {
    frameSettings.apply(s);
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
  Generator captureGeneralGen("CaptureGeneral");
  Generator captureFrameGen("CaptureFrame");

  const auto settings = Zivid::Settings{};
  traverseSettingsTree(settings, captureGeneralGen, captureFrameGen);
  captureFrameGen.dynamicReconfigureCfgGenerator.insertEnabled();

  writeToFile("CaptureGeneral.cfg", captureGeneralGen.dynamicReconfigureCfgGenerator.str());
  writeToFile("CaptureFrame.cfg", captureFrameGen.dynamicReconfigureCfgGenerator.str());

  writeToFile("generated_headers/CaptureGeneralConfigUtils.h", captureGeneralGen.configUtilsHeaderGen.str());
  writeToFile("generated_headers/CaptureFrameConfigUtils.h", captureFrameGen.configUtilsHeaderGen.str());

  return 0;
}
