#ifndef PARSE_SETTINGS_H
#define PARSE_SETTINGS_H

#include <dynamic_reconfigure/config_tools.h>

#include <cctype>
#include <string>
#include <chrono>

template <typename SettingType>
struct AssertValue
{
  static constexpr bool value = false;
};

template <typename SettingType, typename ConfigType>
struct AssertPair
{
  static constexpr bool value = false;
};

// The input string is a hierarchial description of a setting, separated by /
// For example Filters/Contrast/Threshold
// The function replaces '/' with '_' and inserts underscore before capital
// Letter (no the first) before decapitalizing it.
std::string convertSettingsPathToConfigPath(const std::string& path_string)
{
  std::string return_string;

  for (int i = 0; i < path_string.size(); i++)
  {
    char c = path_string[i];
    bool insert_underscore = false;
    bool replace_with_underscore = false;
    bool skip_char = false;

    // Never start with underscore
    if (i > 0)
    {
      bool previous_is_underscore = return_string.size() > 0 && return_string[return_string.size() - 1] == '_';
      if (c == '/' || c == '\\' || c == '_')
        replace_with_underscore = true;
      else if (std::isupper(c))
        insert_underscore = true;

      if (previous_is_underscore)
        insert_underscore = false;
      if (previous_is_underscore && replace_with_underscore)
        skip_char = true;
    }

    if (!skip_char)
    {
      if (insert_underscore || replace_with_underscore)
        return_string += "_";
      if (!replace_with_underscore)
        return_string += std::tolower(c);
    }
  }

  return return_string;
}

// Template for type conversion. The Config files in the dynamic_reconfigure package in ros
// only support the types bool, int, double and string. The Zivid settings use different
// base types, and must therefore be converted. Each Zivid setting base type have a
// corresponding config type which is converted through a specialization of this class.
template <typename SettingsType, typename ConfigType>
class ConfigSettingsValueConversion
{
public:
  static SettingsType configValueToSettingsValue(const ConfigType& config_value)
  {
    static_assert(AssertPair<SettingsType, ConfigType>::value, "TODO false A config to settings value conversion must "
                                                               "be added for this setting type. A settings type must "
                                                               "be converted to either int, bool, double or "
                                                               "std::string");
  }
  static ConfigType settingsValueToConfigValue(const SettingsType& settings_value)
  {
    static_assert(AssertPair<SettingsType, ConfigType>::value, "TODO false A settings to config value conversion must "
                                                               "be added for this setting type. It must be converted "
                                                               "from either int, bool, double or std::string");
  }
};

template <>
class ConfigSettingsValueConversion<bool, bool>
{
public:
  static bool configValueToSettingsValue(const bool& config_value)
  {
    return config_value;
  }
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
  static double configValueToSettingsValue(const double& config_value)
  {
    return config_value;
  }
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
  static std::chrono::microseconds configValueToSettingsValue(const double& config_value)
  {
    std::chrono::duration<double> duration(config_value);
    return std::chrono::duration_cast<std::chrono::microseconds>(duration);
  }
  static double settingsValueToConfigValue(const std::chrono::microseconds& settings_value)
  {
    return ((double)settings_value.count()) / 1000000.0;
  }
  static constexpr const char* configValueType = "double_t";
};

template <>
class ConfigSettingsValueConversion<size_t, int>
{
public:
  static size_t configValueToSettingsValue(const int& config_value)
  {
    return (size_t)config_value;
  }
  static int settingsValueToConfigValue(const size_t& settings_value)
  {
    return (int)settings_value;
  }
  static constexpr const char* configValueType = "int_t";
};

// Template for reading a config message by looking up on a key, and return the corresponding
// Settings value which is converted using the ConfigSettingsValueConversion class.
// This template specialized for each type in the config system which is bool, int, double and string.
// You will probably never have to add new types here unless the dynamic_reconfigure package have added more types.
template <typename SettingsType, typename ConfigType>
class ConfigReaderToSettings
{
public:
  static SettingsType getConfigValueFromSetting(const std::string& key, const dynamic_reconfigure::Config& config)
  {
    static_assert(AssertValue<ConfigType>::value, "No config value reader found for this config type. The currently "
                                                  "only supported types are bool, int, double and string");
  }
};
template <typename SettingsType>
class ConfigReaderToSettings<SettingsType, bool>
{
public:
  static SettingsType getConfigValueFromSetting(const std::string& key, const dynamic_reconfigure::Config& config)
  {
    for (int i = 0; i < config.bools.size(); i++)
    {
      if (config.bools[i].name == key)
        return ConfigSettingsValueConversion<SettingsType, bool>::configValueToSettingsValue(config.bools[i].value);
    }
    throw std::invalid_argument("The key argument " + key +
                                " is not found in the config message. Is both ROS wrapper and Zivid API up to date?");
  }
};
template <typename SettingsType>
class ConfigReaderToSettings<SettingsType, double>
{
public:
  static SettingsType getConfigValueFromSetting(const std::string& key, const dynamic_reconfigure::Config& config)
  {
    for (int i = 0; i < config.doubles.size(); i++)
    {
      if (config.doubles[i].name == key)
        return ConfigSettingsValueConversion<SettingsType, double>::configValueToSettingsValue(config.doubles[i].value);
    }
    throw std::invalid_argument("The key argument " + key +
                                " is not found in the config message. Is both ROS wrapper and Zivid API up to date?");
  }
};
template <typename SettingsType>
class ConfigReaderToSettings<SettingsType, int>
{
public:
  static SettingsType getConfigValueFromSetting(const std::string& key, const dynamic_reconfigure::Config& config)
  {
    for (int i = 0; i < config.ints.size(); i++)
    {
      if (config.ints[i].name == key)
        return ConfigSettingsValueConversion<SettingsType, int>::configValueToSettingsValue(config.ints[i].value);
    }
    throw std::invalid_argument("The key argument " + key +
                                " is not found in the config message. Is both ROS wrapper and Zivid API up to date?");
  }
};
template <typename SettingsType>
class ConfigReaderToSettings<SettingsType, std::string>
{
public:
  static SettingsType getConfigValueFromSetting(const std::string& key, const dynamic_reconfigure::Config& config)
  {
    for (const auto& string_config : config.strs)
    {
      if (string_config.name == key)
        return ConfigSettingsValueConversion<SettingsType, std::string>::configValueToSettingsValue(
            string_config.value);
    }
    throw std::invalid_argument("The key argument " + key +
                                " is not found in the config message. Is both ROS wrapper and Zivid API up to date?");
  }
};

// Template for getting a settings value from a config message by looking up on the config value name.
// This function specifices which types each Zivid setting should be converted to, which is then handled
// By the remaining functions.
template <typename SettingsType>
SettingsType getConfigValueFromString(const std::string& key, const dynamic_reconfigure::Config& config)
{
  if constexpr (std::is_same_v<SettingsType, bool>)
    return ConfigReaderToSettings<SettingsType, bool>::getConfigValueFromSetting(key, config);
  else if constexpr (std::is_same_v<SettingsType, double>)
    return ConfigReaderToSettings<SettingsType, double>::getConfigValueFromSetting(key, config);
  else if constexpr (std::is_same_v<SettingsType, std::chrono::microseconds>)
    return ConfigReaderToSettings<SettingsType, double>::getConfigValueFromSetting(key, config);
  else if constexpr (std::is_same_v<SettingsType, size_t>)
    return ConfigReaderToSettings<SettingsType, int>::getConfigValueFromSetting(key, config);
  else
    static_assert(AssertValue<SettingsType>::value, "Setting to config type convertion is not specified. You must "
                                                    "specify which config type each settings type should be converted "
                                                    "to. Options are: bool, int, double and string");
}
#endif  // PARSE_SETTINGS_H
