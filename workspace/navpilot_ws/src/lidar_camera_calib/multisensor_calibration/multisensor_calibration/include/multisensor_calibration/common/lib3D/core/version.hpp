/* lib3d::core::version.hpp */

#ifndef LIB3D_CORE_VERSION_HPP
#define LIB3D_CORE_VERSION_HPP

#define LIB3D_CORE_VERSION_STR 2.0.1.47.fb85c91
#define LIB3D_CORE_VERSION_MAJOR 2
#define LIB3D_CORE_VERSION_MINOR 0
#define LIB3D_CORE_VERSION_PATCH 1

// std
#include <string>

#include "common.h"

/**
 @namespace lib3d::core
 @brief Base namespace of lib3D_core.
 */
namespace lib3d
{
namespace core
{

  /**
   @brief Function to get current version string of library.
   */
  inline std::string getVersionStr()
  {
    return std::string(LIB3D_TOSTRING(LIB3D_CORE_VERSION_STR));
  }

  /**
   @brief Function to get current version major number of library.
   */
  inline int getVersionMajor()
  {
    return LIB3D_CORE_VERSION_MAJOR;
  }

  /**
   @brief Function to get current version minor number of library.
   */
  inline int getVersionMinor()
  {
    return LIB3D_CORE_VERSION_MINOR;
  }

  /**
   @brief Function to get current version path number of library.
   */
  inline int getVersionPatch()
  {
    return LIB3D_CORE_VERSION_PATCH;
  }

} // namespace core
} // namespace lib3d

#endif // LIB3D_CORE_VERSION_HPP

