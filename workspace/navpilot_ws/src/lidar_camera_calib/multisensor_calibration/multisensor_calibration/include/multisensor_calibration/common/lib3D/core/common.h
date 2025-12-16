#ifndef LIB3D_COMMON_H
#define LIB3D_COMMON_H

// Std
#define _USE_MATH_DEFINES
#include <cmath>

// opencv
#include <opencv2/core/version.hpp>

#define CV_VERSION_CONCAT (CV_VERSION_MAJOR*100 + CV_VERSION_MINOR*10 + CV_VERSION_REVISION)

#define LIB3D_STRINGIFY(x) #x
#define LIB3D_TOSTRING(x) LIB3D_STRINGIFY(x)

#endif // LIB3D_COMMON_H
