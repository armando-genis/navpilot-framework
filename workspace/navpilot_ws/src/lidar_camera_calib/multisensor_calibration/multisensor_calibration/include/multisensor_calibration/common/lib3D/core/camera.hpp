#ifndef LIB3D_CAMERA_H
#define LIB3D_CAMERA_H

// Std
#include <cmath>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// lib3D
#include "extrinsics.hpp"
#include "intrinsics.hpp"

namespace lib3d
{

/**
 @ingroup camera_geom
 @brief This class implements a camera models, made up of intrinsic and extrinsic parameters
 (lib3d::Intrinsics & lib3d::Extrinsics).

 The intrinsic parameters allow to model a perspective or parallel camera with or without distortion.
 The distortion models are implemented by OpenCV and supports radial, tangential and prism distortion.
 For more information see [OpenCV-Website](https://docs.opencv.org/master/d9/d0c/group__calib3d.html).

 <h3>Example</h3>

 Initialization of a perspecitve camera with no distortion (pinhole camera) and no rotation or
 translation use:

 @code{.cpp}

  lib3d::Camera pinholeCamera(lib3d::Intrinsics(
                               cv::Size(1920, 1080),          // image size
                               cv::Point2d(1273.38, 1231.75), // focal length
                               cv::Point2d(960,540)           // principal point
                              ));
 @endcode

 */
class Camera
{

    //--- METHOD DECLERATION ---//

  public:
    /**
     @brief Default (zero initialization) constructor.

     This will initialize the camera with no intrinsic and extrinsic parameters.
     */
    explicit Camera()
        : intrinsics(Intrinsics()), extrinsics(Extrinsics())
    {
    }

    /**
     @brief Initialize camera with given intrinsic parameters
     @param[in] intr Intrinsic camera parameters.
     */
    explicit Camera(const Intrinsics& intr)
        : intrinsics(intr), extrinsics(Extrinsics())
    {
    }

    /**
     @brief Initialize camera with given intrinsic and extrinsic parameters
     @param[in] intr Intrinsic camera parameters.
     @param[in] extr Extrinsic camera parameters.
     */
    explicit Camera(const Intrinsics& intr, const Extrinsics& extr)
        : intrinsics(intr), extrinsics(extr)
    {
    }

    /**
     @brief Copy constructor
     */
    Camera(const Camera& rhs)
    {
        intrinsics = rhs.intrinsics;
        extrinsics = rhs.extrinsics;
    }

    /**
     @brief Move constructor.
     */
    Camera(Camera&& rhs)
    {
        intrinsics = rhs.intrinsics;
        extrinsics = rhs.extrinsics;
    }

    /**
     @brief Copy assignment operator.
     */
    Camera& operator=(const Camera& rhs)
    {

        intrinsics = rhs.intrinsics;
        extrinsics = rhs.extrinsics;

        return *this;
    }

    /**
     @brief Move assignment operator.
     */
    Camera& operator=(Camera&& rhs)
    {
        if (this != &rhs)
        {
            intrinsics = rhs.intrinsics;
            extrinsics = rhs.extrinsics;
        }

        return *this;
    }

    /**
     @brief Comparison operator.
     @returns True, if the intrinsic, distortion and extrinsic parameters are equal.
     */
    bool operator==(const Camera& rhs)
    {
        return (this->intrinsics.getImageSize() == rhs.intrinsics.getImageSize() &&
                this->intrinsics.getK_as3x3() == rhs.intrinsics.getK_as3x3() &&
                cv::norm(this->intrinsics.getDistortionCoeffs() - rhs.intrinsics.getDistortionCoeffs()) == 0 &&
                this->extrinsics == rhs.extrinsics);
    }

    /**
     @brief Comparison operator.
     @returns True, if the intrinsic, distortion and extrinsic parameters are NOT equal.
     */
    bool operator!=(const Camera& rhs)
    {
        return !(*this == rhs);
    }

    /**
     @brief Method to project a 3D world point into 2D pixel coordinates.

     This will first transform the world point into local camera coordinates by applying
     `extrinsics.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL)` to the world point. Then it will perform
     a projection by getting the specific projeciton model from the instrinsic parameters. This will
     yield a point in image coordinates on the focal plance, centered in the principle point. This
     image point is then projected into pixel coordinates according to the calibration matrix \f$\mathrm{K}\f$.

     This function will internally use the function `projectPoints` from OpenCV as this also considers
     the distortion parameters.

     Also see: https://docs.opencv.org/master/d9/d0c/group__calib3d.html
     */
    cv::Point2i projectWorld2Pixel(cv::Point3f worldPnt) const
    {
        //--- pixel coordinates to return
        cv::Point2i retPx;

        //--- specifiy input and output vectors for cv::project points
        std::vector<cv::Point3f> vecWorldPnt = {worldPnt};
        std::vector<cv::Point2f> vecImgPnt;

        //--- get extrinsics as ref to local
        Extrinsics tmpExtr(extrinsics.getRTMatrix(Extrinsics::REF_2_LOCAL));

        //--- call cv::projectPoints
        cv::projectPoints(vecWorldPnt, tmpExtr.getRotationVec(), tmpExtr.getTranslationVec(),
                          intrinsics.getK_as3x3(), intrinsics.getDistortionCoeffs(), vecImgPnt);

        //--- transform to pixel by rounding image points
        retPx.x = static_cast<int>(std::roundf(vecImgPnt.front().x));
        retPx.y = static_cast<int>(std::roundf(vecImgPnt.front().y));

        return retPx;
    }

    /**
     @overload
     */
    cv::Point2i projectWorld2Pixel(float worldPnt_x, float worldPnt_y, float worldPnt_z) const
    {
        return projectWorld2Pixel(cv::Point3f(worldPnt_x, worldPnt_y, worldPnt_z));
    }

    /**
     @brief Method to project a 3D point given in local coordinates of the camera into 2D pixel coordinates.

     This is similar to projectWorld2Pixel, only that it omits the transformation with `extrinsics.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL)`.
     In this it es assumed that the local point is given in the coordinate system of the camera with
     the x-axis pointing to the right, the y-axis to the bottom and the z-axis to the viewing direction
     of the camera.

     This function will internally use the function `projectPoints` from OpenCV as this also considers
     the distortion parameters.

     Also see: https://docs.opencv.org/master/d9/d0c/group__calib3d.html

     */
    cv::Point2i projectLocal2Pixel(cv::Point3f localPnt) const
    {
        //--- pixel coordinates to return
        cv::Point2i retPx;

        //--- specifiy input and output vectors for cv::project points
        std::vector<cv::Point3f> vecLocalPnt = {localPnt};
        std::vector<cv::Point2f> vecImgPnt;

        //--- call cv::projectPoints
        cv::projectPoints(vecLocalPnt, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                          intrinsics.getK_as3x3(), intrinsics.getDistortionCoeffs(), vecImgPnt);

        //--- transform to pixel by rounding image points
        retPx.x = static_cast<int>(std::roundf(vecImgPnt.front().x));
        retPx.y = static_cast<int>(std::roundf(vecImgPnt.front().y));

        return retPx;
    }

    /**
     @overload
     */
    cv::Point2i projectLocal2Pixel(float localPnt_x, float localPnt_y, float localPnt_z) const
    {
        return projectLocal2Pixel(cv::Point3f(localPnt_x, localPnt_y, localPnt_z));
    }

    //--- MEMBER DECLERATION ---//

  public:
    /// Intrinsic camera parameters
    Intrinsics intrinsics;

    /// Extrinsic camera parameters
    Extrinsics extrinsics;
};

/**
 @ingroup camera_geom
 @brief Compute fundamental matrix between two cameras.
 @param[in] leftExtr Extrinsic parameters of left camera
 @param[in] leftIntr Intrinsic parameters of left camera
 @param[in] rightExtr Extrinsic parameters of right camera
 @param[in] rightIntr Intrinsic parameters of right camera
 @return Fundamental matrix
 */
inline cv::Mat computeFundamentalMatrix(lib3d::Extrinsics leftExtr, lib3d::Intrinsics leftIntr,
                                        lib3d::Extrinsics rightExtr, lib3d::Intrinsics rightIntr)
{
    return computeFundamentalMatrix(leftExtr.getRTMatrix(Extrinsics::REF_2_LOCAL), leftIntr.getK_as3x3(),
                                    rightExtr.getRTMatrix(Extrinsics::REF_2_LOCAL), rightIntr.getK_as3x3());
}

/**
 @ingroup camera_geom
 @brief Compute polar coordinates for a ray from a given pixel in the camera home pose reference frame
 @param[in] intrinsics camera intrinsics
 @param[in] x target pixel x-coordinate
 @param[in] y target pixel y-coordinate
 @param[in] pan camera pan value (yaw) in radians
 @param[in] tilt camera tilt value (pitch) in radians
 @param[out] outAzimuth azimuth of the ray in radians (0 is camera z-axis)
 @param[out] outElevation elevation of the ray in radians (0 is camera z-axis)
 */
inline void computePolarCoordinates(lib3d::Intrinsics intrinsics, double x, double y,
                                    double pan, double tilt, double& outAzimuth,
                                    double& outElevation)
{
    cv::Matx33d K = intrinsics.getK_as3x3().inv();

    cv::Matx33d xCompensation = Rotation::createRotationX_rad(tilt);
    cv::Matx33d yCompensation = Rotation::createRotationY_rad(pan);

    cv::Matx33d ptPose = yCompensation * xCompensation;

    cv::Point3d ray(x, y, 1.0);

    ray = cv::Point3d(cv::Mat(ptPose * K * cv::Mat(ray)));

    double length = cv::norm(ray);
    ray /= length;

    cv::Point3d rayOnXZ(ray.x, 0.0, ray.z);

    double lengthRayProjected = cv::norm(rayOnXZ);
    rayOnXZ /= lengthRayProjected;

    cv::Point3d z(0.0, 0.0, 1.0);

    outAzimuth = acos(rayOnXZ.dot(z));
    if (rayOnXZ.x < 0)
    {
        outAzimuth *= -1.0;
    }

    outElevation = acos(rayOnXZ.dot(ray));
    if (ray.y > 0)
    {
        outElevation *= -1.0;
    }
}

/**
 @ingroup camera_geom
 @brief Compute polar coordinates for a ray from a given pixel in the camera home pose reference frame
 @param[in] intrinsics camera intrinsics
 @param[in] x target pixel x-coordinate
 @param[in] y target pixel y-coordinate
 @param[in] pan camera pan value (yaw) in radians
 @param[in] tilt camera tilt value (pitch) in radians
 @param[in] zoom camera zoom value in [0,1]
 @param[in] minF
 @param[in] maxF
 @param[out] outAzimuth azimuth of the ray in radians (0 is camera z-axis)
 @param[out] outElevation elevation of the ray in radians (0 is camera z-axis)
 @param[out] outCompensatedIntrinsics zoom-adapted intrinsics
 */
inline void computePolarCoordinates(lib3d::Intrinsics intrinsics, double x, double y,
                                    double pan, double tilt, double zoom, double minF,
                                    double maxF, double& outAzimuth, double& outElevation,
                                    Intrinsics& outCompensatedIntrinsics)
{
    Intrinsics::updateIntrinsicFocalLength(intrinsics, zoom, minF, maxF, outCompensatedIntrinsics);

    computePolarCoordinates(outCompensatedIntrinsics, x, y, pan, tilt, outAzimuth, outElevation);
}

} // namespace lib3d

#endif // LIB3D_CAMERA_H
