#ifndef LIB3D_GEOMETRY_H
#define LIB3D_GEOMETRY_H

// Std
#define _USE_MATH_DEFINES
#include <cmath>

// OpenCV
#include <opencv2/core.hpp>

#include "exceptions.hpp"

#define LIB3D_EPS 1e-8

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace lib3d {

  /**
   @ingroup camera_geom
   */
  inline double interpolateLinear(double _value, double _minS, double _maxS,
                                                    double _minD, double _maxD)
  {
     if ( _value > _maxS )
     {
         _value = _maxS;
     }
     if ( _value < _minS )
     {
         _value = _minS;
     }

     return ((_maxD - _minD) / (_maxS - _minS) * ( _value - _minS) + _minD);
  }

  /**
   @ingroup camera_geom
   @brief Convert degree to radian.
   */
  inline double degreeToRadian(double degree)
  {
      return ( degree ) * (M_PI / 180.0);
  }

  /**
   @ingroup camera_geom
   @brief Convert radian to degree.
   */
  inline double radianToDegree(double radian)
  {
      return ( radian ) * (180.0 / M_PI);
  }

  /**
   @ingroup camera_geom
   @brief Get the uniform scaling factor of a transformation matrix
   @param[in] transform 3x3 or 4x4 homogenous transformation matrix
   @return scaling factor
   */
  inline double getScale(cv::Mat transform)
  {
    if(transform.size() != cv::Size(3,3) && transform.size() != cv::Size(4,4))
      throw(lib3d::InvalidArgumentException(FN_NAME, "(transform.size() != cv::Size(3,3) && transform.size() != cv::Size(4,4))"));

    cv::Mat R = transform.rowRange(0, 3).colRange(0, 3);
    return std::sqrt(cv::trace(R.t() * R)[0] / 3.0 );
  }

  /**
   @ingroup camera_geom
   @brief Compute fundamental matrix between two cameras.
   @param[in] leftExtrRT 4x4 extrinsic matrix of left camera
   @param[in] leftIntrK 3x3 calibration matrix of left camera
   @param[in] rightExtrRT 4x4 extrinsic matrix of right camera
   @param[in] rightIntrK 3x3 calibration matrix of right camera
   @return Fundamental matrix
   */
  inline cv::Mat computeFundamentalMatrix(cv::Matx44d leftExtrRT, cv::Matx33d leftIntrK,
                                   cv::Matx44d rightExtrRT, cv::Matx33d rightIntrK)
  {
      cv::Mat t = cv::Mat(rightExtrRT.inv() * cv::Mat(leftExtrRT).colRange(3, 4)).rowRange(0, 3);

      //--- t_x is cross product matrix representation of t (skew symmetric) ---
      cv::Mat t_x = cv::Mat::zeros(3, 3, t.type());

      if (t.type() == CV_32F)
      {
          t_x.at<float>(1, 0) = t.at<float>(2, 0);
          t_x.at<float>(2, 0) = -1.0*t.at<float>(1, 0);

          t_x.at<float>(0, 1) = -1.0*t.at<float>(2, 0);
          t_x.at<float>(2, 1) = t.at<float>(0,0);

          t_x.at<float>(0, 2) = t.at<float>(1, 0);
          t_x.at<float>(1, 2) = -1.0*t.at<float>(0, 0);
      }
      else if (t.type() == CV_64F)
      {
          t_x.at<double>(1, 0) = t.at<double>(2, 0);
          t_x.at<double>(2, 0) = -1.0*t.at<double>(1, 0);

          t_x.at<double>(0, 1) = -1.0*t.at<double>(2, 0);
          t_x.at<double>(2, 1) = t.at<double>(0,0);

          t_x.at<double>(0, 2) = t.at<double>(1, 0);
          t_x.at<double>(1, 2) = -1.0*t.at<double>(0, 0);
      }

      cv::Mat R = cv::Mat(rightExtrRT).rowRange(0,3).colRange(0,3).inv() *
          cv::Mat(leftExtrRT).rowRange(0,3).colRange(0,3);

      cv::Mat E = t_x * R;

      cv::Matx33d K_cam_inv = rightIntrK.inv();

      cv::Matx33d K_ref_inv = leftIntrK.inv();

      cv::Mat outF = (K_cam_inv.t())*E*(K_ref_inv);

      return outF;
  }

  /**
   @ingroup camera_geom
   @brief Compute fundamental matrix between two cameras.
   @param[in] leftExtrRT 4x4 extrinsic matrix of left camera
   @param[in] leftIntrK 3x3 calibration matrix of left camera
   @param[in] rightExtrRT 4x4 extrinsic matrix of right camera
   @param[in] rightIntrK 3x3 calibration matrix of right camera
   @return Fundamental matrix
   */
  inline cv::Mat computeFundamentalMatrix(cv::Mat leftExtrRT, cv::Mat leftIntrK,
                                   cv::Mat rightExtrRT, cv::Mat rightIntrK)
  {
    return computeFundamentalMatrix(cv::Matx44d(leftExtrRT), cv::Matx33d(leftIntrK),
                                    cv::Matx44d(rightExtrRT), cv::Matx33d(rightIntrK));
  }

  /**
   @ingroup camera_geom
   @brief Finds the intersection of two lines, or returns fals. The lines are defined with origin and
   direction as (o1, d1) and (o2, d2).
   @see adapted from https://stackoverflow.com/a/7448287
   */
  inline bool intersect2D(cv::Point2f o1, cv::Point2f d1, cv::Point2f o2, cv::Point2f d2,
                   cv::Point2f &r)
  {
    cv::Point2f x = o2 - o1;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < LIB3D_EPS)
      return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
  }

  /**
   @ingroup camera_geom
   @brief Calculate the line segment PaPb that is the shortest route between two lines P1P2 and P3P4.

    Calculate also the values of mua and mub where
        Pa = P1 + mua (P2 - P1)
        Pb = P3 + mub (P4 - P3)
    Return FALSE if no solution exists.

   @see adapted from http://paulbourke.net/geometry/pointlineplane/
   */
  inline bool intersect3D (cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f p4, cv::Point3f& outA, cv::Point3f& outB, double &muA, double &muB)
  {
    cv::Point3f p13,p43,p21;
    double d1343,d4321,d1321,d4343,d2121;
    double numer,denom;

    p13.x = p1.x - p3.x;
    p13.y = p1.y - p3.y;
    p13.z = p1.z - p3.z;
    p43.x = p4.x - p3.x;
    p43.y = p4.y - p3.y;
    p43.z = p4.z - p3.z;
    if (abs(p43.x) < LIB3D_EPS && abs(p43.y) < LIB3D_EPS && abs(p43.z) < LIB3D_EPS)
      return false;
    p21.x = p2.x - p1.x;
    p21.y = p2.y - p1.y;
    p21.z = p2.z - p1.z;
    if (abs(p21.x) < LIB3D_EPS && abs(p21.y) < LIB3D_EPS && abs(p21.z) < LIB3D_EPS)
      return false;

    d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
    d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
    d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
    d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
    d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

    denom = d2121 * d4343 - d4321 * d4321;
    if (abs(denom) < LIB3D_EPS)
      return false;
    numer = d1343 * d4321 - d1321 * d4343;

    muA = numer / denom;
    muB = (d1343 + d4321 * muA) / d4343;

    outA.x = p1.x + muA * p21.x;
    outA.y = p1.y + muA * p21.y;
    outA.z = p1.z + muA * p21.z;
    outB.x = p3.x + muB * p43.x;
    outB.y = p3.y + muB * p43.y;
    outB.z = p3.z + muB * p43.z;

    return true;
  }

  /**
   @ingroup camera_geom
   @brief Function to compute homographic mapping between two cameras that is induces by a scene plane.

   The two cameras are parameterized by the calibration matrices (\f$\mathbf{K}_1, \mathbf{K}_2\f$) and
   the relative rotation and translation (\f$\mathbf{R}, \mathrm{t}\f$). The scene plane is specified by
   a normal vector \f$\mathrm{n}\f$ and the distance \f$d\f$ to the optical center of the first camera.
   (cf. @cite Hartley2004 pp. 325)
   @param[in] K1 Calibration matrix of first camera.
   @param[in] K2 Calibration matrix of second camera.
   @param[in] R Relative rotation matrix.
   @param[in] t Relative translation vector.
   @param[in] n Normal vector of scene plane that induces the homography.
   @param[in] d Distance of scene plane from the optical center of the first camera.
   @return \f$3\times 3\f$ Homography Matrix
   */
  inline cv::Matx33d computeHomography(const cv::Matx33d &K1, const cv::Matx33d &K2,  const cv::Matx33d &R,
                                const cv::Vec3d &t, const cv::Vec3d &n, const double& d)
  {
    cv::Matx33d H;
    cv::Matx33d tmp;

    cv::divide(t * n.t(), d, tmp);

    H = K1 * (R - tmp) * K2.inv();

    return H;
  }

  /**
   @ingroup camera_geom
   @overload
   */
  inline cv::Matx33d computeHomography(const cv::Matx33d &K, const cv::Matx33d &R,
                                const cv::Vec3d &t, const cv::Vec3d &n, const double& d)
  {
    return computeHomography(K, K, R, t, n, d);
  }

  /**
   @ingroup camera_geom
    @brief Function to warp pixel according to given homography. (cf. @cite Hartley2004 pp. 325)
    @param[in] iPixel Pixel to warp.
    @param[in] iHomography Homography accroding to which the pixel is to be warped.
    @return Coordinates of warped pixel with subpixel accuracy.
    */
  inline cv::Vec2d warpPixel(const cv::Vec2i& iPixel, const cv::Matx33d& iHomography)
  {
    cv::Vec2d warpedPixel;

    //--- homogeneous input pixel ---
    cv::Vec3d _pixel(iPixel(0), iPixel(1), 1);

    //--- warp pixel ---
    cv::Vec3d _warpedPixel = iHomography * _pixel;

    //--- compute inhomogeneous warped pixel ---
    warpedPixel(0) = _warpedPixel(0) / _warpedPixel(2);
    warpedPixel(1) = _warpedPixel(1) / _warpedPixel(2);

    return warpedPixel;
  }

  /**
   @ingroup camera_geom
   @brief Reproject image point onto scene plane with given parameters.

   In detail, this correponds to computing a ray from the optical center of camera through the given image point
   and then intersecting this ray with the scene plane, yielding a 3D scene point.
   @param[in] iK Calibration matrix of the camera from which the pixel is to be reprojected.
   @param[in] iImgPnt Image point that is to be reprojected.
   @param[in] iPlaneNormal Normal vector of the scene plane.
   @param[in] iPlaneDistance Distance of the scene plane from the optical center of the camera.
   @return 3D coordinates of reprojected point.
   */
  inline cv::Vec3d reprojectImagePointOntoPlane(const cv::Matx33d iK, const cv::Vec2d &iImgPnt,
                                         const cv::Vec3d &iPlaneNormal,
                                         const float &iPlaneDistance)
  {
    cv::Vec3d reprojectedPoint;
    cv::Vec3d _imgPoint(iImgPnt(0), iImgPnt(1), 1);

    cv::Matx33d invCalMat(iK.inv());

    reprojectedPoint(2) = -iPlaneDistance /
                               (iPlaneNormal.t() * invCalMat * _imgPoint)(0);
    reprojectedPoint(0) = (invCalMat.row(0)* _imgPoint)(0) * reprojectedPoint(2);
    reprojectedPoint(1) = (invCalMat.row(1)* _imgPoint)(0) * reprojectedPoint(2);

    return reprojectedPoint;
  }

  /**
   @ingroup camera_geom
   @brief Find the pixel from the given list that will undergo the most displacement when warped with
   the given homography.
   @param[in] iPixels List of pixels which are to be tested.
   @param[in] iHomography \f$3\times 3\f$ homography matrix with which is to be tested.
   @return Tuple consisting of a two-dimensional vector holding the pixel coordinates and the amount
   of displacement which was found.
   */
  inline std::tuple<cv::Vec2i, float> findMaxDispPixel(const std::vector<cv::Vec2i>& iPixels,
                                                const cv::Matx33d& iHomography)
  {
    //--- init return value ---
    std::tuple<cv::Vec2i, float> retVal(cv::Vec2i(-1,-1), FLT_MIN);

    for (cv::Vec2i px : iPixels)
    {
      cv::Vec2f warpedPx = warpPixel(px, iHomography);

      //--- find max displacement ---
      float displacement = cv::norm(warpedPx - cv::Vec2f(px));
      if(displacement > std::get<1>(retVal))
      {
        std::get<0>(retVal) = px;
        std::get<1>(retVal) = displacement;
      }
    }

    return retVal;
  }

} // namespace lib3d

#endif // LIB3D_GEOMETRY_H
