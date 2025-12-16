#ifndef LIB3D_CAMERAEXTRINSICS_H
#define LIB3D_CAMERAEXTRINSICS_H

// Std
#define _USE_MATH_DEFINES
#include <cmath>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "exceptions.hpp"
#include "geometry.hpp"

namespace lib3d {

/**
 @ingroup camera_geom
 @brief Implementation of the extrinsic camera rotation.
 */
class Rotation
{

    //--- METHOD DECLERATION ---//

  public:

    /**
     @brief Zero initialization constructor.

     This will initialize a zero rotation, i.e. identity rotation matrix.
     */
    explicit Rotation() :
      mRMat(cv::Matx33d::eye()), mRodriguesVec(cv::Vec3d(0.0,0.0,0.0))
    {
    }

    /**
     @brief Initialization constructor, initializing rotation of the camera by providing a \f$3\times 3\f$
      rotation matrix \f$\mathrm{\mathbf{R}}\f$.
     */
    explicit Rotation(const cv::Matx33d& RMat) :
      Rotation()
    {
      this->setRMat(RMat);
    }

    /**
     @brief Initialization constructor, initializing rotation of the camera by providing the rodrigues
     vector.
     */
    explicit Rotation(const cv::Vec3d& rodriguesVec) :
      Rotation()
     {
       this->setRodriguesVec(rodriguesVec);
     }

    /**
     @brief Initialization constructor, initializing rotation of the camera by either providing a
     \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$ or the rodrigues vector.
     @param[in] vecOrMat Rodrigues Vector or Rotation matrix. Thus vecOrMat needs to be of size \f$3\times 3\f$
     or \f$3\times 1\f$ or \f$1\times 3\f$
     */
    explicit Rotation(const cv::Mat& vecOrMat) :
      Rotation()
    {
      if(vecOrMat.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(vecOrMat.channels() > 1)");

      if(vecOrMat.size() == cv::Size(3,3))
        this->setRMat(vecOrMat);
      else if (vecOrMat.size() == cv::Size(1,3) || vecOrMat.size() == cv::Size(3,1))
        this->setRodriguesVec(vecOrMat);
      else
        throw lib3d::InvalidArgumentException(FN_NAME, "vecOrMat.size() != cv::Size(3,3) &&" \
                                                       "vecOrMat.size() != cv::Size(1,3) &&" \
                                                       "vecOrMat.size() != cv::Size(3,1)");
    }

    /**
     @brief Copy constructor
     */
    Rotation(const Rotation& rhs)
    {
      mRMat = rhs.mRMat;
      mRodriguesVec = rhs.mRodriguesVec;
    }

    /**
     @brief Move constructor.
     */
    Rotation(Rotation&& rhs)
    {
      mRMat = rhs.mRMat;
      rhs.mRMat = cv::Matx33d();

      mRodriguesVec = rhs.mRodriguesVec;
      rhs.mRodriguesVec = cv::Vec3d();
    }

    /**
     @brief Copy assignment operator.
     */
    Rotation& operator= (const Rotation& rhs)
    {
      mRMat = rhs.mRMat;
      mRodriguesVec = rhs.mRodriguesVec;

      return *this;
    }

    /**
     @brief Move assignment operator.
     */
    Rotation& operator= (Rotation&& rhs)
    {
      if(this != &rhs)
      {
        mRMat = rhs.mRMat;
        rhs.mRMat = cv::Matx33d();

        mRodriguesVec = rhs.mRodriguesVec;
        rhs.mRodriguesVec = cv::Vec3d();
      }

      return *this;
    }

    /**
     @brief Comparison operator.
     @returns True, if both \f$\mathrm{\mathbb{R}}\f$ and the rodrigues vector of two rotation
     objects are equal.
     */
    bool operator==(const Rotation& rhs)
    {
      return (mRMat == rhs.mRMat &&
              mRodriguesVec == rhs.mRodriguesVec);
    }

    /**
     @brief Comparison operator.
     @returns True, if both \f$\mathrm{\mathbb{R}}\f$ or the rodrigues vector of two rotation
     objects are Not the same.
     */
    bool operator!=(const Rotation& rhs)
    {
      return !(*this == rhs);
    }

    /**
     @brief Multiply and assing operator.
     Allows to add a rotation to existing object.
     */
    Rotation& operator*= (const Rotation& rhs)
    {
      mRMat = mRMat * rhs.getRMat();
      cv::Rodrigues(mRMat, mRodriguesVec);

      return *this;
    }

    /**
     @brief Multiply operator.
     Allows to add tow rotation objects.
     */
    friend Rotation operator* (Rotation lhs, const Rotation& rhs)
    {
      lhs *= rhs;
      return lhs;
    }

    /**
     @brief Returns the \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$.
     */
    cv::Matx33d getRMat() const
    {
      return mRMat;
    }

    /**
     @brief Returns Rodrigues vector.
     */
    cv::Vec3d getRodriguesVec() const
    {
      return mRodriguesVec;
    }

    /**
     @brief Set the extrinsic camera rotation by providing a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$.
     */
    void setRMat(const cv::Matx33d& R)
    {
      if(mRMat != R)
      {
        mRMat = R;
        cv::Rodrigues(mRMat, mRodriguesVec);
      }
    }

    /**
     @overload
     */
    void setRMat(const cv::Mat& R)
    {
      if(R.size() != cv::Size(3,3))
        throw lib3d::InvalidArgumentException(FN_NAME, "(R.size() != cv::Size(3,3))");

      if(R.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(R.channels() > 1)");

      setRMat(cv::Matx33d(R));
    }

    /**
     @brief Set the extrinsic camera rotation by providing the Rodrigues vector.
     */
    void setRodriguesVec(const cv::Vec3d& rodrigues)
    {
      if(mRodriguesVec != rodrigues)
      {
        mRodriguesVec = rodrigues;
        cv::Rodrigues(mRodriguesVec, mRMat);
      }
    }

    /**
     @overload
     */
    void setRodriguesVec(const cv::Mat& rodrigues)
    {
      if (rodrigues.size() != cv::Size(1,3) && rodrigues.size() != cv::Size(3,1))
        throw(lib3d::InvalidArgumentException(FN_NAME, "(rodrigues.size() != cv::Size(1,3) && rodrigues.size() != cv::Size(3,1))"));

      if(rodrigues.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(rodrigues.channels() > 1)");

      setRodriguesVec(cv::Vec3d(rodrigues));
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_x}\f$ which
     rotates by a given angle around the X-Axis.
     @param[in] angleRad Angle to rotate in radians.
     */
    static cv::Matx33d createRotationX_rad(const double angleRad)
    {
      //--- rotation around xAxis (Tilt) ---
      cv::Matx33d xCompensation(1, 0, 0,
                                0, std::cos(angleRad), -std::sin(angleRad),
                                0, std::sin(angleRad), std::cos(angleRad));

      return xCompensation;
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_y}\f$ which
     rotates by a given angle around the Y-Axis.
     @param[in] angleRad Angle to rotate in radians.
     */
    static cv::Matx33d createRotationY_rad(const double angleRad)
    {
      //--- rotation around yAxis (Pan) ---
      cv::Matx33d yCompensation(std::cos(angleRad), 0, std::sin(angleRad),
                            0, 1, 0,
                            -std::sin(angleRad), 0, std::cos(angleRad));

      return yCompensation;
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_z}\f$ which
     rotates by a given angle around the Z-Axis.
     @param[in] angleRad Angle to rotate in radians.
     */
    static cv::Matx33d createRotationZ_rad(const double angleRad)
    {
      //--- rotation around zAxis  ---
      cv::Matx33d zCompensation(std::cos(angleRad), -std::sin(angleRad), 0,
                               std::sin(angleRad), std::cos(angleRad), 0,
                               0, 0, 1);

      return zCompensation;
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_x}\f$ which
     rotates by a given angle around the X-Axis.
     @param[in] angleDeg Angle to rotate in degrees.
     */
    static cv::Matx33d createRotationX_deg(const double angleDeg)
    {
      return createRotationX_rad(degreeToRadian(angleDeg));
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_y}\f$ which
     rotates by a given angle around the Y-Axis.
     @param[in] angleDeg Angle to rotate in degrees.
     */
    static cv::Matx33d createRotationY_deg(const double angleDeg)
    {
      return createRotationY_rad(degreeToRadian(angleDeg));
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}_z}\f$ which
     rotates by a given angle around the Z-Axis.
     @param[in] angleDeg Angle to rotate in degrees.
     */
    static cv::Matx33d createRotationZ_deg(const double angleDeg)
    {
      return createRotationZ_rad(degreeToRadian(angleDeg));
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$ which
     rotates by a given angles around the X-, Y- and Z-Axis.
     @param[in] xRad Angle to rotate around X-Axis in Radians.
     @param[in] yRad Angle to rotate around Y-Axis in Radians.
     @param[in] zRad Angle to rotate around Z-Axis in Radians.
     */
    static cv::Matx33d createRotationXYZ_rad(const double xRad, const double yRad, const double zRad)
    {
      //--- rotation around xAxis (Tilt) ---
      cv::Matx33d xCompensation = createRotationX_rad(xRad);

      //--- rotation around yAxis (Pan) ---
      cv::Matx33d yCompensation = createRotationY_rad(yRad);

      cv::Matx33d zCompensation = createRotationZ_rad(zRad);

      //--- combine rotations and invert ---
      cv::Matx33d rotCompensation = (zCompensation*yCompensation*xCompensation);

      return rotCompensation;
    }

    /**
     @brief Method to create a \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$ which
     rotates by a given angles around the X-, Y- and Z-Axis.
     @param[in] xDeg Angle to rotate around X-Axis in Degrees.
     @param[in] yDeg Angle to rotate around Y-Axis in Degrees.
     @param[in] zDeg Angle to rotate around Z-Axis in Degrees.
     */
    static cv::Matx33d createRotationXYZ_deg(const double xDeg, const double yDeg, const double zDeg)
    {
      return createRotationXYZ_rad(degreeToRadian(xDeg), degreeToRadian(yDeg), degreeToRadian(zDeg));
    }


    //--- MEMBER DECLERATION ---//

  private:

    private:

    /// Rotation matrix \f$\mathrm{\mathbf{R}}\f$.
    cv::Matx33d mRMat;

    /// Rodrigues vector.
    cv::Vec3d mRodriguesVec;
};

/**
 @ingroup camera_geom
 @brief Implementation of the extrinsic camera translation.
 */
class Translation
{
    //--- METHOD DECLERATION ---//

  public:

    /**
     @brief Zero initialization constructor.
     */
    explicit Translation() :
      mTVec(cv::Vec3d(0.0))
    {
    }

    /**
     @brief Initialization constructor, initializing translation vector of camera.
     */
    explicit Translation(const cv::Vec3d& tVec) :
      Translation()
    {
      this->setTVec(tVec);
    }

    /**
     @brief Initialization constructor, initializing translation vector of camera.
     */
    explicit Translation(const double& X, const double& Y, const double& Z) :
      Translation()
    {
      this->setTVec(X, Y, Z);
    }

    /**
     @brief Copy constructor
     */
    Translation(const Translation& rhs)
    {
      mTVec = rhs.mTVec;
    }

    /**
     @brief Move constructor.
     */
    Translation(Translation&& rhs)
    {
      mTVec = rhs.mTVec;
      rhs.mTVec = cv::Vec3d();
    }

    /**
     @brief Copy assignment operator.
     */
    Translation& operator= (const Translation& rhs)
    {
      mTVec = rhs.mTVec;

      return *this;
    }

    /**
     @brief Move assignment operator.
     */
    Translation& operator= (Translation&& rhs)
    {
      if(this != &rhs)
      {
        mTVec = rhs.mTVec;
        rhs.mTVec = cv::Vec3d();
      }

      return *this;
    }

    /**
     @brief Comparison operator.
     @returns True, if both \f$\mathrm{C}\f$ and \f$\mathrm{t}\f$ of two translation objects are equal.
     */
    bool operator==(const Translation& rhs)
    {
      return (mTVec == rhs.mTVec);
    }

    /**
     @brief Comparison operator.
     @returns True, if \f$\mathrm{C}\f$ or \f$\mathrm{t}\f$ of two translation objects are NOT the same.
     */
    bool operator!=(const Translation& rhs)
    {
      return !(*this == rhs);
    }

    /**
     @brief Add and assignt operator.
     */
    Translation& operator+= (const Translation& rhs)
    {
      mTVec += rhs.mTVec;

      return *this;
    }

    /**
     @brief Subtract and assignt operator.
     */
    Translation& operator-= (const Translation& rhs)
    {
      mTVec -= rhs.mTVec;

      return *this;
    }

    /**
     @brief Addition operator.
     */
    friend Translation operator+ (Translation lhs, const Translation& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    /**
     @brief Subtraction operator.
     */
    friend Translation operator- (Translation lhs, const Translation& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    /**
      @brief Returns translation vector of the camera.
     */
    cv::Vec3d getTVec() const
    {
      return mTVec;
    }

    /**
      @brief Set the translation vector of the camera.
     */
    void setTVec(const cv::Vec3d& tVec)
    {
      mTVec = tVec;
    }

    /**
      @overload
     */
    void setTVec(const cv::Mat& tVec)
    {
      if(tVec.size() != cv::Size(1,3) && tVec.size() != cv::Size(3,1))
        throw lib3d::InvalidArgumentException(FN_NAME, "(tVec.size() != cv::Size(1,3) && tVec.size() != cv::Size(3,1))");

      if(tVec.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(tVec.channels() > 1)");

      setTVec(cv::Vec3d(tVec));
    };

    /**
      @overload
     */
    void setTVec(const double& x, const double& y, const double& z)
    {
      setTVec(cv::Vec3d(x,y,z));
    }

  private:

    //--- MEMBER DECLERATION ---//

  private:

    /// Translation vector of camera.
    cv::Vec3d mTVec;
};

/**
 @ingroup camera_geom
 @brief This class implements the extrinsic transformation of a camera, which comprises a rotation and translation
 and thus have 6 degrees if freedom.

 Most literature describe the extrinsic parameters as a combination of a \f$3\times 3\f$ rotation matrix
 and a three-dimenstional translation vector. However, this representation is ambiguous with respect
 to the coordinate system in which the extrisic parameters are described. In the following it is described
 how the different parameters are interpreted in the scope of lib3D:

  <h3>Rotation</h3>

 In the scope of lib3D, the roational part of the extrinsic camera parameters are interpreted as the
 orientation of the new coordinate system with respect to a reference system. This means that the
 column vectors \f$\mathrm{r}_x, \mathrm{r}_y \text{and} \mathrm{r}_z\f$ of the rotation  matrix \f$\mathrm{\mathbf{R}}\f$
 describe the direction of the different coordinate axes belonging to the euclidean coordiante system of the camera:
 \f[
  \mathrm{\mathbf{R}} = \begin{pmatrix}
    \mathrm{r}_x, \mathrm{r}_y , \mathrm{r}_z
  \end{pmatrix}
 \f]
 In the local coordiante system of the camera, the z-axis point in the viewing direction, the x-axis
 to the right and the y-axis downwards. Appart from the rotation matrix, the camera rotation can also
 be accessed and set by a rodrigues vector which, consequently, is also interpreted as the rotation
 of the camera coordinate sytem with respect to the reference system.

 <h3>Translation</h3>

 The three-dimensional translation of the camera can either be accessed by the vector \f$\mathrm{C}\f$
 or by the vector \f$\mathrm{t}\f$. While \f$\mathrm{C}\f$ describes the absolute position of the camera
 center with respect to the reference system, the vector \f$\mathrm{t} = - \mathrm{\mathbf{R}^T} \cdot \mathrm{C}\f$
 describes the origin of the reference system as seen by the camera. Thus, when the rotation of the
 camera is updated, \f$\mathrm{t}\f$ is also recalculated.

 <h3>Extrinsic Transformation</h3>

 In summary there are two transformations that are described by the camera extrinsic parameters,
 namely:
  - \f$\mathrm{\mathbf{T}_{local\,\rightarrow\,ref}} =
      \begin{bmatrix}
        \mathrm{\mathbf{R}} & \mathrm{C} \\
        \mathrm{0} & \mathrm{1}
      \end{bmatrix} \f$ which will transform a point \f$\mathrm{P}_\mathrm{loc}\f$ from the
    local coordinate system of the camera to the reference system and,
  - \f$\mathrm{\mathbf{T}_{ref\,\rightarrow\,local}} = \mathrm{\mathbf{T}_{local\,\rightarrow\,ref}}^{-1} =
      \begin{bmatrix}
          \mathbf{R}^\mathrm{T} & -\mathbf{R}^\mathrm{T} \mathrm{C} \\
          0            & 1
        \end{bmatrix} = \begin{bmatrix}
          \mathbf{R}^\mathrm{T} & \mathrm{t} \\
          0            & 1
        \end{bmatrix} \f$ which will transform a point \f$\mathrm{P}_\mathrm{ref}\f$ given in the
        reference coordiante system to the local camera coordinate system.

  <b>NOTE:</b> The notation above differes from the notation as describen in [OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
  in the notation of \f$\mathrm{\mathbf{R}}\f$. What OpenCV denotes as \f$\mathrm{\mathbf{R}}\f$ corresponds to
  \f$\mathbf{R}^\mathrm{T}\f$ in the notation above.

  lib3d::Extrinisics hold a memeber variable which stores the direction in which the extrinsic transformation
  points and this how the values stored in the rotation ant translation are to be interpreted.
 */
class Extrinsics
{

    //--- ENUM DECLERATION ---//

  public:

    /**
     @brief Enumeration holding direction of transfomration which is represented by the Extrinsic transformation.
     */
    enum ETransfDirection {
      REF_2_LOCAL = 0, ///< Transforming a point \f$\mathrm{P}_\mathrm{ref}\f$ given in a reference coordiante system to the local camera coordinate system.
      LOCAL_2_REF ///< Transforming a point \f$\mathrm{P}_\mathrm{loc}\f$ given in the local camera coordiante system to a reference system.
    };


    //--- METHOD DECLERATION ---//

  public:

    /**
     @brief Default (zero initialization) constructor.
     This will initialize direction of the extrinsic transformation with lib3d::Extrinsics::REF_2_LOCAL.
     */
    explicit Extrinsics() :
      Extrinsics(REF_2_LOCAL)
    {
    }

    /**
     @brief Initialization constructor, intializing the direction with the given variable.
     */
    explicit Extrinsics( const ETransfDirection transfDirection)  :
      translation(Translation()),
      rotation(Rotation()),
      mTransfDirection(transfDirection)
    {
    }

    /**
     @brief Initialization constructor, initializing the extrinsic camera parameters by providing a rotation
     matrix and a translation vector. The interpretation of the given values is provided by the given transformation direction.
     */
    explicit Extrinsics(const cv::Matx33d& RMat, const cv::Vec3d& TVec, const ETransfDirection transfDirection = REF_2_LOCAL) :
      translation(Translation(TVec)),
      rotation(Rotation(RMat)),
      mTransfDirection(transfDirection)
    {
    }

    /**
     @brief Initialization constructor, initializing the extrinsic camera parameters by providing a rodrigues
     vector and a translation vector. The interpretation of the given values is provided by the given transformation direction.
     */
    explicit Extrinsics(const cv::Vec3d& rodriguesVec, const cv::Vec3d& TVec, const ETransfDirection transfDirection = REF_2_LOCAL) :
      translation(Translation(TVec)),
      rotation(Rotation(rodriguesVec)),
      mTransfDirection(transfDirection)
    {
    }

    /**
     @brief Initialization constructor, initializing the extrinsic camera parameters by providing a rotation
     and a translation vector. The rotaton can be given as a matrix or a vector. The interpretation of the given
     values is provided by the given transformation direction.
     */
    explicit Extrinsics(const cv::Mat& RMatOrVec, const cv::Mat& TVec, const ETransfDirection transfDirection = REF_2_LOCAL) :
      Extrinsics(transfDirection)
    {
      if(RMatOrVec.channels() > 1 || TVec.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(RMatOrVec.channels() > 1 || TVec.channels() > 1)");

      if(RMatOrVec.size() == cv::Size(3,3))
        this->rotation.setRMat(cv::Matx33d(RMatOrVec));
      else if (RMatOrVec.size() == cv::Size(1,3) || RMatOrVec.size() == cv::Size(3,1))
        this->rotation.setRodriguesVec(RMatOrVec);
      else
        throw lib3d::InvalidArgumentException(FN_NAME, "(RMatOrVec.size() != cv::Size(3,3) &&" \
                                                       "RMatOrVec.size() != cv::Size(1,3) &&" \
                                                       "RMatOrVec.size() != cv::Size(3,1))");

      if(TVec.size() == cv::Size(1,3) || TVec.size() == cv::Size(3,1))
        this->translation.setTVec(cv::Vec3d(TVec));
      else
        throw lib3d::InvalidArgumentException(FN_NAME, "(TVec.size() != cv::Size(1,3) &&" \
                                                       "TVec.size() != cv::Size(3,1))");
    }

    /**
     @brief Initialization constructor, initializing the extrinsic camera parameters by providing a
     \f$4\times4\f$ transformation matrix. The interpretation of the given values is provided by the
     given transformation direction.
     */
    explicit Extrinsics(const cv::Matx44d& RTMat, const ETransfDirection transfDirection = REF_2_LOCAL) :
      Extrinsics(transfDirection)
    {
      this->setRTMatrix(RTMat);
    }

    /**
     @brief Initialization constructor, initializing the extrinsic camera parameters by providing a
     \f$4\times4\f$ transformation matrix. The interpretation of the given values is provided by the
     given transformation direction.
     */
    explicit Extrinsics(const cv::Mat& RTMat, const ETransfDirection transfDirection = REF_2_LOCAL) :
      Extrinsics(transfDirection)
    {
      if(RTMat.size() == cv::Size(4,4) &&
         RTMat.channels() == 1)
        this->setRTMatrix(cv::Matx44d(RTMat));
      else
        throw(lib3d::InvalidArgumentException(FN_NAME, "(RTMat.size() != cv::Size(4,4) || "\
                                                         "RTMat.channels() != 1)"));
    }

    /**
     @brief Copy constructor
     */
    Extrinsics(const Extrinsics& rhs) :
      translation(rhs.translation),
      rotation(rhs.rotation),
      mTransfDirection(rhs.mTransfDirection)
    {
    }

    /**
     @brief Move constructor.
     */
    Extrinsics(Extrinsics&& rhs) :
      translation(rhs.translation),
      rotation(rhs.rotation),
      mTransfDirection(rhs.mTransfDirection)
    {
      rhs.translation = Translation();
      rhs.rotation = Rotation();
    }

    /**
     @brief Copy assignment operator.
     */
    Extrinsics& operator= (const Extrinsics& rhs)
    {
      rotation = rhs.rotation;
      translation = rhs.translation;
      mTransfDirection = rhs.mTransfDirection;
      return *this;
    }

    /**
     @brief Move assignment operator.
     */
    Extrinsics& operator= (Extrinsics&& rhs)
    {
      if(this != &rhs)
      {
        translation = rhs.translation;
        rhs.translation = Translation();

        rotation = rhs.rotation;
        rhs.rotation = Rotation();

        mTransfDirection = rhs.mTransfDirection;
        rhs.mTransfDirection = REF_2_LOCAL;
      }

      return *this;
    }

    /**
     @brief Comparison operator.
     @returns True, if both rotation and translation of the two objects are equal.
     */
    bool operator==(const Extrinsics& rhs)
    {
      return (rotation == rhs.rotation &&
              translation == rhs.translation &&
              mTransfDirection == rhs.mTransfDirection);
    }

    /**
     @brief Comparison operator.
     @returns True, if both rotation and translation of the two objects are NOT the same.
     */
    bool operator!=(const Extrinsics& rhs)
    {
      return !(*this == rhs);
    }

    /**
     @brief Method to get Extrinsics object that represents the inverse transformation.
     */
    Extrinsics getInverse() const
    {
      Extrinsics retVal = *this;
      retVal.invert();
      return retVal;
    }


    /**
     @brief Get the \f$4\times 4\f$ extrinsic transformation matrix. The direction of transformation
     is determined by the member variable mTransfDirection.
     */
    cv::Matx44d getRTMatrix() const
    {
      cv::Matx44d retVal = cv::Matx44d::eye();

      composeRTMatrix(this->rotation.getRMat(), this->translation.getTVec(), retVal);

      return retVal;
    }

    /**
     @brief Get the \f$4\times 4\f$ extrinsic transformation matrix. The direction of transformation
     is determined either by the member variable mTransfDirection or by the povided input variable transfDirection.
     */
    cv::Matx44d getRTMatrix(const ETransfDirection& transfDirection) const
    {
      if(mTransfDirection == transfDirection)
        return getRTMatrix();
      else
        return this->getInverse().getRTMatrix();
    }

    /**
     @brief Get the \f$3\times 3\f$ rotation matrix. The direction of transformation
     is determined by the member variable mTransfDirection.
     */
    cv::Matx33d getRotationMat() const
    {
      return rotation.getRMat();
    }

    /**
     @brief Get the three dimensional rotational rodrigues vector. The direction of transformation
     is determined by the member variable mTransfDirection.
     */
    cv::Vec3d getRotationVec() const
    {
      return rotation.getRodriguesVec();
    }

    /**
     @brief Get the three dimensional translation vector. The direction of transformation
     is determined by the member variable mTransfDirection.
     */
    cv::Vec3d getTranslationVec() const
    {
      return translation.getTVec();
    }

    /**
     @brief Method to set extrinsics by providing a \f$3\times 3\f$ rotation matrix and a three
     dimensional translation vector. The interpretation of the given variables is determined by the
     member variable mTransfDirection.
     */
    void setExtrinsics(cv::Matx33d rotMat, cv::Vec3d translVec)
    {
      this->rotation.setRMat(rotMat);
      this->translation.setTVec(translVec);
    }

    /**
     @brief Method to set extrinsics by providing a three dimensional rotation vector and a three
     dimensional translation vector. The interpretation of the given variables is determined by the
     member variable mTransfDirection.
     */
    void setExtrinsics(cv::Vec3d rotVec, cv::Vec3d translVec)
    {
      this->rotation.setRodriguesVec(rotVec);
      this->translation.setTVec(translVec);
    }

    /**
     @brief Set the extrinsic camera pose by a \f$4\times 4\f$ transformation matrix. The interpretation
     of the given variables is determined by the member variable mTransfDirection.
     */
    void setRTMatrix(const cv::Matx44d& RTMat)
    {
      cv::Matx33d R;
      cv::Vec3d T;

      decomposeRTMatrix(RTMat, R, T);

      this->rotation.setRMat(R);
      this->translation.setTVec(T);
    }

    /**
     @overload
     */
    void setRTMatrix(const cv::Mat& RTMat)
    {
      if(RTMat.size() != cv::Size(4,4) || RTMat.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(RTMatrix.size() != cv::Size(4,4) || RTMatrix.channels() > 1)");

      this->setRTMatrix(cv::Matx44d(RTMat));
    }

    /**
     @brief Set the extrinsic camera pose by a \f$4\times 4\f$ transformation matrix. The interpretation
     of the given variables is determined either by the member variable mTransfDirection or by the povided
     input variable transfDirection.
     */
    void setRTMatrix(const cv::Matx44d& RTMat, const ETransfDirection& transfDirection)
    {
      if(mTransfDirection == transfDirection)
        setRTMatrix(RTMat);
      else
        setRTMatrix(RTMat.inv());
    }

    /**
     @overload
     */
    void setRTMatrix(const cv::Mat& RTMat, const ETransfDirection& transfDirection)
    {
      if(RTMat.size() != cv::Size(4,4) || RTMat.channels() > 1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(RTMatrix.size() != cv::Size(4,4) || RTMatrix.channels() > 1)");

      this->setRTMatrix(cv::Matx44d(RTMat), transfDirection);

    }

    /**
     @brief Method to set rotational part by providing a \f$3\times 3\f$ rotation matrix. The
     interpretation of the given variables is determined by the member variable mTransfDirection.
     */
    void setRotationMat(const cv::Matx33d& RMat)
    {
      this->rotation.setRMat(RMat);
    }

    /**
     @brief Method to set rotational part by providing a three dimensional rodrigues vector. The
     interpretation of the given variables is determined by the member variable mTransfDirection.
     */
    void setRotationVec(const cv::Vec3d& RodriguesVec)
    {
      this->rotation.setRodriguesVec(RodriguesVec);
    }

    /**
     @brief Method to set rotational part by providing a \f$3\times 3\f$ rotation matrix or a
     three dimensional rodrigues vector. The interpretation of the given variables is determined by
     the member variable mTransfDirection.
     */
    void setRotationMatOrVec(const cv::Mat& rotationMatOrVec)
    {
      if(rotationMatOrVec.size() == cv::Size(3,3) && rotationMatOrVec.channels() == 1)
        this->rotation.setRMat(cv::Matx33d(rotationMatOrVec));
      else if((rotationMatOrVec.size() == cv::Size(3,1) || rotationMatOrVec.size() == cv::Size(1,3) ) &&
              rotationMatOrVec.channels() == 1)
        this->rotation.setRodriguesVec(cv::Vec3d(rotationMatOrVec));
      else
        throw lib3d::InvalidArgumentException(FN_NAME, "( (rotationMatOrVec.size() != cv::Size(3,3) && "\
                                              "rotationMatOrVec.size() != cv::Size(3,1) && " \
                                              "rotationMatOrVec.size() != cv::Size(1,3) ) || " \
                                                       "rotationMatOrVec.channels() != 1");
    }

    /**
     @brief Method to set translational part by providing a three dimensional translation vector. The
     interpretation of the given variables is determined by the member variable mTransfDirection.
     */
    void setTranslationVec(const cv::Vec3d& TVec)
    {
      this->translation.setTVec(TVec);
    }

    /**
     @overload
     */
    void setTranslationVec(const cv::Mat& TVec)
    {
      if((TVec.size() == cv::Size(3,1) || TVec.size() == cv::Size(1,3)) && TVec.channels() == 1)
        this->translation.setTVec(cv::Vec3d(TVec));
      else
        throw(lib3d::InvalidArgumentException(FN_NAME, "((TVec.size() != cv::Size(3,1) && TVec.size() != cv::Size(1,3)) || TVec.channels() != 1)"));
    }

    /**
     @overload
     */
    void setTranslationVec(const double& x, const double& y, const double& z)
    {
      this->translation.setTVec(x,y,z);
    }

    /**
     @brief Returns the direction of transformation of the Extrinsics.
     */
    ETransfDirection getTransfDirection() const
    {
      return mTransfDirection;
    }

    /**
     @brief Method to set the direction of transformation of the Extrinsics. This will invert the
     transformation if data is already set and the passed direction differes from the one stored.
     */
    void setTransfDirection(const ETransfDirection &transfDirection)
    {
      if(transfDirection != mTransfDirection)
      {
        this->invert();
      }
    }

    /**
     @overload
     */
    static void composeRTMatrix(const cv::Matx33d& rotMat , const cv::Vec3d& translVec, cv::Matx44d& RTMat)
    {
      RTMat = cv::Matx44d::eye();

      //--- copy R
      for(int n = 0; n < 3; ++n)
        for(int m = 0; m < 3; ++m)
          RTMat(m,n) = rotMat(m,n);

      //--- copy T
      for(int n = 0; n < 3; ++n)
        RTMat(n,3) = translVec(n);
    }

    /**
     @brief Function to compose \f$4\times 4\f$ transformation matrix from rotation and
     translation.
     @param[in] rotMat \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$
     @param[in] translVec three-dimensional translation vector \f$\mathrm{t}\f$
     @return \f$4\times 4\f$ transformation matrix \f$\mathrm{\mathbf{T}} =
      \begin{bmatrix}
        \mathrm{\mathbf{R}} & \mathrm{t} \\
        \mathrm{0} & \mathrm{1}
      \end{bmatrix} \f$
     */
    static cv::Matx44d composeRTMatrix(const cv::Matx33d& rotMat , const cv::Vec3d& translVec)
    {
      cv::Matx44d RTMat;

      composeRTMatrix(rotMat, translVec, RTMat);

      return RTMat;
    }

    /**
     @overload
     */
    static cv::Matx44d composeRTMatrix(const cv::Vec3d& rotVec , const cv::Vec3d& translVec)
    {
      cv::Matx44d RTMat;

      composeRTMatrix(rotVec, translVec, RTMat);

      return RTMat;
    }

    /**
     @overload
     */
    static void composeRTMatrix(const cv::Vec3d& rotVec , const cv::Vec3d& translVec, cv::Matx44d& RTMat)
    {
      cv::Matx33d rotMat;
      cv::Rodrigues(rotVec, rotMat);

      composeRTMatrix(rotMat, translVec, RTMat);
    }

    /**
     @brief Function to decompose \f$4\times 4\f$ transformation matrix into rotation and
     translation.
     @param[in] RTMat \f$4\times 4\f$ transformation matrix \f$\mathrm{\mathbf{T}} =
      \begin{bmatrix}
        \mathrm{\mathbf{R}} & \mathrm{t} \\
        \mathrm{0} & \mathrm{1}
      \end{bmatrix} \f$
     @param[out] rotMat \f$3\times 3\f$ rotation matrix \f$\mathrm{\mathbf{R}}\f$
     @param[out] translVec three-dimensional translation vector \f$\mathrm{t}\f$
     */
    static void decomposeRTMatrix(const cv::Matx44d& RTMat, cv::Matx33d& rotMat, cv::Vec3d& translVec)
    {
      cv::Matx44d newRTMat = RTMat;
      if(newRTMat(3,3) != 1.0)
        newRTMat(3,3) /= newRTMat(3,3);

      //--- copy R
      for(int n = 0; n < 3; ++n)
        for(int m = 0; m < 3; ++m)
          rotMat(m,n) = newRTMat(m,n);

      //--- copy T
      for(int n = 0; n < 3; ++n)
        translVec(n) = newRTMat(n,3);
    }

    /**
     @overload
     */
    static void decomposeRTMatrix(const cv::Matx44d& RTMat, cv::Vec3d& rotVec, cv::Vec3d& translVec)
    {
      cv::Matx33d rotMat;
      decomposeRTMatrix(RTMat, rotMat, translVec);

      cv::Rodrigues(rotMat, rotVec);
    }

    /**
     @brief Function to compute relative extrinsic transformation between two Extrinsics
     @param[in] eSrc Source Extrinsics given as RT Matrix
     @param[in] eDst Destination Extrinsics given as RT Matrix
     @returns Relative RT matrix going from eSrc to eDst.
     */
    static cv::Matx44d computeRelativeExtrinsics(const cv::Matx44d& eSrc, const cv::Matx44d& eDst)
    {
      cv::Matx44d relativeSourceToDestination;

      //--- get 3x3 and 3x1 vectors ---
      cv::Matx33d rotSource;
      cv::Vec3d tSource;
      decomposeRTMatrix(eSrc, rotSource, tSource);

      cv::Matx33d rotDest;
      cv::Vec3d tDest;
      decomposeRTMatrix(eDst, rotDest, tDest);

      //--- calculate relative transform ---
      cv::Matx33d rotGes =  rotDest*rotSource.t();
      cv::Vec3d tGes = tDest - rotGes * tSource;

      composeRTMatrix(rotGes, tGes, relativeSourceToDestination);
      return relativeSourceToDestination;
    }

    /**
     @overload
     */
    static cv::Mat computeRelativeExtrinsics(const cv::Mat& eSrc, const cv::Mat& eDst)
    {
      if(eSrc.size() != cv::Size(4,4) || eDst.size() != cv::Size(4,4) ||
         eSrc.type() != CV_64FC1 || eDst.type() != CV_64FC1)
        throw lib3d::InvalidArgumentException(FN_NAME, "(eSrc.size() != cv::Size(4,4) || eDst.size() != cv::Size(4,4) || \
                                              eSrc.type() != CV_64FC1 || eDst.type() != CV_64FC1)");

      cv::Matx44d srcRT, dstRT;
      for(int n = 0; n < 4; ++n)
      {
        for(int m = 0; m < 4; ++m)
        {
          srcRT(m,n) = eSrc.at<double>(m,n);
          dstRT(m,n) = eDst.at<double>(m,n);
        }
      }

      return cv::Mat(computeRelativeExtrinsics(srcRT,dstRT));
    }

    /**
     @overload
     @param[in] eSrc Source Extrinsics
     @param[in] eDst Destination Extrinsics
     @returns Relative Extrinsic going from eSrc to eDst.
     @note This will transform the source and destination extrinsics into Extrinsics::REF_2_LOCAL
     before computing the transformation.
     */
    static Extrinsics computeRelativeExtrinsics(const Extrinsics& eSrc, const Extrinsics& eDst)
    {
      Extrinsics eRelativeSourceToDestination;

      //--- calculate relative transform ---
      eRelativeSourceToDestination.setRTMatrix(
            computeRelativeExtrinsics(eSrc.getRTMatrix(Extrinsics::REF_2_LOCAL),
                                      eDst.getRTMatrix(Extrinsics::REF_2_LOCAL)),
            Extrinsics::REF_2_LOCAL);

      //--- convert to rodruigez vector ---
      return eRelativeSourceToDestination;
    }

    /**
     @brief Function to compute HomePose (for which pan=tilt=0) from given Pan-Tilt pose using
     supplied pan and tilt values
     */
    static void computeHomePose(Extrinsics ptPose, double pan, double tilt, Extrinsics& outHomePose)
    {
      cv::Matx33d xCompensation = Rotation::createRotationX_rad(tilt);
      cv::Matx33d yCompensation = Rotation::createRotationY_rad(pan);
      cv::Matx33d zCompensation = Rotation::createRotationZ_rad(0.0);

      //--- combine rotations and invert ---
      cv::Matx33d rotCompensation = (zCompensation*yCompensation*xCompensation).t();

      cv::Mat ptzNormalization = cv::Mat::eye(4,4, CV_64F);
      cv::Mat(rotCompensation).copyTo(ptzNormalization.colRange(0,3).rowRange(0,3));

      // apply ptz normalization
      outHomePose = Extrinsics(ptPose.getRTMatrix() * ptzNormalization);
    }

    /**
     @brief Function to compute shifted Pan-Tilt-Pose with the given values for the given Home-/Zero-Pose
     */
    static void computePanTiltPose(Extrinsics homePose, double pan, double tilt, Extrinsics& outPTPose)
    {
      cv::Matx33d xRot = Rotation::createRotationX_rad(tilt);
      cv::Matx33d yRot = Rotation::createRotationY_rad(pan);
      cv::Matx33d zRot = Rotation::createRotationZ_rad(0.0);

      //--- combine rotations ---
      cv::Matx33d rotCompensation = (zRot * yRot * xRot);

      cv::Mat ptzNormalization = cv::Mat::eye(4,4, CV_64F);
      cv::Mat(rotCompensation).copyTo(ptzNormalization.colRange(0,3).rowRange(0,3));

      // apply ptz normalization
      outPTPose = Extrinsics(homePose.getRTMatrix() * ptzNormalization);
    }

  private:

    /**
     @brief Method to invert the Extrinsics
     */
    void invert()
    {
      this->setRotationMat(this->getRotationMat().t());
      this->setTranslationVec(-this->getRotationMat() * this->getTranslationVec());

      mTransfDirection = (mTransfDirection == LOCAL_2_REF) ?
            REF_2_LOCAL : LOCAL_2_REF;
    }

    //--- MEMBER DECLERATION ---//

  public:

    /// Translational part of the extrinsic camera parameters.
    Translation translation;

    /// Rotational part of the extrinsic camera parameters.
    Rotation rotation;

  private:

    /// Member variable holding the direction of transformation.
    ETransfDirection mTransfDirection;
};

} // namespace lib3d

#endif // LIB3D_CAMERAEXTRINSICS_H
