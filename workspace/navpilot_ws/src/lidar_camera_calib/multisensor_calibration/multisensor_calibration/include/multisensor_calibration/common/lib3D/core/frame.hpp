#ifndef LIB3D_FRAME_H
#define LIB3D_FRAME_H

// Std
#include <ctime>
#include <map>

// OpenCV
#include <opencv2/core.hpp>

// lib3D images
#include "image.hpp"
#include "camera.hpp"

#define INPUT_IMAGE_KEY "input"
#define UNDISTORTED_IMAGE_KEY "undistorted"
#define DISPARITY_MAP_KEY "disparity"
#define DEPTH_MAP_KEY "depth"
#define CONFIDENCE_MAP_KEY "confidence"
#define NORMAL_MAP_KEY "normal"

namespace lib3d {

/**
 @ingroup camera_geom image_types
 @brief This class serves as a container class for frame data within the lib3D.

 Typically one frame consists of:
 - a Name,
 - a Camera, with intrinsic, extrinsic and distortion parameters,
 - an input image,
 - a corrsponding undistorted image,
 - as well as numerous data maps (e.g. disparity map, depth map, ...)

 This class is templated with the precision of the data maps
 (T_PRECISION) and the number of channels corresponding to the input image (T_IMG_CHANNELS).
 */
template<typename T_PRECISION, int T_IMG_CHANNELS>
class Frame
{
    //--- METHOD DECLERATION ---//

  public:

    /**
     Default constructor.

     This will initialize the name with "Frame_<seconds-since-epoch>".
     */
    explicit Frame() : mName("Frame_")
    {
      //--- append current time since epoch to frame name
      mName.append(std::to_string(std::time(nullptr)));
    }

    /**
     Inititalization constructor.

     Initializing the name with iName.
     */
    explicit Frame(const std::string iName) : mName(iName)
    {}

    /**
     Destructor.
     */
    virtual ~Frame()
    {
      //--- clear maps
      mUcharImgs.clear();
      mSingleChnDataMaps.clear();
      mTripleChnDataMaps.clear();
    }

    /**
     @brief Copy assignment operator.
     This will make a deep copy of the content.
     */
    Frame& operator= (const Frame& rhs)
    {
      mName = rhs.mName;
      mCamera = rhs.mCamera;
      mUcharImgs = rhs.mUcharImgs;
      mSingleChnDataMaps = rhs.mSingleChnDataMaps;
      mTripleChnDataMaps = rhs.mTripleChnDataMaps;
      return *this;
    }

    /**
     Access name of frame by reference.
     */
    std::string& name()
    {
      return mName;
    }

    /**
     Access camera object by reference.
     */
    Camera& camera()
    {
      return  mCamera;
    }

    /**
     Access input image by reference.
     If no input image exists yet, a new object is created.
     */
    template<int ch = T_IMG_CHANNELS>
    typename std::enable_if<(ch == 1), GrayscaleImage&>::type inputImg()
    {
      //--- if input image does not exist in map, create and insert
      if(mUcharImgs.find(INPUT_IMAGE_KEY) == mUcharImgs.end())
        mUcharImgs.insert(std::make_pair(INPUT_IMAGE_KEY, GrayscaleImage()));

      return static_cast<GrayscaleImage&>(mUcharImgs.at(INPUT_IMAGE_KEY));
    }

    /**
     Access input image by reference.
     If no input image exists yet, a new object is created.
     */
    template<int ch = T_IMG_CHANNELS>
    typename std::enable_if<(ch == 3), ColorImage&>::type inputImg()
    {
      //--- if input image does not exist in map, create and insert
      if(mUcharImgs.find(INPUT_IMAGE_KEY) == mUcharImgs.end())
        mUcharImgs.insert(std::make_pair(INPUT_IMAGE_KEY, ColorImage()));

      return static_cast<ColorImage&>(mUcharImgs.at(INPUT_IMAGE_KEY));
    }

    /**
     Access undistoted image by reference.
     If no undistoted image exists yet, a new object is created.
     */
    template<int ch = T_IMG_CHANNELS>
    typename std::enable_if<(ch == 1), GrayscaleImage&>::type undistortedImg()
    {
      //--- if undistorted image does not exist in map, create and insert
      if(mUcharImgs.find(UNDISTORTED_IMAGE_KEY) == mUcharImgs.end())
        mUcharImgs.insert(std::make_pair(UNDISTORTED_IMAGE_KEY, GrayscaleImage()));

      return static_cast<GrayscaleImage&>(mUcharImgs.at(UNDISTORTED_IMAGE_KEY));
    }

    /**
     Access undistoted image by reference.
     If no undistoted image exists yet, a new object is created.
     */
    template<int ch = T_IMG_CHANNELS>
    typename std::enable_if<(ch == 3), ColorImage&>::type undistortedImg()
    {
      //--- if undistorted image does not exist in map, create and insert
      if(mUcharImgs.find(UNDISTORTED_IMAGE_KEY) == mUcharImgs.end())
        mUcharImgs.insert(std::make_pair(UNDISTORTED_IMAGE_KEY, ColorImage()));

      return static_cast<ColorImage&>(mUcharImgs.at(UNDISTORTED_IMAGE_KEY));
    }

    /**
     Access disparity map by reference.
     If no disparity map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, float>::value, DisparityMap&>::type disparityMap()
    {
      //--- if disparity map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(DISPARITY_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(DISPARITY_MAP_KEY, DisparityMap()));

      return static_cast<DisparityMap&>(mSingleChnDataMaps.at(DISPARITY_MAP_KEY));
    }

    /**
     Access disparity map by reference.
     If no disparity map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, double>::value, DisparityMapD&>::type disparityMap()
    {
      //--- if disparity map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(DISPARITY_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(DISPARITY_MAP_KEY, DisparityMapD()));

      return static_cast<DisparityMapD&>(mSingleChnDataMaps.at(DISPARITY_MAP_KEY));
    }

    /**
     Access depth map by reference.
     If no depth map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, float>::value, DepthMap&>::type depthMap()
    {
      //--- if depth map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(DEPTH_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(DEPTH_MAP_KEY, DepthMap()));

      return static_cast<DepthMap&>(mSingleChnDataMaps.at(DEPTH_MAP_KEY));
    }

    /**
     Access depth map by reference.
     If no depth map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, double>::value, DepthMapD&>::type depthMap()
    {
      //--- if depth map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(DEPTH_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(DEPTH_MAP_KEY, DepthMapD()));

      return static_cast<DepthMapD&>(mSingleChnDataMaps.at(DEPTH_MAP_KEY));
    }

    /**
     Access confidence map by reference.
     If no confidence map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, float>::value, ConfidenceMap&>::type confidenceMap()
    {
      //--- if confidence map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(CONFIDENCE_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(CONFIDENCE_MAP_KEY, ConfidenceMap()));

      return static_cast<ConfidenceMap&>(mSingleChnDataMaps.at(CONFIDENCE_MAP_KEY));
    }

    /**
     Access confidence map by reference.
     If no confidence map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, double>::value, ConfidenceMapD&>::type confidenceMap()
    {
      //--- if confidence map does not exist in map, create and insert
      if(mSingleChnDataMaps.find(CONFIDENCE_MAP_KEY) == mSingleChnDataMaps.end())
        mSingleChnDataMaps.insert(std::make_pair(CONFIDENCE_MAP_KEY, ConfidenceMapD()));

      return static_cast<ConfidenceMapD&>(mSingleChnDataMaps.at(CONFIDENCE_MAP_KEY));
    }

    /**
     Access normal map by reference.
     If no normal map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, float>::value, NormalMap&>::type normalMap()
    {
      //--- if normal map does not exist in map, create and insert
      if(mTripleChnDataMaps.find(NORMAL_MAP_KEY) == mTripleChnDataMaps.end())
        mTripleChnDataMaps.insert(std::make_pair(NORMAL_MAP_KEY, NormalMap()));

      return static_cast<NormalMap&>(mTripleChnDataMaps.at(NORMAL_MAP_KEY));
    }

    /**
     Access normal map by reference.
     If no normal map exists yet, a new object is created.
     */
    template<typename pr = T_PRECISION>
    typename std::enable_if<std::is_same<pr, double>::value, NormalMapD&>::type normalMap()
    {
      //--- if normal map does not exist in map, create and insert
      if(mTripleChnDataMaps.find(NORMAL_MAP_KEY) == mTripleChnDataMaps.end())
        mTripleChnDataMaps.insert(std::make_pair(NORMAL_MAP_KEY, NormalMapD()));

      return static_cast<NormalMapD&>(mTripleChnDataMaps.at(NORMAL_MAP_KEY));
    }

    /**
     Check whether frame contains input image.
     */
    bool containsInputImg() const
    {
      return (mUcharImgs.find(INPUT_IMAGE_KEY) != mUcharImgs.end());
    }

    /**
     Check whether frame contains undistorted image.
     */
    bool containsUndistortedImg() const
    {
      return (mUcharImgs.find(UNDISTORTED_IMAGE_KEY) != mUcharImgs.end());
    }

    /**
     Check whether frame contains disparity map.
     */
    bool containsDisparityMap() const
    {
      return (mSingleChnDataMaps.find(DISPARITY_MAP_KEY) != mSingleChnDataMaps.end());
    }

    /**
     Check whether frame contains depth map.
     */
    bool containsDepthMap() const
    {
      return (mSingleChnDataMaps.find(DEPTH_MAP_KEY) != mSingleChnDataMaps.end());
    }

    /**
     Check whether frame contains confidence map.
     */
    bool containsConfidenceMap() const
    {
      return (mSingleChnDataMaps.find(CONFIDENCE_MAP_KEY) != mSingleChnDataMaps.end());
    }

    /**
     Check whether frame contains confidence map.
     */
    bool containsNormalMap() const
    {
      return (mTripleChnDataMaps.find(NORMAL_MAP_KEY) != mTripleChnDataMaps.end());
    }

    /**
     Remove input image from container.
     */
    void removeInputImg()
    {
      typename std::map<std::string,Image<uchar, T_IMG_CHANNELS>>::iterator pos =
          mUcharImgs.find(INPUT_IMAGE_KEY);
      if(pos != mUcharImgs.end())
        mUcharImgs.erase(pos);
    }

    /**
     Remove undistorted image from container.
     */
    void removeUndistortedImg()
    {
      typename std::map<std::string,Image<uchar, T_IMG_CHANNELS>>::iterator pos =
          mUcharImgs.find(UNDISTORTED_IMAGE_KEY);
      if(pos != mUcharImgs.end())
        mUcharImgs.erase(pos);
    }

    /**
     Remove disparity map from container.
     */
    void removeDisparityMap()
    {
      typename std::map<std::string,Image<T_PRECISION, 1>>::iterator pos =
          mSingleChnDataMaps.find(DISPARITY_MAP_KEY);
      if(pos != mSingleChnDataMaps.end())
        mSingleChnDataMaps.erase(pos);
    }

    /**
     Remove depth map from container.
     */
    void removeDepthMap()
    {
      typename std::map<std::string,Image<T_PRECISION, 1>>::iterator pos =
          mSingleChnDataMaps.find(DEPTH_MAP_KEY);
      if(pos != mSingleChnDataMaps.end())
        mSingleChnDataMaps.erase(pos);
    }

    /**
     Remove confidence map from container.
     */
    void removeConfidenceMap()
    {
      typename std::map<std::string,Image<T_PRECISION, 1>>::iterator pos =
          mSingleChnDataMaps.find(CONFIDENCE_MAP_KEY);
      if(pos != mSingleChnDataMaps.end())
        mSingleChnDataMaps.erase(pos);
    }

    /**
     Remove normal map from container.
     */
    void removeNormalMap()
    {
      typename std::map<std::string,Image<T_PRECISION, 3>>::iterator pos =
          mTripleChnDataMaps.find(NORMAL_MAP_KEY);
      if(pos != mTripleChnDataMaps.end())
        mTripleChnDataMaps.erase(pos);
    }

    //--- MEMBER DECLERATION

  protected:

    /// Name of the frame.
    std::string                     mName;

    /// Camera object of the frame.
    Camera                        mCamera;

    /// Map holding uchar images with T_IMG_CHANNELS, i.e. Input image, undistorted image ...
    std::map<std::string,Image<uchar, T_IMG_CHANNELS>> mUcharImgs;

    /// Map holding single channel data maps with T_PRECISION per pixel, i.e. disparityMap, depthMap, confidenceMap ...
    std::map<std::string,Image<T_PRECISION, 1>> mSingleChnDataMaps;

    /// Map holding triple channel data maps with T_PRECISION per pixel, i.e. normalMap ...
    std::map<std::string,Image<T_PRECISION, 3>> mTripleChnDataMaps;
};

/// Frame with color input and single precision
using ColorFrame = Frame<float, 3>;

/// Frame with color input and double precision
using ColorFrameD = Frame<double, 3>;

/// Frame with greayscale input and single precision
using GrayscaleFrame = Frame<float, 1>;

/// Frame with greayscale input and double precision
using GrayscaleFrameD = Frame<double, 1>;

} // namespace lib3d

#endif // LIB3D_FRAME_H
