#ifndef LIB3D_IMAGE_H
#define LIB3D_IMAGE_H

// Std
#include <type_traits>

// OpenCV
#include <opencv2/core.hpp>

namespace lib3d {

/**
 @ingroup image_types
 @brief Base class for image data used in 3D image processing.

 The lib3d::types::Image class provides a more convenient and consistent use of
 <a href="https://docs.opencv.org/4.2.0/d3/d63/classcv_1_1Mat.html">cv::Mat</a>.
 It is templated with a typename T and a number of channels l (layers). It subclasses
 <a href="https://docs.opencv.org/4.2.0/d3/d63/classcv_1_1Mat.html">cv::Mat</a>.

 A number of manifistations, which are key to the lib3D and 3D processing, are predefined with a
 fixed datatype and a fixed number of channels:
 - ColorImage: three-channel RGB image with pixel data of type uchar. Content string: 'RGB'
 - GrayscaleImage: single-channel grayscale image with pixel data of type uchar. Content string: 'Luminance'
 - DepthMap: single-channel image with pixel data of type float. Content string: 'Depth'
 - DDepthMap: single-channel image with pixel data of type double. Content string: 'Depth'
 - DisparityMap: single-channel image with pixel data of type float. Content string: 'Disparity'
 - DDisparityMap: single-channel image with pixel data of type double. Content string: 'Disparity'
 - ConfidenceMap: single-channel image with pixel data of type float. Content string: 'Confidence'
 - DConfidenceMap: single-channel image with pixel data of type double. Content string: 'Confidence'
 - NormalMap: three-channel image with pixel data of type float. Content string: 'Normal'
 - DNormalMap: three-channel image with pixel data of type double. Content string: 'Normal'

 <b>Note:</b> The content string of the generic lib3d::types::Image class will be initialized with '%Image'.
 This can be changed with setContentStr().

 <hr>

 The lib3d::types::Image class can be used in the same way as cv::Mat. However, the member
 function valueAt() provides a 'type-aware' access to the image data. In this, the data can be indexed
 by either passing the x, y and c (channel) coordiantes separately, or by passing a cv::Point2i or
 cv::Point3i. Dependent on the number of channels the data can be accessed with a two-fold
 (i.e. x-y or cv::Point2i) or a three-fold (i.e. x-y-c or cv::Point3i) index.

 <b>Note:</b> When passing the indices separately, the horizontal index (x) comes before the vertical index
 (y). This is different to using cv::Mat::at() but provides a more intuitive interface.

 An instance of a cv::Mat can directly be assigned to lib3d::types::Image, by emplyoing the
 Image::operator=(const Mat& m) function. This is particularly needed for using functions returning
 a cv::Mat, e.g. cv::imread().

 <b>Note:</b> In the Image::operator=(const Mat& m) function it is evaluated whether the type of the passed
 cv::Mat corresponds to the type of the Image instance. If this is not the case, the cv::Mat is not
 assigned to the lib3d::types::Image.

 @tparam T Type name of pixel data (e.g. uchar, float, ...).
 @tparam l Number of channels (layers) inside the image.
 */
template<typename T, int l>
class Image : public cv::Mat
{

    //--- METHOD DECLERATION ---//

  public:

    /**
     @brief Default constructor.
     */
    Image()
      : Image("Image"){}

    /**
     @brief Zero initialization constructor creating an empty image.
     */
    Image(const std::string iContentStr)
      : Image(cv::Size(), iContentStr){}

    /**
     @brief Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    Image(const int& iWidth, const int& iHeight, const std::string iContentStr = "Image")
      : Image(cv::Size(iWidth, iHeight), iContentStr){}

    /**
     @brief Class constructor creating a zero-initialized image of size iSize.
     */
    Image(const cv::Size& iSize, const std::string iContentStr = "Image")
      : cv::Mat(iSize, CV_MAKETYPE(cv::DataType<T>::depth, l), cv::Scalar(0)),
        mContentStr(iContentStr){}

    /**
     @brief Destructor.
     */
    virtual ~Image(){}

    /**
     @brief Assignment operator allowing to assign an instance of <a href="https://docs.opencv.org/4.2.0/d3/d63/classcv_1_1Mat.html">cv::Mat</a>
     to an instance of lib3d::types::Image.

     @note In this, it is evaluated whether the type of the passed cv::Mat corresponds to the type of
     the image instance. If this is not the case, the image object is kept unchanged.
     */
    Image& operator =(const Mat& m)
    {
      //--- make assignement only if type is valid
      if(m.type() == CV_MAKETYPE(cv::DataType<T>::depth, l))
        cv::Mat::operator =(m);

      return *this;
    }

    /**
     @brief Various methods providing easy access to the pixel data. Depending on the datatype and the
     channel number of the image the parameters and the return value differ.
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @return Reference of type T if the Image has 1 channel.
     */
    template<int ch = l>
    typename std::enable_if<(ch == 1), T&>::type valueAt(int x, int y)
    {
      return this->at<T>(y,x);
    }

    /** @overload
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @return Const reference of type T if the Image has 1 channel.
     */
    template<int ch = l>
    typename std::enable_if<(ch == 1), const T&>::type valueAt(int x, int y) const
    {
      return this->at<T>(y,x);
    }

    /** @overload
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @return Reference of a l-dimensional cv::Vec of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), cv::Vec<T,ch>&>::type valueAt(int x, int y)
    {
      return this->at<cv::Vec<T,ch>>(y,x);
    }

    /** @overload
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @return Const reference of a l-dimensional cv::Vec of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), const cv::Vec<T,ch>&>::type valueAt(int x, int y) const
    {
      return this->at<cv::Vec<T,ch>>(y,x);
    }

    /** @overload
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @param[in] c Channel of the pixel to access.
     @return Reference of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), T&>::type valueAt(int x, int y, int c)
    {
      return this->at<cv::Vec<T,ch>>(y,x)(c);
    }

    /** @overload
     @param[in] x X-coordinate of the pixel to access.
     @param[in] y Y-coordinate of the pixel to access.
     @param[in] c Channel of the pixel to access.
     @return Const reference of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), const T&>::type valueAt(int x, int y, int c) const
    {
      return this->at<cv::Vec<T,ch>>(y,x)(c);
    }

    /** @overload
     @param[in] px 2D-Coordinate of the pixel to access.
     @return Reference of type T if the Image has 1 channel.
     */
    template<int ch = l>
    typename std::enable_if<(ch == 1), T&>::type valueAt(cv::Point2i px)
    {
      return this->at<T>(px);
    }

    /** @overload
     @param[in] px 2D-Coordinate of the pixel to access.
     @return Const reference of type T if the Image has 1 channel.
     */
    template<int ch = l>
    typename std::enable_if<(ch == 1), const T&>::type valueAt(cv::Point2i px) const
    {
      return this->at<T>(px);
    }

    /** @overload
     @param[in] px 2D-Coordinate of the pixel to access.
     @return Reference of a l-dimensional cv::Vec of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), cv::Vec<T,ch>&>::type valueAt(cv::Point2i px)
    {
      return this->at<cv::Vec<T,ch>>(px);
    }

    /** @overload
     @param[in] px 2D-Coordinate of the pixel to access.
     @return Const reference of a l-dimensional cv::Vec of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), const cv::Vec<T,ch>&>::type valueAt(cv::Point2i px) const
    {
      return this->at<cv::Vec<T,ch>>(px);
    }

    /** @overload
     @param[in] px 3D-Coordinate of the pixel to access.
     @return Reference of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), T&>::type valueAt(cv::Point3i px)
    {
      return this->at<cv::Vec<T,ch>>(px.y,px.x)(px.z);
    }

    /** @overload
     @param[in] px 3D-Coordinate of the pixel to access.
     @return Const reference of type T.
     */
    template<int ch = l>
    typename std::enable_if<(ch > 1), const T&>::type valueAt(cv::Point3i px) const
    {
      return this->at<cv::Vec<T,ch>>(px.y,px.x)(px.z);
    }

    /**
     @brief Returns a string that classifys the content of the image.
     */
    std::string contentStr() const
    {
      return mContentStr;
    }

    /**
     @brief Set content string which classifying the content of the image.
     */
    void setContentStr(const std::string &iContentStr)
    {
      mContentStr = iContentStr;
    }

     //--- MEMBER DECLERATION ---//

  protected:

    /// String to classify the content of the image.
    std::string mContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for three-channel RGB image with pixel data of type uchar.

 Content string will be initalized with 'RGB'.
 */
class ColorImage : public Image<uchar,3>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    ColorImage()
      : Image<uchar,3>(cv::Size(), "RGB"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    ColorImage(const int& iWidth, const int& iHeight)
      : Image<uchar,3>(cv::Size(iWidth, iHeight), "RGB"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    ColorImage(const cv::Size& iSize)
      : Image<uchar,3>(iSize, "RGB"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel grayscale image with pixel data of type uchar.

 Content string will be initalized with 'Luminance'.
 */
class GrayscaleImage : public Image<uchar,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    GrayscaleImage()
      : Image<uchar,1>(cv::Size(), "Luminance"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    GrayscaleImage(const int& iWidth, const int& iHeight)
      : Image<uchar,1>(cv::Size(iWidth, iHeight), "Luminance"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    GrayscaleImage(const cv::Size& iSize)
      : Image<uchar,1>(iSize, "Luminance"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type float.

 Content string will be initalized with 'Depth'.
 */
class DepthMap : public Image<float,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    DepthMap()
      : Image<float,1>(cv::Size(), "Depth"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    DepthMap(const int& iWidth, const int& iHeight)
      : Image<float,1>(cv::Size(iWidth, iHeight), "Depth"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    DepthMap(const cv::Size& iSize)
      : Image<float,1>(iSize, "Depth"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type double.

 Content string will be initalized with 'Depth'.
 */
class DepthMapD : public Image<double,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    DepthMapD()
      : Image<double,1>(cv::Size(), "Depth"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    DepthMapD(const int& iWidth, const int& iHeight)
      : Image<double,1>(cv::Size(iWidth, iHeight), "Depth"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    DepthMapD(const cv::Size& iSize)
      : Image<double,1>(iSize, "Depth"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type float.

 Content string will be initalized with 'Disparity'.
 */
class DisparityMap : public Image<float,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    DisparityMap()
      : Image<float,1>(cv::Size(), "Disparity"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    DisparityMap(const int& iWidth, const int& iHeight)
      : Image<float,1>(cv::Size(iWidth, iHeight), "Disparity"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    DisparityMap(const cv::Size& iSize)
      : Image<float,1>(iSize, "Disparity"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type double.

 Content string will be initalized with 'Disparity'.
 */
class DisparityMapD : public Image<double,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    DisparityMapD()
      : Image<double,1>(cv::Size(), "Disparity"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    DisparityMapD(const int& iWidth, const int& iHeight)
      : Image<double,1>(cv::Size(iWidth, iHeight), "Disparity"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    DisparityMapD(const cv::Size& iSize)
      : Image<double,1>(iSize, "Disparity"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type float.

 Content string will be initalized with 'Confidence'.
 */
class ConfidenceMap : public Image<float,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    ConfidenceMap()
      : Image<float,1>(cv::Size(), "Confidence"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    ConfidenceMap(const int& iWidth, const int& iHeight)
      : Image<float,1>(cv::Size(iWidth, iHeight), "Confidence"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    ConfidenceMap(const cv::Size& iSize)
      : Image<float,1>(iSize, "Confidence"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for single-channel image with pixel data of type double.

 Content string will be initalized with 'Confidence'.
 */
class ConfidenceMapD : public Image<double,1>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    ConfidenceMapD()
      : Image<double,1>(cv::Size(), "Confidence"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    ConfidenceMapD(const int& iWidth, const int& iHeight)
      : Image<double,1>(cv::Size(iWidth, iHeight), "Confidence"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    ConfidenceMapD(const cv::Size& iSize)
      : Image<double,1>(iSize, "Confidence"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for three-channel image with pixel data of type float.

 Each pixel will have the three components (x,y and z) of the normal vector.

 Content string will be initalized with 'Normal'.
 */
class NormalMap : public Image<float,3>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    NormalMap()
      : Image<float,3>(cv::Size(), "Normal"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    NormalMap(const int& iWidth, const int& iHeight)
      : Image<float,3>(cv::Size(iWidth, iHeight), "Normal"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    NormalMap(const cv::Size& iSize)
      : Image<float,3>(iSize, "Normal"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};

/**
 @ingroup image_types
 @brief Manifistation of lib3d::types::Image for three-channel image with pixel data of type double.

 Each pixel will have the three components (x,y and z) of the normal vector.

 Content string will be initalized with 'Normal'.
 */
class NormalMapD : public Image<double,3>
{
  public:

    /**
     Zero initialization constructor creating an empty image.
     */
    NormalMapD()
      : Image<double,3>(cv::Size(), "Normal"){}

    /**
     Class constructor creating a zero-initialized image of size iWidth \f$\times\f$ iHeight.
     */
    NormalMapD(const int& iWidth, const int& iHeight)
      : Image<double,3>(cv::Size(iWidth, iHeight), "Normal"){}

    /**
     Class constructor creating a zero-initialized image of size iSize.
     */
    NormalMapD(const cv::Size& iSize)
      : Image<double,3>(iSize, "Normal"){}


    // Using assignment operator of base class
    using Image::operator =;

  private:
    /// @private
    using Image::setContentStr;
};


} // namespace lib3d

#endif // LIB3D_IMAGE_H
