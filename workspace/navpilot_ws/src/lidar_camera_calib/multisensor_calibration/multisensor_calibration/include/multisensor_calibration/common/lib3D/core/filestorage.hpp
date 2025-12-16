#ifndef LIB3D_FILESTORAGE_H
#define LIB3D_FILESTORAGE_H

// std
#include <string>
#include <cctype>
#include <iostream>
#include <fstream>

//Qt
#include <QDir>
#include <QFileInfo>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// lib3D
#include "version.hpp"
#include "camera.hpp"
#include "image.hpp"
#include "frame.hpp"

namespace lib3d {

/**
 @ingroup file_storage
 @brief Class to store lib3D image and camera data into files.

 Files can be stored in a binary or an ascii format, the latter being useful for debug purposes.

 <h3>Storing of image data</h3>
 <h4>Binary format</h4>

 When the binary format the image data is stored into mixed text and binary files.
 The text header holds the version of the lib3D_core module, the type id of the data as well as the
 dimensions of the image in the format 'CoreVersion&TypeId&Width&Height&Channels&'.
 After the header the data is stored as row-major format. The TypeId field will hold the value
 returned by 'typeid(T).name', i.e. 'd' for double.

 The file ending is constructed with a prefix and the content string of the image, i.e. (.lib3d<contentStr>).
 For example, 'lib3ddepth' for depth maps. The type id of the data is not encoded into the file name,
 thus when storing 'DepthMap' and 'DDepthMap' the file endings will be the same. A check for the use
 of correct type id is done when parsing the file.

 <h4>ASCII format</h4>

 When the ascii format the image data is stored into a XML formatted ascii file which can be opened
 in a text editor. In this the 'cv::FileStorage' is utilized. It will save the verison string of the
 lib3D_core module as well as the data of the image.

 The file ending is constructed with a prefix and the content string of the image followed by '.xml' to
 indicate the intention for debug purposes, i.e. (.lib3d<contentStr>.xml).

 <h3>Storing of camera data</h3>
 <h4>Binary format</h4>

 Similiar to saving iamge data to files, the camera data is stored into mixed text and binary files
 ending with '.lib3dcamera'. At the beginning of the file stands the version string of lib3D_core
 followed by the header for the intrinsic camera data, 'CoreVersion&K&TypeId&NumOfParameters&'. The
 data that follows are the parameters of the calibration matrix \f$K\f$ of type stated in TypeId.
 After the parameter data of the calibration matrix comes the header for the distortion parameters,
 '&D&TypeId&NumOfParameters&' followe by the actual data. Afterwards comes the header for the extrinsic
 matrix of the camera, '&RT&TypeId&NumOfParameters&', together with the actual data.

 <h4>ASCII format</h4>

 Again, when using an ascii format, the 'cv::FileStorage' is utilized. It will save the verison string of the
 lib3D_core module as well as the data of the camera into ascii files ending with '.lib3dcamera.xml'

 <h3>Example</h3>

 In this short example an object of a camera model, a depth map and a normal map is initialized and
 written to binary files in a specified output directory.

 @code{.cpp}
  ...

  // create camera object, depth map and normal map
  lib3d::Camera camera;
  lib3d::DepthMap depthMap;
  lib3d::NormalMap normalMap;

  // ... Fill with data

  // specify file name without extension
  std::string fileName = "foo";

  // sepcify output directory
  std::string outputDir = "/home/user/foo_output/";

  // write to binary files in outputDir
  lib3d::FileStorage lib3dFS(outputDir, lib3d::FileStorage::BIN);
  lib3dFS.writeCameraModel(fileName, camera);
  lib3dFS.writeDepthMap(fileName, depthMap);
  lib3dFS.writeNormalMap(fileName, normalMap);

  ...
 @endcode
 */
class FileStorage
{
  public:

    /**
     @brief Enumeration holding possible formats in which the files can be stored.
     */
    enum EFileFormat {
      BIN,
      ASCII
    };

    //--- METHOD DECLERATION ---//

  public:

    /**
     @brief Default constructor initializing the base directory with './' and a binary file format.
     */
    explicit FileStorage() : FileStorage("./") {}

    /**
     @brief Initialization constructor
     @param[in] iBaseDirPath Path to base directory in which the data is to be stored.
     @param[in] iFileFormat File format to use.
     */
    explicit FileStorage(const std::string& iBaseDirPath, const EFileFormat& iFileFormat = BIN) :
      FileStorage(QDir(QString::fromStdString(iBaseDirPath)), iFileFormat) {}

    /**
     @brief Initialization constructor
     @param[in] iBaseDir Base directory in which the data is to be stored.
     @param[in] iFileFormat File format to use.
     */
    explicit FileStorage(const QDir& iBaseDir, const EFileFormat& iFileFormat = BIN) :
      mBaseDir(iBaseDir), mFileformat(iFileFormat) { }

    virtual ~FileStorage(){}

    /**
     @brief Returns base directory in which the data is to be stored.
     */
    QDir baseDir() const
    {
      return mBaseDir;
    }

    /**
     @brief Set base directory in which the data is to be stored.
     */
    void setBaseDir(const QDir &baseDir)
    {
      mBaseDir = baseDir;
      mBaseDir.makeAbsolute();
    }

    /**
     @overload
     @param[in] iDirPath Path to diretory.
     */
    void setBaseDir(const std::string &iDirPath)
    {
      mBaseDir = QDir(QString::fromStdString(iDirPath));
      mBaseDir.makeAbsolute();
    }

    /**
     @brief Returns file format to use.
     */
    EFileFormat fileformat() const
    {
      return mFileformat;
    }

    /**
     @brief Set file format to use.
     */
    void setFileformat(const EFileFormat &fileformat)
    {
      mFileformat = fileformat;
    }

    /**
     @brief Write camera model to file.

     @param[in] iFilename Name of the file into which the data is to be stored. Can be given with
     or without file ending. If the corresponding file ending (.lib3dcamera) is not provided, it will
     be appended to the file. The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[in] iCamObj Camera object that is to be stored.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    bool writeCameraModel(const std::string& iFilename, const Camera& iCamObj)
    {
      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      QString filePath = mBaseDir.absolutePath() + QDir::separator() + QString::fromStdString(iFilename);

      //--- if filename does not contain file appropriate ending (case insensitive) appen ending
      QString fileEnding = ".lib3dcamera";
      if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        filePath.append(fileEnding);

      if(mFileformat == BIN)
      {
        //--- open stream object
        std::ofstream outputStream;
        outputStream.open(filePath.toStdString(), std::ios::binary);

        if(!outputStream.is_open())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                    << " could not be opened!" << std::endl;
          return false;
        }

        //--- write header
        outputStream << core::getVersionStr() << '&' << std::flush;
        outputStream << iCamObj.intrinsics.getWidth() << '&' << std::flush;
        outputStream << iCamObj.intrinsics.getHeight() << '&' << std::flush;

        //--- write intrinsics
        outputStream << 'K' << '&' << typeid(double).name() << '&' << '9' << '&' << std::flush;
        outputStream.write(reinterpret_cast<const char*>(&iCamObj.intrinsics.getK_as3x3()(0,0)),
                           9 * sizeof(double));
        outputStream << '&' << std::flush;

        //--- write distortion
        cv::Mat distCoeffs = iCamObj.intrinsics.getDistortionCoeffs();
        int numDistCoeffs = (distCoeffs.size().width == 0 || distCoeffs.size().height == 0) ? 0 :
                    std::max(distCoeffs.size().width, distCoeffs.size().height);
        outputStream << 'D' << '&' << typeid(double).name() << '&' << numDistCoeffs << '&' << std::flush;
        if (numDistCoeffs > 0)
          outputStream.write(reinterpret_cast<const char*>(&distCoeffs.at<double>(0,0)),
                             numDistCoeffs * sizeof(double));
        outputStream << '&' << std::flush;

        //--- write Rotation
        outputStream << "RT" << '&' << static_cast<int>(iCamObj.extrinsics.getTransfDirection())
                     << '&' << typeid(double).name() << '&' << "16" << '&' << std::flush;
        outputStream.write(reinterpret_cast<const char*>(&iCamObj.extrinsics.getRTMatrix()(0,0)),
                           16 * sizeof(double));

        outputStream.close();
      }
      else if(mFileformat == ASCII)
      {
        //--- append debug tag to file ending
        filePath.append(".xml");

        cv::FileStorage fs(filePath.toStdString(), cv::FileStorage::WRITE|cv::FileStorage::FORMAT_XML);
        fs << "CoreVersion"     << core::getVersionStr();
        fs << "ImageSize"       << iCamObj.intrinsics.getImageSize();
        fs << "K"               << iCamObj.intrinsics.getK_as3x3();
        fs << "D"               << iCamObj.intrinsics.getDistortionCoeffs();
        fs << "TransfDir"       << iCamObj.extrinsics.getTransfDirection();
        fs << "RT"              << iCamObj.extrinsics.getRTMatrix();
        fs.release();
      }

      return true;
    }

    /**
     @brief Write camera model to file.

     @param[in] iFilename Name of the file into which the data is to be stored. Can be given with
     or without file ending. If the corresponding file ending (.lib3dcamera) is not provided, it will
     be appended to the file. The file name is to be given relative to the base directory (setBaseDir())
     and the subdirectory (iSubDir).
     @param[in] iCamObj Camera object that is to be stored.
     @param[in] iSubdir Subdirectory in which the file is to be stored. If subdirectory does not exists
     it is created before the file is stored.
     @return False, if not successful. True, otherwise.
     */
    bool writeCameraModel(const std::string& iFilename, const Camera& iCamObj,
                          const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeCameraModel((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, iCamObj);
    }

    /**
     @brief Read camera model from file.

     @param[in] iFilename Name of the file in which the data is to be stored. Is to be given with
     file ending (.lib3dcamera). The file name is to be given relative to the base directory (setBaseDir()).
     @param[out] oCamObj Camera object in which the data is to be read.
     @note The method will check if the type of data stored inside the file corresponds to the camera
     object.
     @return False, if not successful (e.g. data types are incompatible). True, otherwise.
     */
    bool readCameraModel(const std::string& iFilename, Camera& oCamObj)
    {
      //--- create file path and check if exists
      QString filePath = mBaseDir.absolutePath() + QDir::separator() + QString::fromStdString(iFilename);
      if(!QFileInfo(filePath).exists())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                  << " does not exist!" << std::endl;
        return false;
      }

      QString fileEnding = ".lib3dcamera";

      if(mFileformat == BIN)
      {
        //--- check if file ending corresponds to image type
        if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Incompatible file endings."
                    << "\n\tFile: " << filePath.toStdString()
                    << "\n\tExpected ending: " << fileEnding.toStdString()
                    << std::endl;
          return false;
        }

        //--- open stream object
        std::ifstream inputStream;
        inputStream.open(filePath.toStdString(), std::ios::binary);

        if(!inputStream.is_open())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                    << " could not be opened!" << std::endl;
          return false;
        }

        //--- copies all data into buffer ---
        std::vector<char> buffer((std::istreambuf_iterator<char>(inputStream)),
                                 (std::istreambuf_iterator<char>()));



        //--- define metadata
        std::string coreVersionStr = "";
        int imgWidth = 0;
        int imgHeight = 0;

        //--- read header
        std::string tmpString = "";
        int i = 0;
        while(i < 3)
        {
          //--- get front char ---
          char currentChar = buffer.front();
          buffer.erase(buffer.begin());

          if(currentChar == '&')
          {
            if(i == 0)
              coreVersionStr = tmpString;
            else if(i == 1)
              imgWidth = std::stoi(tmpString);
            else if(i == 2)
              imgHeight = std::stoi(tmpString);

            ++i;
            tmpString = "";
          }
          else
          {
            tmpString += currentChar;
          }
        }

        //--- store frame size
        oCamObj.intrinsics.setImageSize(cv::Size(imgWidth, imgHeight));

        //--- read calibration matrix
        std::string typeName;
        tmpString = "";
        i = 0;
        int numParams;
        while(i < 3)
        {
          //--- get front char ---
          char currentChar = buffer.front();
          buffer.erase(buffer.begin());

          if(currentChar == '&')
          {
            if(i == 1)
              typeName = tmpString;
            else if(i == 2)
              numParams = std::stoi(tmpString);

            ++i;
            tmpString = "";
          }
          else
          {
            tmpString += currentChar;
          }
        }
        assert(typeName == "d" && numParams == 9);

        //--- copy data
        cv::Matx33d tmpK;
        std::memcpy(reinterpret_cast<char*>(&tmpK(0,0)), buffer.data(), numParams*sizeof(double));
        oCamObj.intrinsics.setBy_K(tmpK);
        buffer.erase(buffer.begin(), buffer.begin() + numParams *sizeof(double) + 1); // +1 due to trailing '&'

        //--- read distortion parameters
        tmpString = "";
        i = 0;
        while(i < 3)
        {
          //--- get front char ---
          char currentChar = buffer.front();
          buffer.erase(buffer.begin());

          if(currentChar == '&')
          {
            if(i == 1)
              typeName = tmpString;
            else if(i == 2)
              numParams = std::stoi(tmpString);

            ++i;
            tmpString = "";
          }
          else
          {
            tmpString += currentChar;
          }
        }
        assert(typeName == "d");

        //--- copy data
        cv::Mat tmpParams = cv::Mat(cv::Size(1,numParams),CV_64FC1, 0.f);
        std::memcpy(reinterpret_cast<char*>(tmpParams.data), buffer.data(), numParams*sizeof(double));
        oCamObj.intrinsics.setDistortionCoeffs(tmpParams);
        buffer.erase(buffer.begin(), buffer.begin() + numParams *sizeof(double) + 1); // +1 due to trailing '&'

        //--- read roation parameters
        tmpString = "";
        int transfDirection;
        i = 0;
        while(i < 4)
        {
          //--- get front char ---
          char currentChar = buffer.front();
          buffer.erase(buffer.begin());

          if(currentChar == '&')
          {
            if(i == 1)
              transfDirection = std::stoi(tmpString);
            else if(i == 2)
              typeName = tmpString;
            else if(i == 3)
              numParams = std::stoi(tmpString);

            ++i;
            tmpString = "";
          }
          else
          {
            tmpString += currentChar;
          }
        }
        assert(typeName == "d" && numParams == 16);

        //--- copy data
        cv::Matx44d tmpRT;
        std::memcpy(reinterpret_cast<char*>(&tmpRT(0,0)), buffer.data(), numParams*sizeof(double));
        oCamObj.extrinsics =
            lib3d::Extrinsics(static_cast<lib3d::Extrinsics::ETransfDirection>(transfDirection));
        oCamObj.extrinsics.setRTMatrix(tmpRT);
        buffer.erase(buffer.begin(), buffer.begin() + numParams *sizeof(double));

        inputStream.close();
      }
      else if(mFileformat == ASCII)
      {
        //--- check if file ending corresponds to image type
        fileEnding.append(".xml");
        if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Incompatible file endings."
                    << "\n\tFile: " << filePath.toStdString()
                    << "\n\tExpected ending: " << fileEnding.toStdString()
                    << std::endl;
          return false;
        }

        //--- read data into temporary mat
        cv::FileStorage fs(filePath.toStdString(), cv::FileStorage::READ|cv::FileStorage::FORMAT_XML);

        std::string tmpStr;

        cv::Size frameSize;
        fs["ImageSize"] >> frameSize;
        oCamObj.intrinsics.setImageSize(frameSize);
        cv::Matx33d tmp33M;
        fs["K"] >> tmp33M;
        oCamObj.intrinsics.setBy_K(tmp33M);
        cv::Mat tmpParams;
        fs["D"] >> tmpParams;
        oCamObj.intrinsics.setDistortionCoeffs(tmpParams);
        int tmpI;
        fs["TransfDir"] >> tmpI;
        oCamObj.extrinsics =
            lib3d::Extrinsics(static_cast<lib3d::Extrinsics::ETransfDirection>(tmpI));
        cv::Matx44d tmp44M;
        fs["RT"] >> tmp44M;
        oCamObj.extrinsics.setRTMatrix(tmp44M);
        fs.release();
      }

      return true;
    }

    /**
     @brief Read camera model from file.
     @param[in] iFilename Name of the file in which the data is to be stored. Is to be given with
     file ending (.lib3dcamera). The file name is to be given relative to the base directory (setBaseDir())
     and the subdirectory (iSubDir).
     @param[out] oCamObj Camera object in which the data is to be read.
     @param[in] iSubdir Subdirectory in which the file is stored. If subdirectory does not exists
     the method will return false.
     @note The method will check if the type of data stored inside the file corresponds to the camera
     object.
     @return False, if not successful (e.g. data types are incompatible). True, otherwise.
     */
    bool readCameraModel(const std::string& iFilename, Camera& oCamObj,
                         const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- call overloaded method
      return readCameraModel((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, oCamObj);
    }

    /**
     @brief Template method to write image data to file.
     @param[in] iFilename Name of the file into which the data is to be stored. Can be given with
     or without file ending. If the corresponding file ending (.lib3d<contentStr>) is not provided, it will
     be appended to the file. The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[in] iImgObj Image object that is to be stored.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    template<typename T, int l>
    bool writeImageData(const std::string& iFilename, const Image<T,l>& iImgObj)
    {
      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      QString filePath = mBaseDir.absolutePath() + QDir::separator() + QString::fromStdString(iFilename);

      //--- if filename does not contain file appropriate ending (case insensitive) appen ending
      QString fileEnding = constructFileEnding(iImgObj.contentStr());
      if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        filePath.append(fileEnding);

      if(mFileformat == BIN)
      {
        //--- collect metadata
        std::string typeName = std::string(typeid(T).name());
        int imgWidth = iImgObj.size().width;
        int imgHeight = iImgObj.size().height;
        int imgChannels = l;

        //--- open stream object
        std::ofstream outputStream;
        outputStream.open(filePath.toStdString(), std::ios::binary);

        if(!outputStream.is_open())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                    << " could not be opened!" << std::endl;
          return false;
        }

        //--- write header
        outputStream << core::getVersionStr() << '&'
                     << typeName << '&'
                     << imgWidth << '&'
                     << imgHeight << '&'
                     << imgChannels << '&'
                     << std::flush;

        //--- write data
        outputStream.write(reinterpret_cast<const char*>(iImgObj.data), imgWidth * imgHeight * imgChannels * sizeof(T));

        outputStream.close();
      }
      else if(mFileformat == ASCII)
      {
        //--- append debug tag to file ending
        filePath.append(".xml");

        cv::FileStorage fs(filePath.toStdString(), cv::FileStorage::WRITE|cv::FileStorage::FORMAT_XML);
        fs << "CoreVersion"        << core::getVersionStr();
        fs << iImgObj.contentStr() << iImgObj;
        fs.release();
      }

      return true;
    }

    /**
     @brief Write color image to file.
     @see writeImageData() for more information
     */
    bool writeColorImage(const std::string& iFilename, const ColorImage& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write grayscale image to file.
     @see writeImageData() for more information
     */
    bool writeGraysaleImage(const std::string& iFilename, const GrayscaleImage& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write depth map to file.
     @see writeImageData() for more information
     */
    bool writeDepthMap(const std::string& iFilename, const DepthMap& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @overload
     */
    bool writeDepthMap(const std::string& iFilename, const DepthMapD& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write disparity map to file.
     @see writeImageData() for more information
     */
    bool writeDisparityMap(const std::string& iFilename, const DisparityMap& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @overload
     */
    bool writeDisparityMap(const std::string& iFilename, const DisparityMapD& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write confidence map to file.
     @see writeImageData() for more information
     */
    bool writeConfidenceMap(const std::string& iFilename, const ConfidenceMap& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @overload
     */
    bool writeConfidenceMap(const std::string& iFilename, const ConfidenceMapD& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write normal map to file.
     @see writeImageData() for more information
     */
    bool writeNormalMap(const std::string& iFilename, const NormalMap& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @overload
     */
    bool writeNormalMap(const std::string& iFilename, const NormalMapD& iImgObj)
    {
      return writeImageData(iFilename, iImgObj);
    }

    /**
     @brief Write image to visual file (e.g. png, jpg, ...) using write routine from OpenCV.
     @param[in] iFilename Name of the file into which the data is to be stored. Needs to be given with
     desired file ending (e.g. png, jpg, ...) in order for OpenCV to know, which encoding to use.
     The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[in] iImgObj Image object that is to be stored.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    bool writeVisImage(const std::string& iFilename, const cv::Mat& iImgObj)
    {
      QString fullFilePath = mBaseDir.absolutePath() + QDir::separator()
          + QString::fromStdString(iFilename);
      cv::imwrite(fullFilePath.toStdString(), iImgObj);

      //--- check if successful, i.e. file exists
      return QFileInfo(fullFilePath).exists();
    }

    /**
     @brief Template method to write iamge model to file.
     @param[in] iFilename Name of the file into which the data is to be stored. Can be given with
     or without file ending. If the corresponding file ending (.lib3d<contentStr>) is not provided, it will
     be appended to the file. The file name is to be given relative to the base directory (setBaseDir())
     and the subdirectory (iSubDir).
     @param[in] iImgObj Image object that is to be stored.
     @param[in] iSubdir Subdirectory in which the file is to be stored. If subdirectory does not exists
     it is created before the file is stored.
     @return False, if not successful. True, otherwise.
     */
    template<typename T, int l>
    bool writeImageData(const std::string& iFilename, const Image<T,l>& iImgObj,
                        const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeImageData<T,l>((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, iImgObj);
    }

    /**
     @brief Write color image to file.
     @see writeImageData() for more information
     */
    bool writeColorImage(const std::string& iFilename, const ColorImage& iImgObj,
                         const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write grayscale image to file.
     @see writeImageData() for more information
     */
    bool writeGraysaleImage(const std::string& iFilename, const GrayscaleImage& iImgObj,
                            const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write disparity map to file.
     @see writeImageData() for more information
     */
    bool writeDisparityMap(const std::string& iFilename, const DisparityMap& iImgObj,
                           const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool writeDisparityMap(const std::string& iFilename, const DisparityMapD& iImgObj,
                           const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write depth map to file.
     @see writeImageData() for more information
     */
    bool writeDepthMap(const std::string& iFilename, const DepthMap& iImgObj,
                       const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool writeDepthMap(const std::string& iFilename, const DepthMapD& iImgObj,
                           const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write confidence map to file.
     @see writeImageData() for more information
     */
    bool writeConfidenceMap(const std::string& iFilename, const ConfidenceMap& iImgObj,
                            const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool writeConfidenceMap(const std::string& iFilename, const ConfidenceMapD& iImgObj,
                            const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write normal map to file.
     @see writeImageData() for more information
     */
    bool writeNormalMap(const std::string& iFilename, const NormalMap& iImgObj,
                        const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool writeNormalMap(const std::string& iFilename, const NormalMapD& iImgObj,
                        const QDir& iSubdir)
    {
      return writeImageData(iFilename, iImgObj, iSubdir);
    }

    /**
     @brief Write image to visual file (e.g. png, jpg, ...) using write routine from OpenCV.
     @param[in] iFilename Name of the file into which the data is to be stored. Needs to be given with
     desired file ending (e.g. png, jpg, ...) in order for OpenCV to know, which encoding to use.
     The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[in] iImgObj Image object that is to be stored.
     @param[in] iSubdir Subdirectory in which the file is to be stored. If subdirectory does not exists
     it is created before the file is stored.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    bool writeVisImage(const std::string& iFilename, const cv::Mat& iImgObj,
                       const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeVisImage((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, iImgObj);
    }

    /**
     @brief Template method to read image data from file.
     @param[in] iFilename Name of the file in which the data is to be stored. Is to be given with
     file ending (.lib3d<contentStr>). The file name is to be given relative to the base directory (setBaseDir()).
     @param[out] oImgObj Image object in which the data is to be read.
     @note The method will check if the type of data stored inside the file corresponds to the image
     object.
     @return False, if not successful (e.g. data types are incompatible). True, otherwise.
     */
    template<typename T, int l>
    bool readImageData(const std::string& iFilename, Image<T,l>& oImgObj)
    {
      //--- create file path and check if exists
      QString filePath = mBaseDir.absolutePath() + QDir::separator() + QString::fromStdString(iFilename);
      if(!QFileInfo(filePath).exists())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ":File " << filePath.toStdString()
                  << " does not exist!" << std::endl;
        return false;
      }

      QString fileEnding = constructFileEnding(oImgObj.contentStr());

      if(mFileformat == BIN)
      {
        //--- check if file ending corresponds to image type
        if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Incompatible file endings."
                    << "\n\tFile: " << filePath.toStdString()
                    << "\n\tExpected ending: " << fileEnding.toStdString()
                    << std::endl;
          return false;
        }

        //--- open stream object
        std::ifstream inputStream;
        inputStream.open(filePath.toStdString(), std::ios::binary);

        if(!inputStream.is_open())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                    << " could not be opened!" << std::endl;
          return false;
        }

        //--- copies all data into buffer ---
        std::vector<char> buffer((std::istreambuf_iterator<char>(inputStream)),
                                 (std::istreambuf_iterator<char>()));



        //--- define metadata
        std::string coreVersionStr = "";
        std::string typeName = "";
        int imgWidth = 0;
        int imgHeight = 0;
        int imgChannels = 1;

        //--- read header
        std::string tmpString = "";
        int i = 0;
        while(i < 5)
        {
          //--- get front char ---
          char currentChar = buffer.front();
          buffer.erase(buffer.begin());

          if(currentChar == '&')
          {
            if(i == 0)
              coreVersionStr = tmpString;
            else if(i == 1)
              typeName = tmpString;
            else if(i == 2)
              imgWidth = std::stoi(tmpString);
            else if(i == 3)
              imgHeight = std::stoi(tmpString);
            else if(i == 4)
              imgChannels = std::stoi(tmpString);

            ++i;
            tmpString = "";
          }
          else
          {
            tmpString += currentChar;
          }
        }

        //--- check that type and number of channels are correct
        if(typeName != typeid(T).name() || imgChannels != l)
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Incompatible image types."
                    << "\n\tImagetype from file: " << typeName << "["  << imgChannels << "]"
                    << "\n\tExpected Imagetype: " << typeid(T).name() << "["  << l << "]"
                    << std::endl;
          return false;
        }

        //--- resize image to size
        oImgObj = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_MAKETYPE(cv::DataType<T>::depth, l));

        //--- copy data
        std::memcpy(oImgObj.data, buffer.data(), imgWidth*imgHeight*imgChannels*sizeof(T));

        inputStream.close();
      }
      else if(mFileformat == ASCII)
      {
        //--- check if file ending corresponds to image type
        fileEnding.append(".xml");
        if(!filePath.endsWith(fileEnding, Qt::CaseInsensitive))
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Incompatible file endings."
                    << "\n\tFile: " << filePath.toStdString()
                    << "\n\tExpected ending: " << fileEnding.toStdString()
                    << std::endl;
          return false;
        }

        //--- read data into temporary mat
        cv::Mat tmpMat;
        cv::FileStorage fs(filePath.toStdString(), cv::FileStorage::READ|cv::FileStorage::FORMAT_XML);
        fs[oImgObj.contentStr()] >> tmpMat;
        fs.release();

        //--- if type of tmpMat does not correspond to output mat, then convert; else just copy
        if(tmpMat.type() != oImgObj.type())
          tmpMat.convertTo(oImgObj, oImgObj.type());
        else
          tmpMat.copyTo(oImgObj);
      }

      return true;
    }

    /**
     @brief Read color image from file.
     @see readImageData() for more information
     */
    bool readColorImage(const std::string& iFilename, ColorImage& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read grayscale image from file.
     @see readImageData() for more information
     */
    bool readGraysaleImage(const std::string& iFilename, GrayscaleImage& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read depth map from file.
     @see readImageData() for more information
     */
    bool readDepthMap(const std::string& iFilename, DepthMap& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @overload
     */
    bool readDepthMap(const std::string& iFilename, DepthMapD& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read disparity map from file.
     @see readImageData() for more information
     */
    bool readDisparityMap(const std::string& iFilename, DisparityMap& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @overload
     */
    bool readDisparityMap(const std::string& iFilename, DisparityMapD& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read confidence map from file.
     @see readImageData() for more information
     */
    bool readConfidenceMap(const std::string& iFilename, ConfidenceMap& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @overload
     */
    bool readConfidenceMap(const std::string& iFilename, ConfidenceMapD& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read normal map from file.
     @see readImageData() for more information
     */
    bool readNormalMap(const std::string& iFilename, NormalMap& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @overload
     */
    bool readNormalMap(const std::string& iFilename, NormalMapD& oImgObj)
    {
      return readImageData(iFilename, oImgObj);
    }

    /**
     @brief Read visual image file (e.g. png, jpg, ...) using write routine from OpenCV.
     @param[in] iFilename Name of the file into which the data is to be stored. Needs to be given with
     desired file ending (e.g. png, jpg, ...) in order for OpenCV to know, which encoding to use.
     The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[out] oImgObj Output image.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    bool readVisImage(const std::string& iFilename, cv::Mat& oImgObj)
    {
      //--- create file path and check if exists
      QString filePath = mBaseDir.absolutePath() + QDir::separator() + QString::fromStdString(iFilename);
      if(!QFileInfo(filePath).exists())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << filePath.toStdString()
                  << " does not exist!" << std::endl;
        return false;
      }

      oImgObj = cv::imread(filePath.toStdString());

      //--- check if successful, i.e. image not empty
      return (!oImgObj.empty());
    }

    /**
     @brief Template method to read image data from file.
     @param[in] iFilename Name of the file in which the data is to be stored. Is to be given with
     file ending (.lib3d<contentStr>). The file name is to be given relative to the base directory (setBaseDir())
     and the subdirectory (iSubDir).
     @param[out] oImgObj Image object in which the data is to be read.
     @param[in] iSubdir Subdirectory in which the file is stored. If subdirectory does not exists
     the method will return false.
     @note The method will check if the type of data stored inside the file corresponds to the camera
     object.
     @return False, if not successful (e.g. data types are incompatible). True, otherwise.
     */
    template<typename T, int l>
    bool readImageData(const std::string& iFilename, Image<T,l>& oImgObj,
                        const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- call overloaded method
      return readImageData<T,l>((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, oImgObj);
    }

    /**
     @brief Read color image from file.
     @see readImageData() for more information
     */
    bool readColorImage(const std::string& iFilename, ColorImage& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read grayscale image from file.
     @see readImageData() for more information
     */
    bool readGraysaleImage(const std::string& iFilename, GrayscaleImage& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read depth map from file.
     @see readImageData() for more information
     */
    bool readDepthMap(const std::string& iFilename, DepthMap& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool readDepthMap(const std::string& iFilename, DepthMapD& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read disparity map from file.
     @see readImageData() for more information
     */
    bool readDisparityMap(const std::string& iFilename, DisparityMap& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool readDisparityMap(const std::string& iFilename, DisparityMapD& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read confidence map from file.
     @see readImageData() for more information
     */
    bool readConfidenceMap(const std::string& iFilename, ConfidenceMap& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool readConfidenceMap(const std::string& iFilename, ConfidenceMapD& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read normal map from file.
     @see readImageData() for more information
     */
    bool readNormalMap(const std::string& iFilename, NormalMap& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @overload
     */
    bool readNormalMap(const std::string& iFilename, NormalMapD& oImgObj,
                        const QDir& iSubdir)
    {
      return readImageData(iFilename, oImgObj, iSubdir);
    }

    /**
     @brief Read visual image file (e.g. png, jpg, ...) using write routine from OpenCV.
     @param[in] iFilename Name of the file into which the data is to be stored. Needs to be given with
     desired file ending (e.g. png, jpg, ...) in order for OpenCV to know, which encoding to use.
     The file name is to be given relative to the base directory (setBaseDir()).
     However, if the file lies in a subdirectory it is to be ensured that the subdirectory exists.
     @param[out] oImgObj Output image.
     @param[in] iSubdir Subdirectory in which the file is stored. If subdirectory does not exists
     the method will return false.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    bool readVisImage(const std::string& iFilename, cv::Mat& oImgObj,
                      const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- call overloaded method
      return readVisImage((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, oImgObj);
    }

    /**
     @brief Write frame onto disk. This will write the content of the frame into separate files,
     according to their type.

     @param[in] iFilename Name of the files into which the data is to be stored. It is assumed to be
     given without ending. Thus, the appropriate file endings will be attached. The file name is to be
     given relative to the base directory (setBaseDir()). However, if the file lies in a subdirectory
     it is to be ensured that the subdirectory exists.
     @param[in] iFrameObj Frame object that is to be stored.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    template<typename T, int l>
    bool writeFrame(const std::string& iFilename, const Frame<T, l>& iFrameObj)
    {
      bool success = true;

      success &= writeCameraModel(iFilename, const_cast<Frame<T, l>&>(iFrameObj).camera());
      if(iFrameObj.containsInputImg())
        success &= writeVisImage(iFilename + "_input.png", const_cast<Frame<T, l>&>(iFrameObj).inputImg());
      if(iFrameObj.containsUndistortedImg())
        success &= writeVisImage(iFilename + "_undistorted.png", const_cast<Frame<T, l>&>(iFrameObj).undistortedImg());
      if(iFrameObj.containsDisparityMap())
        success &= writeImageData(iFilename, const_cast<Frame<T, l>&>(iFrameObj).disparityMap());
      if(iFrameObj.containsDepthMap())
        success &= writeImageData(iFilename, const_cast<Frame<T, l>&>(iFrameObj).depthMap());
      if(iFrameObj.containsConfidenceMap())
        success &= writeImageData(iFilename, const_cast<Frame<T, l>&>(iFrameObj).confidenceMap());
      if(iFrameObj.containsNormalMap())
        success &= writeImageData(iFilename, const_cast<Frame<T, l>&>(iFrameObj).normalMap());

      return success;
    }

    /**
     @overload
     @param[in] iSubdir Subdirectory in which the file is to be stored. If subdirectory does not exists
     it is created before the file is stored.
     */
    template<typename T, int l>
    bool writeFrame(const std::string& iFilename, const Frame<T, l>& iFrameObj, const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeFrame((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, iFrameObj);
    }

    /**
     @overload

     This will use the name of the frame as file name.
     */
    template<typename T, int l>
    bool writeFrame(const Frame<T, l>& iFrameObj)
    {

      //--- call overloaded method
      return writeFrame(const_cast<Frame<T, l>&>(iFrameObj).name(), iFrameObj);
    }

    /**
     @overload

     This will use the name of the frame as file name.
     */
    template<typename T, int l>
    bool writeFrame(const Frame<T, l>& iFrameObj, const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeFrame((iSubdir.path()+ QDir::separator()).toStdString() +
                        const_cast<Frame<T, l>&>(iFrameObj).name(), iFrameObj);
    }

    /**
     @brief Write frame onto disk. This will write the content of the frame into separate files,
     according to their type.

     @param[in] iFilename Name of the files into which the data is to be stored. It is assumed to be
     given without ending. Thus, the appropriate file endings will be attached. The file name is to be
     given relative to the base directory (setBaseDir()). However, if the file lies in a subdirectory
     it is to be ensured that the subdirectory exists.
     @param[out] oFrameObj Output frame object.
     @return False, if not successful (e.g. subdirectory does not exist). True, otherwise.
     */
    template<typename T, int l>
    bool readFrame(const std::string& iFilename, Frame<T, l>& oFrameObj)
    {
      bool success = true;

      QString filenameWithEnding = (mFileformat == BIN) ? QString::fromStdString(iFilename) + ".lib3dcamera" :
                                                          QString::fromStdString(iFilename) + ".lib3dcamera.xml";

      oFrameObj.name() = iFilename;

      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readCameraModel(filenameWithEnding.toStdString(), oFrameObj.camera());

      filenameWithEnding = QString::fromStdString(iFilename) + "_input.png";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readVisImage(filenameWithEnding.toStdString(), oFrameObj.inputImg());

      filenameWithEnding = QString::fromStdString(iFilename) + "_undistorted.png";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readVisImage(filenameWithEnding.toStdString(), oFrameObj.undistortedImg());

      filenameWithEnding = (mFileformat == BIN) ? QString::fromStdString(iFilename) + ".lib3ddisparity" :
                                                  QString::fromStdString(iFilename) + ".lib3ddisparity.xml";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readDisparityMap(filenameWithEnding.toStdString(), oFrameObj.disparityMap());

      filenameWithEnding = (mFileformat == BIN) ? QString::fromStdString(iFilename) + ".lib3ddepth" :
                                                  QString::fromStdString(iFilename) + ".lib3ddepth.xml";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readDepthMap(filenameWithEnding.toStdString(), oFrameObj.depthMap());

      filenameWithEnding = (mFileformat == BIN) ? QString::fromStdString(iFilename) + ".lib3dconfidence" :
                                                  QString::fromStdString(iFilename) + ".lib3dconfidence.xml";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readConfidenceMap(filenameWithEnding.toStdString(), oFrameObj.confidenceMap());

      filenameWithEnding = (mFileformat == BIN) ? QString::fromStdString(iFilename) + ".lib3dnormal" :
                                                  QString::fromStdString(iFilename) + ".lib3dnormal.xml";
      if(QFileInfo(mBaseDir.absolutePath() + QDir::separator() + filenameWithEnding).exists())
        success &= readNormalMap(filenameWithEnding.toStdString(), oFrameObj.normalMap());

      return success;
    }

    /**
     @overload
     @param[in] iSubdir Subdirectory in which the file is to be stored. If subdirectory does not exists
     it is created before the file is stored.
     */
    template<typename T, int l>
    bool readFrame(const std::string& iFilename, Frame<T, l>& oFrameObj, const QDir& iSubdir)
    {
      //--- if subdir is not relative return with false
      if(!iSubdir.isRelative())
        return false;

      //--- if subdir does not exist, create
      mBaseDir.mkdir(".");
      mBaseDir.mkpath(iSubdir.path());

      //--- call overloaded method
      return writeFrame((iSubdir.path()+ QDir::separator()).toStdString() + iFilename, oFrameObj);
    }

  private:

    /// @cond

    /**
     @return File ending corresponding to given content string.
     */
    QString constructFileEnding(const std::string &iContentStr) const
    {
      QString fileEnding = ".lib3d";

      fileEnding.append(QString::fromStdString(iContentStr).toLower());

      return fileEnding;
    }

    /// @endcond

    //--- MEMBER DECLERATION ---//

  private:

    /// Member holding base directory in which the data is to be stored.
    QDir  mBaseDir;

    /// File format to use
    EFileFormat mFileformat;
};


} // namespace lib3d

#endif // LIB3D_FILESTORAGE_H
