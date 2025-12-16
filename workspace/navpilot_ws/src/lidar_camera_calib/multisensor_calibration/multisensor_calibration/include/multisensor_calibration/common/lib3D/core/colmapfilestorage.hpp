#ifndef LIB3D_COLMAPFILESTORAGE_H
#define LIB3D_COLMAPFILESTORAGE_H

// std
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Qt
#include <QDir>
#include <QDataStream>
#include <QQuaternion>

// OpenCV
#include <opencv2/opencv.hpp>

// lib3D
#include "camera.hpp"
#include "image.hpp"

namespace lib3d {

/**
 @ingroup file_storage
 @brief Class to read COLMAP projects and format into lib3D types.

 More information on the camera models of colmap can be found here: https://colmap.github.io/cameras.html

 <h3>Example</h3>

 In this short example an object of a SimplePinholeCamera model and a depth map is read from a colmap
 project.

 @code{.cpp}
  ...

  // create camera object and depth map
  lib3d::Camera camera;
  lib3d::DepthMap depthMap;

  // write to binary files in outputDir
  lib3d::ColmapFileStorage colmapFS("/path/to/sparse/dir", "/path/to/depth_map/dir");

  // get list of files of specified camera model
  std::vector<std::string> fileList = colmapFS.getFileNameList();

  // read camera data
  colmapFS.readCameraModel("image_file.jpg", camera);

  // read depth map data
  colmapFS.readDepthMap("image_file.jpg", depthMap);

  ...
 @endcode
 */
class ColmapFileStorage
{
  public:

    //--- METHOD DECLERATION ---//

    /**
     @brief Default constructor initializing the Cameradata directory with './' and a binary file format.
     */
    explicit ColmapFileStorage():
      ColmapFileStorage("./","./")
    {
    }

    /**
     @brief Initialization constructor
     @param[in] iCameradataDirPath Path to Cameradata directory in which e.g images.txt and cameras.txt are stored
     @param[in] iDepthMapDir Path to depth map directory
     */
    explicit ColmapFileStorage(const std::string& iCameradataDirPath,const std::string& iDepthMapDir)  :
      ColmapFileStorage(QDir(QString::fromStdString(iCameradataDirPath)),
                        QDir(QString::fromStdString(iDepthMapDir)))
    {
    }

    /**
     @brief Initialization constructor
     @param[in] iCameradataDir Path to Cameradata directory in which e.g images.txt and cameras.txt are stored
     @param[in] iDepthMapDir Path to depth map directory
     */
    explicit ColmapFileStorage(const QDir& iCameradataDir,const QDir& iDepthMapDir):
      mCameraDataDir(iCameradataDir),mDepthMapDir(iDepthMapDir)
    {
      setupCameraMaps(mCameraDataDir);
    }

    virtual ~ColmapFileStorage()
    {
    }

    /**
     @brief Returns camera directory in which e.g images.txt and cameras.txt are stored
     */
    QDir cameradataDir() const
    {
      return mCameraDataDir;
    }

    /**
     @brief Set Cameradata directory in which the data is to be stored.
     @param[in] cameradataDir Path to diretory.
     */
    void setCameraDataDir(const QDir &cameradataDir)
    {
      if(mCameraDataDir!=cameradataDir){
          mCameraDataDir = cameradataDir;
          mCameraDataDir.makeAbsolute();
          setupCameraMaps(mCameraDataDir);
      }
    }

    /**
     @overload
     @param[in] iDirPath Path to diretory.
     */
    void setCameraDataDir(const std::string &iDirPath)
    {
      setCameraDataDir(QDir(QString::fromStdString(iDirPath)));
    }

    /**
     @brief Returns camera data directory in which e.g images.txt and cameras.txt are stored
     */
    QDir depthMapDir() const
    {
      return mDepthMapDir;
    }

    /**
     @brief Set depth map directory in which the data is to be stored.
     @param[in] depthMapDir Path to diretory.
     */
    void setDepthMapDir(const QDir &depthMapDir)
    {
      mDepthMapDir = depthMapDir;
      mDepthMapDir.makeAbsolute();
    }

    /**
     @overload
     @param[in] iDirPath Path to diretory.
     */
    void setDepthMapDir(const std::string &iDirPath)
    {
      setDepthMapDir(QDir(QString::fromStdString(iDirPath)));
    }

    /**
     @brief Get list of file names available for the templated camera model
     */
    std::vector<std::string> getFileNameList() const
    {
      std::vector<std::string> fileList;

      for(std::map<std::string, lib3d::Camera>::const_iterator it = mCameras.begin();
          it != mCameras.end(); ++it) {
        fileList.push_back(it->first);
      }

      return fileList;
    }

    /**
     @brief Read camera data from map.
     @param[in] iFileName File name for which the data is to be extracted.
     @param[out] oCamera output camera
     @return Returns false if there exists no such camera model to given file name
     */
    bool readCameraModel(const std::string& iFileName, lib3d::Camera& oCamera)
    {
      auto cameraItr = mCameras.find(iFileName);
      if(cameraItr == mCameras.end())
        return false;

      oCamera = cameraItr->second;
      return true;
    }

    /**
     @brief Read depth map from disk. Uses iFileName and base path.
     @param[in] iFileName base name of depth map file.
     @param[out] oDepthMap output depth map.
     @param[in] iDepthMapType type of depth map (geometric or photometric).
     @returns True if successful, false otherise.
     */
    bool readDepthMap(const std::string& iFileName, cv::Mat& oDepthMap,std::string iDepthMapType ="geometric")
    {
      //--- read buffer ---
      std::ifstream fsDepthFile;
      std::string lineStr;
      std::string depthMapPath = mDepthMapDir.path().toStdString() + "/"+iFileName+"."+iDepthMapType+".bin";
      fsDepthFile.open(depthMapPath, std::ios::binary);

      if(!fsDepthFile.is_open())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << depthMapPath << " could not be opened!" << std::endl;
        return false;
      }

      //--- copies all data into buffer ---
      std::vector<char> buffer((std::istreambuf_iterator<char>(fsDepthFile)),
                               (std::istreambuf_iterator<char>()));


      //--- READ FILE SIZE
      cv::Size depthMapSize;
      int depthMapChannels;
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
            depthMapSize.width = std::stoi(tmpString);
          else if(i == 1)
            depthMapSize.height = std::stoi(tmpString);
          else if(i == 2)
            depthMapChannels = std::stoi(tmpString);

          ++i;
          tmpString = "";
        }
        else
        {
          tmpString += currentChar;
        }
      }

      if(depthMapChannels != 1)
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << depthMapPath << " has more than one channel!" << std::endl;
        return false;
      }

      //--- READ DEPTH MAP
      oDepthMap = cv::Mat::zeros(depthMapSize, CV_32FC1);
      std::memcpy(oDepthMap.data, buffer.data(), buffer.size());

      return true;
    }

    /**
     @overload
     */
    bool readDepthMap(const std::string& iFileName, lib3d::DepthMap& oDepthMap, std::string iDepthMapType ="geometric")
    {
      return readDepthMap(iFileName, static_cast<cv::Mat&>(oDepthMap), iDepthMapType);
    }

    /**
     @brief Write depth map to disk in colmap format. Uses iFileName and base path.
     @param[in] iFileName base name of depth map file.
     @param[in] iDepthMap depth map to write.
     @param[in] iDepthMapType type of depth map (geometric or photometric).
     @returns True if successful, false otherise.
     */
    bool writeDepthMap(const std::string& iFileName, const cv::Mat& iDepthMap, std::string iDepthMapType ="geometric")
    {
      //--- get image dimensions
      int depthMapWidth = iDepthMap.size().width;
      int depthMapHeight = iDepthMap.size().height;
      int depthMapChannels = iDepthMap.channels();

      if(depthMapChannels != 1)
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Passed file has more than one channel!" << std::endl;
        return false;
      }

      //--- if output dir does not exist, create
      if(!mDepthMapDir.exists())
        mDepthMapDir.mkpath(mDepthMapDir.absolutePath());

      //--- open buffer buffer ---
      std::ofstream fsDepthFile;
      std::string depthMapPath = mDepthMapDir.path().toStdString() + "/"+iFileName+"."+iDepthMapType+".bin";
      fsDepthFile.open(depthMapPath, std::ios::binary);

      if(!fsDepthFile.is_open())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << depthMapPath << " could not be opened!" << std::endl;
        return false;
      }

      //--- write header
      fsDepthFile << depthMapWidth << '&'
                  << depthMapHeight << '&'
                  << depthMapChannels << '&'
                  << std::flush;

      //--- write data
      fsDepthFile.write(reinterpret_cast<const char*>(iDepthMap.data),
                        depthMapWidth * depthMapHeight * depthMapChannels * sizeof(float));

      fsDepthFile.close();

      return true;
    }

    /**
     @overload
     */
    bool writeDepthMap(const std::string& iFileName, const lib3d::DepthMap& iDepthMap, std::string iDepthMapType ="geometric")
    {
      return writeDepthMap(iFileName, static_cast<const cv::Mat&>(iDepthMap), iDepthMapType);
    }

  private:

    /**
     @brief Method to read the colmap camera files and fill the maps holding the camera models.
     */
    bool setupCameraMaps(const QDir &cameradataDir)
    {

      //--- check if given folder exists
      if(!cameradataDir.exists())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Folder " << cameradataDir.path().toStdString() << " does not exist!" << std::endl;
        return false;
      }

      //--- clear old data
      mCameras.clear();

      //--- if bin files exists, parse bin files. use txt files otherwise
      std::string binCamerasFilePath = cameradataDir.path().toStdString()+"/cameras.bin";
      std::string binImagesFilePath = cameradataDir.path().toStdString()+"/images.bin";
      std::string txtCamerasFilePath = cameradataDir.path().toStdString()+"/cameras.txt";
      std::string txtImagesFilePath = cameradataDir.path().toStdString()+"/images.txt";
      if(QFile(QString::fromStdString(binCamerasFilePath)).exists() && QFile(QString::fromStdString(binImagesFilePath)).exists())
      {
        bool success = parseCameraModelsBin(cameradataDir,mCameras);

        if(!success)
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Error in trying to read binary files!" << std::endl;
          return false;
        }
      }
      else if(QFile(QString::fromStdString(txtCamerasFilePath)).exists() && QFile(QString::fromStdString(txtImagesFilePath)).exists())
      {
        bool success = parseCameraModelsTxt(cameradataDir,mCameras);

        if(!success)
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Error in trying to read txt files!" << std::endl;
          return false;
        }
      }

      return true;
    }


    /**
     @brief Method to parse the camera models from txt file of colmap.
     */
    bool parseCameraModelsTxt(const QDir& iSparseFolderPath,
                              std::map<std::string,Camera> &oCameras) const
    {

      std::ifstream fsCameraFile, fsImgFile;
      std::string lineTrash;
      std::map<int, Intrinsics> cameraTypes;

      //--- PARSE CAMERAS.TXT
      std::string camerasFilePath = iSparseFolderPath.path().toStdString()+"/cameras.txt";
      fsCameraFile.open(camerasFilePath);
      if(!fsCameraFile.is_open())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << camerasFilePath << " could not be opened!" << std::endl;
        return false;
      }

      //--- read header and move into trash ---
      for(uint i = 0; i < 3; ++i)
      {
        std::getline(fsCameraFile,lineTrash);
        if(fsCameraFile.eof())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Reached unexpected eof!" << std::endl;
          return false;
        }
      }

      while(!fsCameraFile.eof())
      {
        //--- get camera id
        int camId;
        fsCameraFile >> camId;

        //--- get model string
        std::string cameraModel;
        fsCameraFile >> cameraModel;

        Intrinsics camIntr;

        //--- get size
        int width, height;
        fsCameraFile >> width;
        fsCameraFile >> height;
        camIntr.setImageSize(width,height);

        cv::Matx33d K = cv::Matx33d::eye();

        //--- get focallenght
        double fx;
        fsCameraFile >> fx;
        K(0,0)= fx;
        if(cameraModel=="SIMPLE_PINHOLE"||cameraModel=="SIMPLE_RADIAL"||cameraModel=="RADIAL"){
            K(1,1) = fx;
        }else {
            double fy;
            fsCameraFile >> fy;
            K(1,1) = fy;
        }

        //--- get principalpoint
        double cx,cy;
        fsCameraFile >> cx;
        fsCameraFile >> cy;
        K(0,2) = cx;
        K(1,2) = cy;

        //--- get distortion
        std::vector<double> distortionParams;
        if(cameraModel=="SIMPLE_RADIAL"||cameraModel=="RADIAL"||cameraModel=="OPENCV"||cameraModel=="FULL_OPENCV"){
            double distortionParameter1;
            fsCameraFile >> distortionParameter1;
            distortionParams.push_back(distortionParameter1);
        }
        if(cameraModel=="RADIAL"||cameraModel=="OPENCV"||cameraModel=="FULL_OPENCV"){
            double distortionParameter2;
            fsCameraFile >> distortionParameter2;
            distortionParams.push_back(distortionParameter2);
        }
        if(cameraModel=="OPENCV"||cameraModel=="FULL_OPENCV"){
            double distortionParameter3;
            fsCameraFile >> distortionParameter3;
            distortionParams.push_back(distortionParameter3);
            double distortionParameter4;
            fsCameraFile >> distortionParameter4;
            distortionParams.push_back(distortionParameter4);

        }
        if(cameraModel=="FULL_OPENCV"){
            double distortionParameter5;
            double distortionParameter6;
            double distortionParameter7;
            double distortionParameter8;
            fsCameraFile >> distortionParameter5;
            fsCameraFile >> distortionParameter6;
            fsCameraFile >> distortionParameter7;
            fsCameraFile >> distortionParameter8;
            distortionParams.push_back(distortionParameter5);
            distortionParams.push_back(distortionParameter6);
            distortionParams.push_back(distortionParameter7);
            distortionParams.push_back(distortionParameter8);
        }

        cv::Mat distCoeffs = cv::Mat(cv::Size(1, distortionParams.size()), CV_64FC1);
        for(uint i = 0; i < distortionParams.size(); ++i)
          distCoeffs.at<double>(i,0) = distortionParams[i];

        if(camIntr.getWidth() > 0 && camIntr.getHeight() > 0) {
          camIntr.setBy_K(K);
          camIntr.setDistortionCoeffs(distCoeffs);
          cameraTypes[camId] = camIntr;
        }

        std::getline(fsCameraFile,lineTrash);
        std::getline(fsCameraFile,lineTrash);
      }

      fsCameraFile.close();

      if(cameraTypes.empty()){
          return true;
      }

      //--- PARSE IMAGES.TXT
      std::string imagesFilePath = iSparseFolderPath.path().toStdString() + "/images.txt";
      fsImgFile.open(imagesFilePath);
      if(!fsImgFile.is_open())
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << imagesFilePath << " could not be opened!" << std::endl;
        return false;
      }

      //--- read header and move into trash ---
      for(uint i = 0; i < 4; ++i)
      {
        std::getline(fsImgFile,lineTrash);
        if(fsImgFile.eof())
        {
          std::cerr << "[IO Error]\t" << __FUNCTION__ << ": Reached unexpected eof!" << std::endl;
          return false;
        }
      }


      std::string imgFile;
      while(!fsImgFile.eof())
      {

        //--- get image id
        int imgId;
        fsImgFile >> imgId;

        Extrinsics camExt;

        //--- read quaternion and comput R
        double qW, qX, qY, qZ;
        fsImgFile >> qW;
        fsImgFile >> qX;
        fsImgFile >> qY;
        fsImgFile >> qZ;
        QQuaternion quat(static_cast<float>(qW), static_cast<float>(qX),
                         static_cast<float>(qY), static_cast<float>(qZ));
        QMatrix3x3 qRotMat = quat.normalized().toRotationMatrix();


        cv::Matx33d R;
        for(int i = 0 ; i < 3; ++i)
          for(int j = 0; j < 3; ++j)
            R(i,j) = qRotMat(i,j);

        camExt.rotation.setRMat(R);

        //--- read translation
        float x,y,z;
        fsImgFile >> x;
        fsImgFile >> y;
        fsImgFile >> z;

        camExt.translation.setTVec(x,y,z);

        //--- get camera id
        int camId;
        fsImgFile >> camId;

        //--- get Image name
        fsImgFile >> imgFile;

        //--- read one trash lines ---
        std::getline(fsImgFile,lineTrash);
        std::getline(fsImgFile,lineTrash);

        QDir colmapOutputFolder = QDir(iSparseFolderPath);
        colmapOutputFolder.cdUp();

        //--- store camera parameters if camera id is within camera types
        if(cameraTypes.find(camId) != cameraTypes.end())
        {
          Camera camParam;
          camParam.intrinsics = cameraTypes[camId];
          camParam.extrinsics = camExt;
          oCameras[imgFile] = camParam;
        }
      }
      fsImgFile.close();

      return true;
    }

    /**
     @brief Method to parse the camera models from bin file of colmap.
     */
    bool parseCameraModelsBin(const QDir& iSparseFolderPath,
                              std::map<std::string,Camera> &oCameras) const
    {
      std::ifstream fsCameraFile, fsImgFile;
      std::string lineTrash;
      std::map<int, Intrinsics> cameraTypes;

      //--- PARSE CAMERAS.BIN
      std::string camerasFilePath = iSparseFolderPath.path().toStdString()+"/cameras.bin";
      QFile camsFile(QString::fromStdString(camerasFilePath));
      if (!camsFile.open(QFile::ReadOnly))
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << camerasFilePath << " could not be opened!" << std::endl;
        return false;
      }

      QDataStream camFileStream(&camsFile);
      camFileStream.setByteOrder(QDataStream::LittleEndian);

      quint64 numCams;
      camFileStream >> numCams;

      int camId, type;
      quint64 width, height;
      for (size_t i = 0; i < numCams; i++)
      {
        camFileStream >> camId >> type >> width >> height;

        Intrinsics camIntr;

        //--- get size
        camIntr.setImageSize(width, height);

        cv::Matx33d K = cv::Matx33d::eye();

        //--- get focallength
        double fx, fy;
        if(type == 1 || type == 4 || type == 6) // pinhole, opencv, full_opencv
        {
          camFileStream >> fx >> fy;

          K(0,0) = fx;
          K(1,1) = fy;
        }
        else
        {
          camFileStream >> fx;

          K(0,0) = fx;
          K(1,1) = fx;
        }

        //--- get principalpoint
        double cx,cy;
        camFileStream >> cx >> cy;
        K(0,2) = cx;
        K(1,2) = cy;

        //--- get distortion
        std::vector<double> distortionParams;
        if(type == 2) // simple_radial
        {
          double d;
          camFileStream >> d;
          distortionParams.push_back(d);
        }
        else if(type == 3) // radial
        {
          double d1, d2;
          camFileStream >> d1 >> d2;
          distortionParams.push_back(d1);
          distortionParams.push_back(d2);
        }
        else if(type == 4) // opencv
        {
          double d1, d2, d3, d4;
          camFileStream >> d1 >> d2 >> d3 >> d4;
          distortionParams.push_back(d1);
          distortionParams.push_back(d2);
          distortionParams.push_back(d3);
          distortionParams.push_back(d4);
        }
        else if(type == 6) // full_opencv
        {
          double d1, d2, d3, d4, d5, d6, d7, d8;
          camFileStream >> d1 >> d2 >> d3 >> d4 >>d5 >> d6 >> d7 >> d8;
          distortionParams.push_back(d1);
          distortionParams.push_back(d2);
          distortionParams.push_back(d3);
          distortionParams.push_back(d4);
          distortionParams.push_back(d5);
          distortionParams.push_back(d6);
          distortionParams.push_back(d7);
          distortionParams.push_back(d8);
        }

        cv::Mat distCoeffs = cv::Mat(cv::Size(1, distortionParams.size()), CV_64FC1);
        for(uint i = 0; i < distortionParams.size(); ++i)
          distCoeffs.at<double>(i,0) = distortionParams[i];

        if(camIntr.getWidth() > 0 && camIntr.getHeight() > 0) {
          camIntr.setBy_K(K);
          camIntr.setDistortionCoeffs(distCoeffs);
          cameraTypes[camId] = camIntr;
        }
      }

      camsFile.close();

      if(cameraTypes.empty()){
          return true;
      }

      //--- PARSE IMAGES.TXT
      std::string imagesFilePath = iSparseFolderPath.path().toStdString() + "/images.bin";
      QFile imgsFile(QString::fromStdString(imagesFilePath));
      if (!imgsFile.open(QFile::ReadOnly))
      {
        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File " << imagesFilePath << " could not be opened!" << std::endl;
        return false;
      }

      QDataStream imgFileStream(&imgsFile);
      imgFileStream.setByteOrder(QDataStream::LittleEndian);

      quint64 numImages;
      imgFileStream >> numImages;

      for (size_t i = 0; i < numImages; i++)
      {
        quint32 id;
        quint32 cameraID;
        double qw, qx, qy, qz, tx, ty, tz;
        QString name = "";

        imgFileStream >> id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cameraID;

        char c;
        do {
            imgFileStream.readRawData(&c, 1);
            if (c != '\0')
                name.append(c);
        } while (c != '\0');


        Extrinsics camExt;

        //--- read quaternion and comput R
        QQuaternion quat(static_cast<float>(qw), static_cast<float>(qx),
                         static_cast<float>(qy), static_cast<float>(qz));
        QMatrix3x3 qRotMat = quat.normalized().toRotationMatrix();


        cv::Matx33d R;
        for(int i = 0 ; i < 3; ++i)
          for(int j = 0; j < 3; ++j)
            R(i,j) = qRotMat(i,j);

        camExt.rotation.setRMat(R);

        //--- read translation
        camExt.translation.setTVec(tx,ty,tz);

        //--- store camera parameters if camera id is within camera types
        if(cameraTypes.find(camId) != cameraTypes.end())
        {
          Camera camParam;
          camParam.intrinsics = cameraTypes[camId];
          camParam.extrinsics = camExt;
          oCameras[name.toStdString()] = camParam;
        }

        //--- read observations
        quint64 numPoints;
        imgFileStream >> numPoints;
        for (size_t j = 0; j < numPoints; j++)
        {
            double x, y;
            quint64 id3D;
            imgFileStream >> x >> y >> id3D;
        }
      }
      imgsFile.close();

      return true;
    }


    //--- MEMBER DECLERATION ---//

    private:

      /// Member holding the directory in which the Cameradata is stored e.g. images.txt and cameras.txt
      QDir  mCameraDataDir;

      /// Member holding the directory in which the depth maps are stored
      QDir  mDepthMapDir;

      /// Map to save camera intrinsics and extrinsics per image name
      std::map<std::string, lib3d::Camera> mCameras;

};

} // namespace lib3d

#endif // LIB3D_COLMAPFILESTORAGE_H
