#ifndef LIB3D_METASHAPEFILESTORAGE_H
#define LIB3D_METASHAPEFILESTORAGE_H

// std
#include <string>
#include <vector>

// Qt
#include <QFile>
#include <QtXml/QDomNode>
#include <QDataStream>
#include <QFileInfo>
#include <QDebug>
#include <QtXml/QDomDocument>

// OpenCV
#include <opencv2/opencv.hpp>

// lib3D
#include "camera.hpp"
#include "image.hpp"

namespace lib3d {

/**
 @ingroup file_storage
 @brief Class to read Agisoft Metashape projects and formats into lib3D types.

 <h3>Example</h3>

 In this short example an object of a OpenCV camera model is read from a Metashape export.

 @code{.cpp}
  ...

  // create camera object
  lib3d::Camera cam;

  // initialize metashape storage
  lib3d::io::MetashapeFileStorage metashapeFS("/path/to/cameras.xml");

  // get list of files of specified camera model
  std::vector<std::string> fileList = metashapeFS.getFileNameList();

  // read camera data
  metashapeFS.readCameraModel("image_file.jpg", cam);

  ...
 @endcode
 */
class MetashapeFileStorage
{
  public:

    //--- METHOD DECLERATION ---//

    /**
     @brief Default constructor performing zero initialization
     */
    explicit MetashapeFileStorage() :
      MetashapeFileStorage("")
    {
    }

    /**
     @brief Initialization constructor
     @param[in] iCameraXmlFilePath Path to `cameras.xml` file from Metashape
     */
    explicit MetashapeFileStorage(const std::string& iCameraXmlFilePath)  :
      mCameraXmlFile(QString::fromStdString(iCameraXmlFilePath))
    {
      parseCameraXmlFile(mCameraXmlFile, mCameras);
    }


    virtual ~MetashapeFileStorage()
    {
    }


    /**
     @brief Returns path to `cameras.xml` file from Metashape
     */
    std::string cameraXmlFilePath() const
    {
      return QFileInfo(mCameraXmlFile).absolutePath().toStdString();
    }

    /**
     @brief Set `cameras.xml` file.
     @param[in] iCameraXmlFilePath Path to `cameras.xml` file from Metashape
     */
    void setCameraXmlFile(const std::string &iCameraXmlFilePath)
    {
      if(mCameraXmlFile.isOpen())
        mCameraXmlFile.close();

      mCameraXmlFile.setFileName(QString::fromStdString(iCameraXmlFilePath));

      parseCameraXmlFile(mCameraXmlFile, mCameras);
    }

    /**
     @brief Get list of file names for which camera data is available.
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
     Note that in the `camera.xml` file from metashape only the basename (without extension) is stored.
     This function will strip an extension if available.
     @param[out] oCamera output camera     
     @return Returns false if there exists no such camera model to given file name
     */
    bool readCameraModel(const std::string& iFileName, lib3d::Camera& oCamera)
    {
      //--- strip extension if given
      std::string stripedFileName = iFileName.substr(0, iFileName.find_last_of('.'));

      //--- check if camera exists with or without extension
      auto cameraItr = mCameras.find(iFileName);
      if(cameraItr == mCameras.end())
      {
        cameraItr = mCameras.find(stripedFileName);
        if(cameraItr == mCameras.end())
          return false;
      }

      oCamera = cameraItr->second;
      return true;
    }

  private:

    /**
     @brief Method to parse camera extrinsics from given root node.
     @param[in] iRootNode Root node containing the camera extrinsics
     @param[in] iChunkTransf \f$4\,\times\,4\f$ transformation matrix of chunk
     @param[out] oCamExtr list of camera extrinsics together with camera id and label
     */
    void parseCameraExtr(const QDomNode& iRootNode, const cv::Matx44d &iChunkTransf,
                         std::vector<std::tuple<Extrinsics, int, std::string> > &oCamExtr)
    {
      QDomNodeList camerasNodeList = iRootNode.childNodes();
      for(int i = 0; i <  camerasNodeList.size(); ++i)
      {
        QDomElement cameraElement = camerasNodeList.at(i).toElement();

        //--- get sensor id
        int camId = cameraElement.attribute("sensor_id").toInt();

        //--- read file name
        std::string label = cameraElement.attribute("label").toStdString();

        //--- read transformation
        Extrinsics camExtr(Extrinsics::LOCAL_2_REF);
        QDomElement transformationElement = cameraElement.namedItem("transform").toElement();
        QStringList matrixElements = transformationElement.text().split(" ");
        cv::Matx44d tranformMatrix;
        for(int n = 0; n < 4; ++n)
          for(int m = 0; m < 4; ++m)
            tranformMatrix(n, m) = matrixElements.at(n*4 + m).toDouble();
        camExtr.setRTMatrix(iChunkTransf * tranformMatrix);

        oCamExtr.push_back(std::make_tuple(camExtr, camId, label));
      }
    }

    /**
     @brief Method to read the camera models from xml file of Metashape.
     @return True, if successful. False, otherwise.
     */
    bool parseCameraXmlFile(const QFile& iCameraXmlFile,
                            std::map<std::string, lib3d::Camera> &oCameras)

    {
      //--- check if file exists
      if(!iCameraXmlFile.exists())
        return false;

      //--- open xml file
      if (!mCameraXmlFile.open(QIODevice::ReadOnly)) {

        std::cerr << "[IO Error]\t" << __FUNCTION__ << ": File "
                  << QFileInfo(mCameraXmlFile).absolutePath().toStdString()
                  << " could not be opened!" << std::endl;
        return false;
      }

      //--- clear previous data
      oCameras.clear();

      //--- read file content into xmlDOM
      QDomDocument xmlDOM;
      xmlDOM.setContent(&mCameraXmlFile);
      mCameraXmlFile.close();

      //--- Extract the root markup
      QDomElement rootElement = xmlDOM.documentElement();

      //--- Get the first child of the root (Markup CHUNK is expected)
      QDomElement chunkElement = rootElement.firstChild().toElement();

      //--- Loop while there is a child
      while(!chunkElement.isNull())
      {
          //--- Check if the child tag name is "chunk"
          if (chunkElement.tagName() == "chunk")
          {
            QDomNodeList chunkDataNodeList = chunkElement.childNodes();

            //--- get relevant elements
            QDomNode sensorsNode = chunkElement.namedItem("sensors");
            QDomNode camerasNode = chunkElement.namedItem("cameras");
            QDomElement chunkTransformElement = chunkElement.namedItem("transform").toElement();

            //--- parse chunk transform
            QStringList rotationElements = chunkTransformElement.namedItem("rotation").toElement().text().split(" ");
            QStringList translationElements = chunkTransformElement.namedItem("translation").toElement().text().split(" ");
            double scale = chunkTransformElement.namedItem("scale").toElement().text().toDouble();
            cv::Matx44d chunkTransform = cv::Matx44d::eye();
            for(int n = 0; n < 3; ++n)
              for(int m = 0; m < 3; ++m)
                chunkTransform(n, m) = static_cast<double>(scale * rotationElements.at(n * 3 + m).toDouble());
            for(int n = 0; n < 3; ++n)
                chunkTransform(n, 3) = translationElements.at(n).toDouble();

            //--- parse sensor data
            QDomNodeList sensorsNodeList = sensorsNode.childNodes();
            std::map<int, Intrinsics> cameraIntr;
            for(int i = 0; i <  sensorsNodeList.size(); ++i)
            {
              QDomElement sensorElement = sensorsNodeList.at(i).toElement();

              //--- if sensor is not of type frame read next
              if(sensorElement.attribute("type") != "frame")
                continue;

              //--- get camera id
              int camId = sensorElement.attribute("id").toInt();

              //--- read intrinsic data
              Intrinsics intr;
              QDomNode calibrationNode = sensorElement.namedItem("calibration");
              QDomElement resolutionElement = calibrationNode.namedItem("resolution").toElement();
              intr.setImageSize(cv::Size(resolutionElement.attribute("width").toInt(),
                                         resolutionElement.attribute("height").toInt()));
              double f = calibrationNode.namedItem("f").firstChild().toText().data().toDouble();
              intr.setFocalLength(cv::Point2d(f, f));
              double cx = static_cast<double>(intr.getWidth()) / 2.f +
                  calibrationNode.namedItem("cx").firstChild().toText().data().toDouble();
              double cy = static_cast<double>(intr.getHeight()) / 2.f +
                  calibrationNode.namedItem("cy").firstChild().toText().data().toDouble();
              intr.setPrincipalPnt(cv::Point2d(cx, cy));

              //--- read distortion parameters
              std::vector<QString> distortionTags = {"k1", "k2", "p1", "p2", "k3", "k4"};
              std::vector<double> distortionParams;
              for(QString tag : distortionTags)
                distortionParams.push_back(calibrationNode.namedItem(tag).
                                        firstChild().toText().data().toDouble());

              cv::Mat distCoeffs = cv::Mat(cv::Size(1, 8), CV_64FC1);
              for(uint i = 0; i < 8; ++i)
              {
                if(i < distortionParams.size())
                  distCoeffs.at<double>(i,0) = distortionParams[i];
                else
                  distCoeffs.at<double>(i,0) = 0.0;
              }

              intr.setDistortionCoeffs(distCoeffs);

              cameraIntr[camId] = intr;
            }

            if(cameraIntr.empty()){
              continue;
            }

            //--- parse camera data
            std::vector<std::tuple<Extrinsics, int, std::string>> cameraExtrinsics;
            if(camerasNode.firstChild().toElement().tagName() == "group")
            {
              //--- loop over groups and parse camera extriniscs for each group
              QDomNodeList camerasNodeList = camerasNode.childNodes();
              for(int i = 0; i <  camerasNodeList.size(); ++i)
              {
                parseCameraExtr(camerasNodeList.at(i), chunkTransform, cameraExtrinsics);
              }
            }
            else if(camerasNode.firstChild().toElement().tagName() == "camera")
            {
              //--- loop over children and parse camera extrinsics
              parseCameraExtr(camerasNode, chunkTransform, cameraExtrinsics);
            }

            //--- package camera data
            for(std::vector<std::tuple<Extrinsics, int, std::string>>::iterator camExtrItr = cameraExtrinsics.begin();
                camExtrItr != cameraExtrinsics.end(); ++camExtrItr)
            {
              //--- if sensor is not in list, continue
              std::map<int, Intrinsics>::iterator camIntrItr =
                  cameraIntr.find(std::get<1>(*camExtrItr));
              if(camIntrItr == cameraIntr.end())
                continue;

              Camera camera;
              camera.intrinsics = camIntrItr->second;
              camera.extrinsics = std::get<0>(*camExtrItr);

              oCameras[std::get<2>(*camExtrItr)] = camera;
            }
          }

          //--- Next chunk
          chunkElement = chunkElement.nextSibling().toElement();
      }

      return true;
    }


    //--- MEMBER DECLERATION ---//

    private:

      /// Member holding the directory in which the Cameradata is stored e.g. images.txt and cameras.txt
      QFile  mCameraXmlFile;

      /// Map to save camera intrinsics and extrinsics per image name
      std::map<std::string, Camera> mCameras;

};

} // namespace lib3d

#endif // LIB3D_METASHAPEFILESTORAGE_H
