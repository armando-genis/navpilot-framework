#pragma once

#include <filesystem>
#include <vector>
#include <memory>
#include <iostream>
#include <regex>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"

namespace fs = std::filesystem;

class CalibrationLoader
{
public:

    std::vector<std::shared_ptr<CameraUndistorter>> camera_array;
    std::vector<std::shared_ptr<CameraLidarExtrinsics>> extrinsics_array;


    void load(const std::string& calib_dir)
    {
        std::regex cam_intrinsic_re("cam(\\d+)_intrinsic.yaml");
        std::regex lidar_cam_re("LidartoCam(\\d+).yaml");

        std::map<int, std::string> intrinsic_files;
        std::map<int, std::string> extrinsic_files;

        /* scan directory */
        for (const auto& entry : fs::directory_iterator(calib_dir))
        {
            std::string name = entry.path().filename();

            std::smatch match;

            if (std::regex_match(name, match, cam_intrinsic_re))
            {
                int idx = std::stoi(match[1]);
                intrinsic_files[idx] = entry.path();
            }

            if (std::regex_match(name, match, lidar_cam_re))
            {
                int idx = std::stoi(match[1]);
                extrinsic_files[idx] = entry.path();
            }
        }

        int max_cam = 0;

        if(!intrinsic_files.empty())
            max_cam = std::max(max_cam, intrinsic_files.rbegin()->first);

        if(!extrinsic_files.empty())
            max_cam = std::max(max_cam, extrinsic_files.rbegin()->first);

        camera_array.resize(max_cam + 1);
        extrinsics_array.resize(max_cam + 1);

        for(int cam = 0; cam <= max_cam; cam++)
        {
            if(intrinsic_files.count(cam))
                camera_array[cam] = load_intrinsics(intrinsic_files[cam]);

            if(extrinsic_files.count(cam))
                extrinsics_array[cam] = load_extrinsics(extrinsic_files[cam]);
        }

        std::cout << "Calibration intrinsic and extrinsic loaded successfully" << std::endl;

    }

private:

    std::shared_ptr<CameraUndistorter> load_intrinsics(const std::string& path)
    {
        YAML::Node data = YAML::LoadFile(path);

        int w = data["image_width"].as<int>();
        int h = data["image_height"].as<int>();

        std::vector<double> Kvec = data["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> Dvec = data["distortion_coefficients"]["data"].as<std::vector<double>>();

        cv::Mat K(3,3,CV_64F,Kvec.data());
        cv::Mat D(Dvec.size(),1,CV_64F,Dvec.data());

        return std::make_shared<CameraUndistorter>(K.clone(),D.clone(),cv::Size(w,h));
    }

    std::shared_ptr<CameraLidarExtrinsics> load_extrinsics(const std::string& path)
    {
        YAML::Node data = YAML::LoadFile(path);

        auto opencv = data["extrinsics"]["opencv_frame"];
        auto robot  = data["extrinsics"]["robot_frame"];

        cv::Mat R_opencv = mat3(opencv["R"]);
        cv::Mat t_opencv = vec3(opencv["t"]);

        cv::Mat R_robot  = mat3(robot["R"]);
        cv::Mat t_robot  = vec3(robot["t"]);

        LidarRotation rot;
        rot.axis_x = 0;
        rot.axis_y = 0;
        rot.axis_z = 0;

        return std::make_shared<CameraLidarExtrinsics>(
            R_opencv,t_opencv,R_robot,t_robot,rot
        );
    }

    cv::Mat mat3(const YAML::Node& node)
    {
        std::vector<double> v = node["data"].as<std::vector<double>>();
        return cv::Mat(3,3,CV_64F,v.data()).clone();
    }

    cv::Mat vec3(const YAML::Node& node)
    {
        std::vector<double> v = node["data"].as<std::vector<double>>();
        return cv::Mat(3,1,CV_64F,v.data()).clone();
    }
};