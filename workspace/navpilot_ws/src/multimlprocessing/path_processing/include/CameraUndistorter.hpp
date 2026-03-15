#pragma once

#include <opencv2/opencv.hpp>

class CameraUndistorter
{
public:

    CameraUndistorter(const cv::Mat& K_in,
                      const cv::Mat& D_in,
                      const cv::Size& frame_size_wh)
    {
        K = K_in.clone();
        K.convertTo(K, CV_64F);

        D = D_in.clone();
        D.convertTo(D, CV_64F);

        if (D.rows != 4)
            D = D.reshape(1,4);

        frame_size = frame_size_wh;

        compute_maps();
    }

    void ensure_size(int w, int h)
    {
        if (frame_size.width != w || frame_size.height != h)
        {
            frame_size = cv::Size(w, h);
            compute_maps();
        }
    }

    cv::Mat undistort(const cv::Mat& img)
    {
        cv::Mat output;
        cv::remap(img, output, map1, map2, cv::INTER_LINEAR);
        return output;
    }

    cv::Mat get_K() const
    {
        return K;
    }

    cv::Mat get_D() const
    {
        return D;
    }

    cv::Size get_frame_size() const
    {
        return frame_size;
    }

    cv::Mat get_zero_distortion() const
    {
        return cv::Mat::zeros(5,1,CV_64F);
    }

private:

    cv::Mat K;
    cv::Mat D;

    cv::Size frame_size;

    cv::Mat map1;
    cv::Mat map2;

    void compute_maps()
    {
        cv::Mat R = cv::Mat::eye(3,3,CV_64F);

        cv::fisheye::initUndistortRectifyMap(
            K,
            D,
            R,
            K,
            frame_size,
            CV_16SC2,
            map1,
            map2
        );
    }
};