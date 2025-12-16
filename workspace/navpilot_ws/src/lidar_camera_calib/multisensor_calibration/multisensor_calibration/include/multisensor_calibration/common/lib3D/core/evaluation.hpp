#ifndef LIB3D_EVAL_H
#define LIB3D_EVAL_H

// Std
#include <vector>
#include <memory>
#include <array>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <float.h>
#include <math.h>
#include <numeric>


// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace lib3d {


  /**
   @ingroup evaluation
   @brief Method to compute difference of a depth map with respect to a given ground truth.

   Only pixels for which a predicted estimate and a ground truth is given, are evaluated.
   
   @param[in] iComputedDepthmap Predicted depth map given as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
   @param[in] iGroundtruthDepthmap Ground truth depth map given as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
   @param[out] oAbsAvgDifference Average absolute difference between the predicted depth map and its ground truth.
   This can also be considered as the absolute \f$\text{L1}\f$ error:
   \f[\text{L1-abs} = \frac{1}{m}\sum_{m}\left|d-\hat{d}\right|\f]
   With \f$d\f$ and \f$\hat{d}\f$ being the predicted depth and the ground truth depth value, and \f$m\f$ being
   the pixels for which both \f$d\f$ and \f$\hat{d}\f$ exist.
   @param[out] oAbsStdDev Standard deviation of pixel-wise absolute difference between the two depth maps.
   @param[out] oAvgDifference Average difference between the predicted depth map and its ground truth.
   @param[out] oStdDev Standard deviation of pixel-wise absolute difference between the two depth maps.
   @param[out] oL1Rel Relative \f$\text{L1}\f$ error:
   \f[\text{L1-rel} = \frac{1}{m}\sum_{m}\frac{\left|d-\hat{d}\right|}{\hat{d}}\f]
   With \f$d\f$ and \f$\hat{d}\f$ being the predicted depth and the ground truth depth value, and \f$m\f$ being
   the pixels for which both \f$d\f$ and \f$\hat{d}\f$ exist.
   @param[out] oComputedDensity Density of predicted depth map.
   @param[out] oGroundtruthDensity Density of ground truth depth map.
   @param[out] oDiffImage Difference image
   @param[in] iRoiMask Binary mask indicating region of interest. If provided the error is only calculated
   for the pixels for wich iRoiMask = 1.
   @return Difference image. Negative values suggest that the computed depth is samler than the groundtruth.

   @note Input depthmaps have to be of same size and type.
   @note Since lib3d::types::DepthMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
   be passed directly to this function.
   @deprecated
   */
   void computeDepthDifference(const cv::Mat &iComputedDepthmap, const cv::Mat &iGroundtruthDepthmap,
                         float &oAbsAvgDifference, float &oAbsStdDev, float &oAvgDifference,
                         float &oStdDev, float &oL1Rel,
                         float &oComputedDensity, float &oGroundtruthDensity, cv::Mat& oDiffImage,
                         const cv::Mat &iRoiMask = cv::Mat())
  {
    // check for same size and type of input images
    assert(iComputedDepthmap.size().width == iGroundtruthDepthmap.size().width);
    assert(iComputedDepthmap.size().height == iGroundtruthDepthmap.size().height);
    assert(iComputedDepthmap.type() == CV_32FC1 && iGroundtruthDepthmap.type() == CV_32FC1);
    if(!iRoiMask.empty())
    {
      assert(iRoiMask.size().width == iComputedDepthmap.size().width);
      assert(iRoiMask.size().height == iComputedDepthmap.size().height);
      assert(iRoiMask.type() == CV_8UC1);
    }

    //--- noDataMask = 1 if no data in groundtruth and result exists
    cv::Mat noDataMask= cv::Mat::ones(iGroundtruthDepthmap.size(), CV_8UC1);
    cv::Mat availableDataMask= cv::Mat::zeros(iGroundtruthDepthmap.size(), CV_8UC1);
    cv::Mat computedDataMask, groundTruthDataMask;

    cv::threshold(iComputedDepthmap, computedDataMask, 0, 1, cv::THRESH_BINARY);
    computedDataMask.convertTo(computedDataMask, CV_8UC1);
    if(!iRoiMask.empty())
      computedDataMask = computedDataMask.mul(iRoiMask);

    cv::threshold(iGroundtruthDepthmap, groundTruthDataMask, 0, 1, cv::THRESH_BINARY);
    groundTruthDataMask.convertTo(groundTruthDataMask, CV_8UC1);
    if(!iRoiMask.empty())
      groundTruthDataMask = groundTruthDataMask.mul(iRoiMask);

    availableDataMask = computedDataMask.mul(groundTruthDataMask);
    noDataMask.setTo(0, availableDataMask);

    // compute difference between both images
    oDiffImage = iComputedDepthmap - iGroundtruthDepthmap;
    oDiffImage.setTo(0, noDataMask);

    // compute absolute difference between both images
    cv::Mat absDiffImg = cv::Mat(iComputedDepthmap.size(), iComputedDepthmap.type());
    cv::absdiff(iComputedDepthmap, iGroundtruthDepthmap, absDiffImg);
    absDiffImg.setTo(0, noDataMask);

    // compute mean and standard deviation from difference image
    cv::Scalar tmpMean, tmpStdDev;
    cv::meanStdDev(oDiffImage,tmpMean, tmpStdDev, availableDataMask);
    oAvgDifference = static_cast<float>(tmpMean[0]);
    oStdDev = static_cast<float>(tmpStdDev[0]);

    // compute mean and standard deviation from absolute difference image
    cv::meanStdDev(absDiffImg,tmpMean, tmpStdDev, availableDataMask);
    oAbsAvgDifference = static_cast<float>(tmpMean[0]);
    oAbsStdDev = static_cast<float>(tmpStdDev[0]);

    //--- compute relative l1 error
    int nPixelsIntexection = 0;
    int nPixelsComputed = 0;
    int nPixelsGroundTruth = 0;
    oL1Rel = 0;
    for (int x = 0; x < iComputedDepthmap.size().width; ++x) {
      for (int y = 0; y < iComputedDepthmap.size().height; ++y) {
        if(availableDataMask.at<uchar>(y,x) == 1
           && iComputedDepthmap.at<float>(y,x) != NAN
           && iComputedDepthmap.at<float>(y,x) != INFINITY
           && iGroundtruthDepthmap.at<float>(y,x) != NAN
           && iGroundtruthDepthmap.at<float>(y,x) != INFINITY)
        {
          oL1Rel += std::abs(iComputedDepthmap.at<float>(y,x) - iGroundtruthDepthmap.at<float>(y,x)) /
                    iGroundtruthDepthmap.at<float>(y,x);
          nPixelsIntexection++;
        }

        if(!iRoiMask.empty())
        {
          if(computedDataMask.at<uchar>(y,x) == 1 && iRoiMask.at<uchar>(y,x) == 1)
            nPixelsComputed++;
          if(groundTruthDataMask.at<uchar>(y,x) == 1 && iRoiMask.at<uchar>(y,x) == 1)
            nPixelsGroundTruth++;
        }
        else
        {
          if(computedDataMask.at<uchar>(y,x) == 1)
            nPixelsComputed++;
          if(groundTruthDataMask.at<uchar>(y,x) == 1)
            nPixelsGroundTruth++;
        }
      }
    }
    oL1Rel /= nPixelsIntexection;

    //--- compute densities
    oComputedDensity = static_cast<float>(nPixelsComputed) /
        static_cast<float>(iComputedDepthmap.size().width * iComputedDepthmap.size().height);
    oGroundtruthDensity = static_cast<float>(nPixelsGroundTruth) /
        static_cast<float>(iGroundtruthDepthmap.size().width * iGroundtruthDepthmap.size().height);
  }

  /**
    @overload
   */
  cv::Mat computeDepthDifference(const cv::Mat &iComputedDepthmap,
                                 const cv::Mat &iGroundtruthDepthmap,
                                 float &oAbsAvgDifference, float &oAbsStdDev,
                                 float &oAvgDifference, float &oStdDev, float &oL1Rel,
                                 float &oComputedDensity,
                                 float &oGroundtruthDensity,
                                 const cv::Mat &iRoiMask = cv::Mat())
  {
    // check for same size and type of input images
    assert(iComputedDepthmap.size().width == iGroundtruthDepthmap.size().width);
    assert(iComputedDepthmap.size().height == iGroundtruthDepthmap.size().height);
    assert(iComputedDepthmap.type() == CV_32FC1 && iGroundtruthDepthmap.type() == CV_32FC1);

    cv::Mat differenceImg = cv::Mat(iComputedDepthmap.size(), iComputedDepthmap.type());

    //--- compute error ---
    computeDepthDifference(iComputedDepthmap, iGroundtruthDepthmap, oAbsAvgDifference, oAbsStdDev,
                         oAvgDifference, oStdDev, oL1Rel,oComputedDensity,
                      oGroundtruthDensity, differenceImg, iRoiMask);

    return differenceImg;

  }

  /**
   @ingroup evaluation
   @brief Method to compute error of a depth map with respect to a given ground truth.

   Only pixels for which a predicted estimate and a ground truth is given, are evaluated.
   
   @param[in] iEstimatedDepthmap estimated depth map given as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
   @param[in] iGroundtruthDepthmap Ground truth depth map given as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
   @param[out] oL1Abs Average absolute difference between the estimated depth map and its ground truth.
   This can also be considered as the absolute \f$\text{L1}\f$ error:
   \f[\text{L1-abs} = \frac{1}{m}\sum_{m}\left|d-\hat{d}\right|\f]
   With \f$d\f$ and \f$\hat{d}\f$ being the predicted depth and the ground truth depth value, and \f$m\f$ being
   the pixels for which both \f$d\f$ and \f$\hat{d}\f$ exist.
   @param[out] oL1Rel Relative \f$\text{L1}\f$ error:
   \f[\text{L1-rel} = \frac{1}{m}\sum_{m}\frac{\left|d-\hat{d}\right|}{\hat{d}}\f]
   With \f$d\f$ and \f$\hat{d}\f$ being the predicted depth and the ground truth depth value, and \f$m\f$ being
   the pixels for which both \f$d\f$ and \f$\hat{d}\f$ exist.
   @param[out] oDensityEstimate Density of predicted depth map.
   @param[out] oDensityGroundtruth Density of ground truth depth map.
   @param[out] oAccuracy Fraction of estimated depth values, which are within a distance threshold of the ground truth.
   List of thresholds is to be which are to be used for calculation are given in iAccComplThresh.
   @param[out] oCompleteness Amount of ground truth pixel for which their distance to the corresponding estimate is below 
   the evaluation threshold. List of thresholds is to be which are to be used for calculation are given in iAccComplThresh.
   @param[out] oFScore Harmonic mean between oAccuracy and oCompleteness.
   @param[out} oAbsDiffMap Difference map holding per pixel absolute different between estimate and groundtruth.
   @param[in] iAdjustScaling Flag to adjust scaling of estimated depth map in order account for
   scale ambiguities by minimizing pixel-wise quadratic difference between ground truth and estimate.
   @param[in] iAccComplThresh List of thresholds which are to be used for calculation of accuracy and completeness.
   @param[in] iRoiMask Binary mask indicating region of interest. If provided the error is only calculated
   for the pixels for wich iRoiMask = 1.
   @note Input depthmaps have to be of same size and type.
   @note Since lib3d::types::DepthMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
   be passed directly to this function.
   @param T Type of depth maps.
   */
  template<typename T>
  void computeDepthError(const cv::Mat &iEstimatedDepthmap,
                         const cv::Mat &iGroundtruthDepthmap,
                         T& oL1Abs, T& oL1Rel,
                         T& oDensityEstimate,
                         T& oDensityGroundtruth,
                         std::vector<T>& oAccuracy,
                         std::vector<T>& oCompleteness,
                         std::vector<T>& oFScore,
                         cv::Mat& oAbsDiffMap,
                         const bool iAdjustScaling = true,
                         const std::vector<T>& iAccComplThresh = {0.25, 0.2, 0.15, 0.1, 0.05, 0.01},
                         const cv::Mat &iRoiMask = cv::Mat())
  {
    assert(iEstimatedDepthmap.size() == iGroundtruthDepthmap.size());
    assert(iEstimatedDepthmap.type() == cv::DataType<T>::type);
    assert(iEstimatedDepthmap.type() == iEstimatedDepthmap.type());
    if(!iRoiMask.empty())
    {
      assert(iRoiMask.size() == iEstimatedDepthmap.size());
      assert(iRoiMask.type() == CV_8UC1);
    }    

    const cv::Size IMG_SIZE = iEstimatedDepthmap.size();

    cv::Mat tmpEstimatedDepthMap = iEstimatedDepthmap;

    //--- perform uniform scaling of predicted depth map in order to minimizing pixel-wise
    //--- quadratic difference
    float scale = 1.0;
    if(iAdjustScaling)
    {
      float nom = 0.f;
      float denom = 0.f;

      //--- loop over image ---
      for(int y = 0; y < IMG_SIZE.height; y++)
      {
        for(int x = 0; x < IMG_SIZE.width; x++)
        {
          float Dp = tmpEstimatedDepthMap.at<T>(y,x);
          float Dg = iGroundtruthDepthmap.at<T>(y,x);

          if(Dp > 0.f && Dg > 0.f)
          {
            nom += Dp * Dg;
            denom += Dp * Dp;
          }
        }
      }

      scale = nom / (denom + 0.0001);

      tmpEstimatedDepthMap *= scale;
    }


    //--- compute difference between ground truth and estimate and set invalid pixel to zero
    cv::absdiff(iGroundtruthDepthmap, tmpEstimatedDepthMap, oAbsDiffMap);
    oAbsDiffMap.setTo(0, (iGroundtruthDepthmap <= FLT_EPSILON));
    oAbsDiffMap.setTo(0, (tmpEstimatedDepthMap <= FLT_EPSILON));
    if(!iRoiMask.empty()) oAbsDiffMap.setTo(0, (iRoiMask == 0));

    //--- intialize standard error metrics
    oL1Abs = 0;
    oL1Rel = 0;    

    //--- initialize Accuracy Completness vectors
    int numThresh = iAccComplThresh.size();
    oAccuracy = std::vector<T>(numThresh, 0);    
    oCompleteness = std::vector<T>(numThresh, 0);
    oFScore = std::vector<T>(numThresh, 0);

    //--- loop over difference image
    int numDiffPxs = 0;
    for(int y = 0; y < IMG_SIZE.height; ++y)
    {
      for(int x = 0; x < IMG_SIZE.width; ++x)
      {
        T absDiffValue = oAbsDiffMap.at<T>(y,x);

        if(absDiffValue == static_cast<T>(0))
          continue;

        T estimatedDepth = tmpEstimatedDepthMap.at<T>(y,x);
        T gtDepth = iGroundtruthDepthmap.at<T>(y,x);

        //--- update L1 error metrics
        oL1Abs += absDiffValue;
        oL1Rel += (absDiffValue / gtDepth);
        numDiffPxs++;

        //--- update accuracy, completeness metrics
        T maxDepthRatio = std::max(estimatedDepth / gtDepth, gtDepth / estimatedDepth);
        for(uint i = 0; i < iAccComplThresh.size(); ++i)
        {
          if(maxDepthRatio <= (1.0+ iAccComplThresh[i]))
          {
            oAccuracy[i]++;
            oCompleteness[i]++;
          }
        }        
      }
    }

    //--- normalize L1 error metrics
    oL1Abs /= numDiffPxs;
    oL1Rel /= numDiffPxs;

    //--- compute densities
    int numEstimates = cv::countNonZero(tmpEstimatedDepthMap);
    int numGts = cv::countNonZero(iGroundtruthDepthmap);
    oDensityEstimate = static_cast<T>(numEstimates) / IMG_SIZE.area();
    oDensityGroundtruth = static_cast<T>(numGts) / IMG_SIZE.area();
    oDensityEstimate = round(oDensityEstimate * 1000) / 1000; // round to one decimal percent
    oDensityGroundtruth = round(oDensityGroundtruth * 1000) / 1000; // round to one decimal percent

    //--- compute acc, cpl and fscore
    for(uint i = 0; i < iAccComplThresh.size(); ++i)
    {
      oAccuracy[i] /= numEstimates;
      oCompleteness[i] /= numGts;
      oFScore[i] = (2*oAccuracy[i]*oCompleteness[i])/(oAccuracy[i]+oCompleteness[i]);
    }
  }


  /**
   @ingroup evaluation
   @brief Compute Average Angular Error error between two normal maps.
   @param[in] iComputedNormalMap Computed normal map with normalized vectors.
   @param[in] iGroundtruthNormalMap Ground truth normal map with normalized vectors.
   @param[out] oAvgAngError Average Angular Error in radian:
   \f[\text{AAE} = \frac{1}{m}\sum_{m}\delta(\mathrm{n}, \hat{\mathrm{n}})\ ,\ \ \text{with}\ \delta(\mathrm{n}, \hat{\mathrm{n}}) = \cos^{-1}\langle \mathrm{n}, \hat{\mathrm{n}} \rangle.\f]
   With \f$\mathrm{n}\f$ and \f$\hat{\mathrm{n}}\f$ being the predicted normal and the ground truth normal value, and \f$m\f$ being
   the pixels for which both \f$\mathrm{n}\f$ and \f$\hat{\mathrm{n}}\f$ exist. In this, \f$\delta(\mathrm{n}, \hat{\mathrm{n}})\f$ can
   be considered as the enclosed angles between the two vectors. Both normal vectors need to be normalized.
   @param[out] oAngErrorStdDev Standard deviation of the pixel wise average angular error:
   \f[\sigma\text{-AE} = \sqrt{\frac{\sum_{m}\left(\delta(\mathrm{n}, \hat{\mathrm{n}}) - \text{AAE}\right)^2}{m}}.\f]
   Under the assumption that the ground truth is sufficiantly smooth, this will indicate the smoothness of the estimate.
   If \f$\sigma\text{-AE}\f$ is small, it can be assumed that the estimate is just as smooth as the groundtruth,
   but might have a certain offset.
   @param[out] oComputedDensity Density of computed normal map.
   @param[out] oGroundtruthDensity Density of ground truth normal map.
   @param[in] iRoiMask Binary mask indicating region of interest. If provided the error is only calculated
   for the pixels for wich iRoiMask = 1.

   @note Since lib3d::types::NormalMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
   be passed directly to this function.

   */  
  template<typename T>
  void computeNormalError(const cv::Mat &iComputedNormalMap, const cv::Mat &iGroundtruthNormalMap,
                          T &oAvgAngError, T &oAngErrorStdDev,
                          T &oComputedDensity, T &oGroundtruthDensity,
                          const cv::Mat &iRoiMask = cv::Mat())
  {
    // check for same size and type of input images
    assert(iComputedNormalMap.size().width == iGroundtruthNormalMap.size().width);
    assert(iComputedNormalMap.size().height == iGroundtruthNormalMap.size().height);
    assert(iComputedNormalMap.type() == CV_32FC3 && iGroundtruthNormalMap.type() == CV_32FC3);
    if(!iRoiMask.empty())
    {
      assert(iRoiMask.size().width == iComputedNormalMap.size().width);
      assert(iRoiMask.size().height == iComputedNormalMap.size().height);
      assert(iRoiMask.type() == CV_8UC1);
    }

    int nPixelsComputed= 0;
    int nPixelsGroundTruth= 0;

    //--- compute average angular error
    std::vector<T> pixelwiseAngularError;
    for (int x = 0; x < iComputedNormalMap.size().width; ++x) {
      for (int y = 0; y < iComputedNormalMap.size().height; ++y) {
        cv::Vec<T, 3> computedNormal = iComputedNormalMap.at<cv::Vec<T, 3>>(y, x);
        cv::Vec<T, 3> groundTruthNormal = iGroundtruthNormalMap.at<cv::Vec<T, 3>>(y, x);

        bool condition;
        if(!iRoiMask.empty())
        {
          condition = (cv::norm(computedNormal) != 0 && cv::norm(computedNormal) == cv::norm(computedNormal) && // check for nan
                        cv::norm(groundTruthNormal) != 0 && cv::norm(groundTruthNormal) == cv::norm(groundTruthNormal)
                   && iRoiMask.at<uchar>(y,x) == 1);
        }
        else
        {
          condition = (cv::norm(computedNormal) != 0 && cv::norm(computedNormal) == cv::norm(computedNormal)
              && cv::norm(groundTruthNormal) != 0 && cv::norm(groundTruthNormal) == cv::norm(groundTruthNormal));
        }


        if(condition)
        {
          T aae = std::abs(std::acos(cv::normalize(computedNormal).dot(
                                       cv::normalize(groundTruthNormal))));
          if(aae == aae)
            pixelwiseAngularError.push_back(aae);
        }

        if(!iRoiMask.empty())
        {
          if(cv::norm(computedNormal) != 0
             && cv::norm(computedNormal) == cv::norm(computedNormal)
             && iRoiMask.at<uchar>(y,x) == 1)
            nPixelsComputed++;
          if(cv::norm(groundTruthNormal) != 0
             && cv::norm(groundTruthNormal) == cv::norm(groundTruthNormal)
             && iRoiMask.at<uchar>(y,x) == 1)
            nPixelsGroundTruth++;
        }
        else
        {
          if(cv::norm(computedNormal) != 0
             && cv::norm(computedNormal) == cv::norm(computedNormal))
            nPixelsComputed++;
          if(cv::norm(groundTruthNormal) != 0
             && cv::norm(groundTruthNormal) == cv::norm(groundTruthNormal))
            nPixelsGroundTruth++;
        }
      }
    }

    //--- compute angular errors
    oAvgAngError = std::accumulate( pixelwiseAngularError.begin(), pixelwiseAngularError.end(), 0.0)/pixelwiseAngularError.size();
    T sq_sum = 0;
    for(uint i = 0; i < pixelwiseAngularError.size(); i++)
    {
      sq_sum += (pixelwiseAngularError[i]-oAvgAngError)*(pixelwiseAngularError[i]-oAvgAngError);
    }
    oAngErrorStdDev = std::sqrt(sq_sum / pixelwiseAngularError.size());

    //--- compute densities
    oComputedDensity = static_cast<T>(nPixelsComputed) /
        static_cast<T>(iComputedNormalMap.size().width * iComputedNormalMap.size().height);
    oGroundtruthDensity = static_cast<T>(nPixelsGroundTruth) /
        static_cast<T>(iGroundtruthNormalMap.size().width * iGroundtruthNormalMap.size().height);
  }

  /**
   @ingroup evaluation
   @brief Method to compute values for ROC curve in which L1-abs depth error ist evaluated with respect
   to an additional confidence map.

   In this context, a ROC curve represents the error rate as a function of the percentage of pixels
   sampled from a disparity map in the order of increasing uncertainty. More precisely, in the first
   step, the 5% of pixels having the lowest uncertainty, i.e. the highest confidence are sampled from
   an estimated map and the percentage of erroneous pixels in this set is determined. In the second
   step, this procedure is repeated, but using the 10% of pixels with the lowest uncertainty. This
   procedure is further applied in 5% steps, until the full density is reached, so that the error of
   the last set is equal to the overall error of the estimated disparity map.

   In addition, Area Under the Curve (AUC) is computed for the previously estimated ROC curve, which
   can then used to assess the accuracy of a uncertainty map regarding the detection of wrong assignments.

   @param[in] iConfidenceMap Confidence map.
   @param[in] iEstimatedMap Estimated range map. Can be a depth or disparity map, or any other which
   can be evaluated with a absolute L1 measure with respect to the ground truth.
   @param[in] iGroundtruthMap Ground thruth range map of same type as iEstimatedMap
   @param[out] oRocValues Vector of paired values for the ROC curve. The first value of each pair holds
   the density threshold, while the second value holds the corresponding error value.
   @param[out] oAUC Area under the curve.
   @param[out] oOptimalAUC Assuming that an optimal uncertainty map contains smaller values for every
   correct disparity assignment than for any incorrect one, all pixels with a correct disparity
   assigned are sampled before any pixel with an incorrect one. Under this assumption, the optimal
   AUC can be computed directly from the overall error
   @param[in] iAccThresh Accuracy threshold. Default is 0.25, meaning that for each pixel the estimate
   and the ground truth is only allowed to differ up to 25% in their value.
   @param[in] iAdjustScaling Flag to adjust scaling of estimated depth map in order account for
   scale ambiguities by minimizing pixel-wise quadratic difference between ground truth and estimate.
   @param[in] iRoiMask Binary mask indicating region of interest. If provided the error is only calculated
   for the pixels for wich iRoiMask = 1.

   @see Mehltretter, M. & Heipke, C. Aleatoric uncertainty estimation for dense stereo matching via
   CNN-based cost volume analysis ISPRS Journal of Photogrammetry and Remote Sensing, Elsevier BV, 2021

   @note Input maps have to be of same size and type.
   @note Since lib3d::types::NormalMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
   be passed directly to this function.

   */
  template<typename T>
  void computeConfidenceAccROC(const cv::Mat &iConfidenceMap, const cv::Mat &iEstimatedMap,
                               const cv::Mat &iGroundtruthMap,
                               std::vector<std::pair<T,T>>& oRocValues,
                               T& oAUC,
                               T& oOptimalAUC,
                               const float iAccThresh = .05f,
                               const bool iAdjustScaling = true,
                               const cv::Mat &iRoiMask = cv::Mat())
  {
    // check for same size and type of input images
    assert(iConfidenceMap.size() == iEstimatedMap.size());
    assert(iConfidenceMap.size() == iGroundtruthMap.size());
    assert(iConfidenceMap.type() == cv::DataType<T>::type);
    assert(iEstimatedMap.type() == cv::DataType<T>::type);
    assert(iGroundtruthMap.type() == cv::DataType<T>::type);
    if(!iRoiMask.empty())
    {
      assert(iRoiMask.size() == iConfidenceMap.size());
      assert(iRoiMask.type() == CV_8UC1);
    }

    const cv::Size IMG_SIZE = iEstimatedMap.size();

    cv::Mat tmpEstimatedMap = iEstimatedMap;

    //--- perform uniform scaling of predicted depth map in order to minimizing pixel-wise
    //--- quadratic difference
    float scale = 1.0;
    if(iAdjustScaling)
    {
      float nom = 0.f;
      float denom = 0.f;

      //--- loop over image ---
      for(int y = 0; y < IMG_SIZE.height; y++)
      {
        for(int x = 0; x < IMG_SIZE.width; x++)
        {
          float Dp = tmpEstimatedMap.at<T>(y,x);
          float Dg = iGroundtruthMap.at<T>(y,x);

          if(Dp > 0.f && Dg > 0.f)
          {
            nom += Dp * Dg;
            denom += Dp * Dp;
          }
        }
      }

      scale = nom / (denom + 0.0001);

      tmpEstimatedMap *= scale;
    }

    //--- adjust mask
    cv::Mat tmpMask = (!iRoiMask.empty()) ? iRoiMask : cv::Mat::ones(IMG_SIZE,CV_8UC1);
    tmpMask.setTo(0, (iGroundtruthMap <= FLT_EPSILON));
    tmpMask.setTo(0, (tmpEstimatedMap <= FLT_EPSILON));

    //--- compute difference between ground truth and estimate and set invalid pixel to zero
    cv::Mat diffMap = iGroundtruthMap - tmpEstimatedMap;

    //--- loop over diff map, for all non-zero values store confidence and absolute differenct into vector
    std::vector<std::tuple<T,T,T>> confL1AbsAccVector;

    for(int y = 0; y < IMG_SIZE.height; ++y)
    {
      for(int x = 0; x < IMG_SIZE.width; ++x)
      {
        if(tmpMask.at<uchar>(y,x) == 0)
          continue;

        T estimatedDepth = tmpEstimatedMap.at<T>(y,x);
        T gtDepth = iGroundtruthMap.at<T>(y,x);

        T l1Abs = std::abs(diffMap.at<T>(y,x));
        T acc = std::max(estimatedDepth/gtDepth, gtDepth/estimatedDepth);
        T confidence = iConfidenceMap.at<T>(y,x);

        // check for nan
        if(l1Abs == l1Abs && acc == acc && confidence == confidence)
          confL1AbsAccVector.push_back(std::make_tuple(confidence, l1Abs, acc));
      }
    }

    std::sort(confL1AbsAccVector.begin(), confL1AbsAccVector.end());

    float densityStep = 0.05;
    T density = densityStep;
    T err = 0;
    int numPx = 1;

    //--- loop over sorted list of confidence values from top to bottom
    oRocValues.push_back(std::make_pair(0, 0));
    for(int i = (confL1AbsAccVector.size()-1); i >= 0; --i, ++numPx)
    {
      //--- add increase error if acc exceeds acc-threshold
      T acc = std::get<2>(confL1AbsAccVector[i]);
      if(acc >= (1.f+iAccThresh))
        err += 1;

      //--- if density value skips over density threshold, compute mean error and push to output
      if((static_cast<float>(numPx)/confL1AbsAccVector.size()) > density)
      {
        oRocValues.push_back(std::make_pair(density, err / numPx));

        //--- increase density threshold
        density += densityStep;
        density = std::round(density * 100.f) / 100.f;
      }
    }
    //--- finally, insert last values to output
    T overallError = err / numPx;
    oRocValues.push_back(std::make_pair(1, overallError));

    //--- compute AUC and optimalAUC
    oOptimalAUC = overallError + (1-overallError)*log(1-overallError);
    T auc = 0;
    for(int i = 1; i < oRocValues.size(); i++)
    {
      auc += ((oRocValues[i].second + oRocValues[i-1].second) / 2) *
          (oRocValues[i].first - oRocValues[i-1].first);
    }
    oAUC = auc;

  }

  /**
   @ingroup evaluation
   @brief Method to compute mean and variance histogram of a given input image.

   The Bins of the histogram are based on the data of the given reference image.
   @param[in] iInputImg Input image of which the mean and the variance is to be computed.
   @param[in] iReferenceImg Reference image used to generate the histogram bins.
   @param[in] iNBins Number of bins that the histogram should have.
   @return Histogram data. Each vector item in the first dimension holds the data for one
   histogram bin, thus there exist iNBins vector items. Each data bundle (2nd dimension) for one
   histgram bin successivly holds the borders of the bin (min,max), the mean value and the variance
   ( [0] min, [1] max, [2] mean, [3] variance ).
   */
  std::vector<std::array<float, 4>> computeMeanVarianceHistogram(const cv::Mat &iInputImg,
                                                                 const cv::Mat &iReferenceImg,
                                                                 const int &iNBins)
  {
    assert(iInputImg.size().width == iReferenceImg.size().width);
    assert(iInputImg.size().height == iReferenceImg.size().height);
    assert(iInputImg.type() == CV_32FC1 && iReferenceImg.type() == CV_32FC1);

    //--- return value ---
    //--- each vector item holds the data for one histogram bin, thus there exist iNBins vector items
    //--- each data bundle for one histgram bin successivly holds the borders of the bin (min,max), the mean value and the standard deviation
    //---     [0] min, [1] max, [2] mean, [3] std. deviation
    std::vector<std::array<float,4>> histogramData =
        std::vector<std::array<float,4>>(iNBins, {0,0,0,0});

    //--- get min-max value from refernce image and divide range into nBins ---
    double minVal, maxVal, binSize;
    cv::minMaxLoc(iReferenceImg, &minVal, &maxVal);
    binSize = (maxVal - minVal) / iNBins;

    //--- compute bin boarders and store into histogrgamData ---
    std::vector<float> tmpUpperBinBorders = std::vector<float>(iNBins, 0); // temp store for upper border, needed for later search of index.
    histogramData[0][0] = (float) std::floor(minVal); // left most border (min)
    histogramData[iNBins - 1][1] =
      tmpUpperBinBorders[iNBins -1] = (float) std::ceil(maxVal);  // right most border (max)
    for( int i = 1; i < iNBins; i++)
    {
      float borderVal = std::floor(histogramData[0][0] + i * binSize); // border value
      histogramData[i - 1][1] = borderVal; // store border value as upper border of previous bin
      histogramData[i][0] = borderVal; // store border value as lower border of current bin
      tmpUpperBinBorders[i - 1] = borderVal; // store in tmp vector
    }

    //--- loop through input images.
    //--- use value of refernce image to store value of differenc image in the right bin of the histogram
    std::vector<std::vector<float>> tmpData = std::vector<std::vector<float>>(iNBins); // tmp store for bin specific data of the differnce image

  //  #pragma omp for share(iReferenceImg, iDiffImg, tmpData) collapse(2)
    int binIdx;
    for(int y = 0; y < iReferenceImg.size().height; y++)
    {
      for(int x = 0; x < iReferenceImg.size().width; x++)
      {
        //--- find bin corresponding to reference value ---
        float refVal = iReferenceImg.at<float>(y,x);
        typename std::vector<float>::iterator binItr = std::upper_bound(tmpUpperBinBorders.begin(),
                                                           tmpUpperBinBorders.end(), refVal);
        binIdx = binItr - tmpUpperBinBorders.begin();

        //--- store value of difference image in tmpData vector at binIdx ---
        tmpData[binIdx].push_back(iInputImg.at<float>(y,x));
      }
    }

    //--- compute mean and variance value ---
    binIdx = 0;
    auto lmbMeanVarCalc = [&histogramData, &binIdx](std::vector<float> dataVec) {
        float tmpSum = 0,  mean = 0;

        std::for_each(dataVec.begin(), dataVec.end(), [&tmpSum](float val){ tmpSum += val; }); // compute sum of values
        histogramData[binIdx][2] = mean = tmpSum / dataVec.size(); // compute mean value

        tmpSum = 0;
        std::for_each(dataVec.begin(), dataVec.end(), [&tmpSum, &mean](float val){
          tmpSum += std::pow((val - mean),2); // compute sum of squared deviation from mean value
        });
        histogramData[binIdx][3] = tmpSum / dataVec.size(); // compute variance

        binIdx++;
      };
    std::for_each(tmpData.begin(), tmpData.end(), lmbMeanVarCalc);


    return histogramData;

  }

  /**
   @ingroup evaluation
   @brief Method to compute the pixel wise error between two binary masks.
   @param[in] iComputedBinaryMask Computed binary mask.
   @param[in] iGroundtruthBinaryMask Ground truth binary mask.
   @param[out] oTruePositiveRate Rate of true positive, i.e. pixels in which both iComputedBinaryMask = 1 and
   iGroundtruthBinaryMask = 1.
   @param[out] oFalseNegativeRate Rate of false negatives, i.e. pixels in which iComputedBinaryMask = 0 and
   iGroundtruthBinaryMask = 1.
   @param[out] oFalsePositiveRate Rate of false positives, i.e. pixels in which both iComputedBinaryMask = 1 and
   iGroundtruthBinaryMask = 0.
   @param[out] oTrueNegativeRate Rate of false positives, i.e. pixels in which both iComputedBinaryMask = 0 and
   iGroundtruthBinaryMask = 0.
   @return Colored change mask. Green equals regions of true positives, red of flase positives and
   blue of false negatives
   */
  cv::Mat computeBinaryError(const cv::Mat &iComputedBinaryMask,
                             const cv::Mat &iGroundtruthBinaryMask,
                             float &oTruePositiveRate, float &oFalseNegativeRate,
                             float &oFalsePositiveRate, float &oTrueNegativeRate )
  {
    cv::Mat coloredChangeMask;

    //--- check that both change masks are not empty, have the same size and are of type CV_8UC1 ---
    assert(!iComputedBinaryMask.empty() && !iGroundtruthBinaryMask.empty());
    assert(iComputedBinaryMask.size() == iGroundtruthBinaryMask.size());
    assert(iComputedBinaryMask.type() == CV_8UC1 && iGroundtruthBinaryMask.type() == CV_8UC1);

    //--- init results ---
    coloredChangeMask = cv::Mat::zeros(iComputedBinaryMask.size(), CV_8UC3);
    int nTruePositives = 0, nFalseNegatives = 0, nFalsePositives = 0, nTrueNegatives = 0,
        nPositives = 0, nNegatives = 0;

    //--- loop over change masks ---
  //#pragma omp for share(coloredChangeMask, oTruePositiveRate, oFalsePositiveRate, oTruePositiveRate) collapse(2)
    for(int y = 0; y < iComputedBinaryMask.size().height; y++)
    {
      for(int x = 0; x < iComputedBinaryMask.size().width; x++)
      {
        if(iGroundtruthBinaryMask.at<uchar>(y,x) == 255)
        {
          nPositives++;

          //--- true-positives ---
          if(iComputedBinaryMask.at<uchar>(y,x) == 255)
          {
            nTruePositives++;

            //--- set true positives to green ---
            coloredChangeMask.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 255, 0);
          }
          //--- false-negatives ---
          else
          {
            nFalseNegatives++;

            //--- set false negative to blue ---
            coloredChangeMask.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 0, 0);
          }
        }
        else
        {
          nNegatives++;

          //--- false-positives ---
          if(iComputedBinaryMask.at<uchar>(y,x) == 255)
          {
            nFalsePositives++;

            //--- set false positives to red ---
            coloredChangeMask.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 255);
          }
          else
          //--- true-neagtives ---
          {
            nTrueNegatives++;
          }

        }
      }
    }

    //--- calculate rates ---
    oTruePositiveRate = (float)nTruePositives / (float)nPositives;
    oFalseNegativeRate = (float)nFalseNegatives / (float)nPositives;
    oFalsePositiveRate = (float)nFalsePositives / (float)nNegatives;
    oTrueNegativeRate = (float)nTrueNegatives / (float)nNegatives;

    return coloredChangeMask;

  }

} // namespace lib3d

#endif // LIB3D_EVAL_H
