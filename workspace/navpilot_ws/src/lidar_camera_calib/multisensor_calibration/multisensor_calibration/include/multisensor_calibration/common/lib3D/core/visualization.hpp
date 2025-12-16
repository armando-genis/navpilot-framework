#ifndef LIB3D_VIS_RANGE_IMG_H
#define LIB3D_VIS_RANGE_IMG_H

// std
#include <assert.h>
#include <iomanip>
#include <iostream>

#ifndef NO_QT
// Qt
#include <QImage>
#endif

// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#include "common.h"

namespace lib3d
{

/**
 @ingroup visualization
 @brief Color maps to use for colorization.

 This extends the [`cv::ColorMap`](https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html)
 enumeration with a grayscale colormap.
 */
enum EColorMaps
{
    COLORMAP_GRAY    = -1,
    COLORMAP_AUTUMN  = cv::COLORMAP_AUTUMN,
    COLORMAP_BONE    = cv::COLORMAP_BONE,
    COLORMAP_JET     = cv::COLORMAP_JET,
    COLORMAP_WINTER  = cv::COLORMAP_WINTER,
    COLORMAP_RAINBOW = cv::COLORMAP_RAINBOW,
    COLORMAP_OCEAN   = cv::COLORMAP_OCEAN,
    COLORMAP_SUMMER  = cv::COLORMAP_SUMMER,
    COLORMAP_SPRING  = cv::COLORMAP_SPRING,
    COLORMAP_COOL    = cv::COLORMAP_COOL,
    COLORMAP_HSV     = cv::COLORMAP_HSV,
    COLORMAP_PINK    = cv::COLORMAP_PINK,
    COLORMAP_HOT     = cv::COLORMAP_HOT
#if (CV_VERSION_CONCAT >= 346) // 3.4.6
      ,
    COLORMAP_PARULA           = cv::COLORMAP_PARULA,
    COLORMAP_MAGMA            = cv::COLORMAP_MAGMA,
    COLORMAP_INFERNO          = cv::COLORMAP_INFERNO,
    COLORMAP_PLASMA           = cv::COLORMAP_PLASMA,
    COLORMAP_VIRIDIS          = cv::COLORMAP_VIRIDIS,
    COLORMAP_CIVIDIS          = cv::COLORMAP_CIVIDIS,
    COLORMAP_TWILIGHT         = cv::COLORMAP_TWILIGHT,
    COLORMAP_TWILIGHT_SHIFTED = cv::COLORMAP_TWILIGHT_SHIFTED
#endif
#if (CV_VERSION_CONCAT >= 420) // 4.2.0
      ,
    COLORMAP_TURBO = cv::COLORMAP_TURBO
#endif
};

#ifndef NO_QT
/**
 @ingroup visualization
 @brief Enumeration defining order of color channels within cv::Mat from OpenCV.
 */
enum MatColorOrder
{
    MCO_BGR,
    MCO_RGB,
    MCO_BGRA = MCO_BGR,
    MCO_RGBA = MCO_RGB,
    MCO_ARGB
};

inline cv::Mat argb2bgra(const cv::Mat& mat)
{
    Q_ASSERT(mat.channels() == 4);
    cv::Mat newMat(mat.rows, mat.cols, mat.type());
    int from_to[] = {0, 3, 1, 2, 2, 1, 3, 0};
    cv::mixChannels(&mat, 1, &newMat, 1, from_to, 4);
    return newMat;
}

inline cv::Mat adjustChannelsOrder(const cv::Mat& srcMat, MatColorOrder srcOrder, MatColorOrder targetOrder)
{
    Q_ASSERT(srcMat.channels() == 4);
    if (srcOrder == targetOrder)
        return srcMat.clone();
    cv::Mat desMat;
    if ((srcOrder == MCO_ARGB && targetOrder == MCO_BGRA) || (srcOrder == MCO_BGRA && targetOrder == MCO_ARGB))
    {
        // ARGB <==> BGRA
        desMat = argb2bgra(srcMat);
    }
    else if (srcOrder == MCO_ARGB && targetOrder == MCO_RGBA)
    {
        // ARGB ==> RGBA
        desMat        = cv::Mat(srcMat.rows, srcMat.cols, srcMat.type());
        int from_to[] = {0, 3, 1, 0, 2, 1, 3, 2};
        cv::mixChannels(&srcMat, 1, &desMat, 1, from_to, 4);
    }
    else if (srcOrder == MCO_RGBA && targetOrder == MCO_ARGB)
    {
        // RGBA ==> ARGB
        desMat        = cv::Mat(srcMat.rows, srcMat.cols, srcMat.type());
        int from_to[] = {0, 1, 1, 2, 2, 3, 3, 0};
        cv::mixChannels(&srcMat, 1, &desMat, 1, from_to, 4);
    }
    else
    {
        // BGRA <==> RBGA
        cv::cvtColor(srcMat, desMat, cv::COLOR_BGRA2RGBA);
    }
    return desMat;
}

inline QImage::Format findClosestFormat(QImage::Format formatHint)
{
    QImage::Format format;
    switch (formatHint)
    {
    case QImage::Format_Indexed8:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
#if QT_VERSION >= 0x040400
    case QImage::Format_RGB888:
#endif
#if QT_VERSION >= 0x050200
    case QImage::Format_RGBX8888:
    case QImage::Format_RGBA8888:
    case QImage::Format_RGBA8888_Premultiplied:
#endif
#if QT_VERSION >= 0x050500
    case QImage::Format_Alpha8:
    case QImage::Format_Grayscale8:
#endif
        format = formatHint;
        break;
    case QImage::Format_Mono:
    case QImage::Format_MonoLSB:
        format = QImage::Format_Indexed8;
        break;
    case QImage::Format_RGB16:
        format = QImage::Format_RGB32;
        break;
#if QT_VERSION > 0x040400
    case QImage::Format_RGB444:
    case QImage::Format_RGB555:
    case QImage::Format_RGB666:
        format = QImage::Format_RGB888;
        break;
    case QImage::Format_ARGB4444_Premultiplied:
    case QImage::Format_ARGB6666_Premultiplied:
    case QImage::Format_ARGB8555_Premultiplied:
    case QImage::Format_ARGB8565_Premultiplied:
        format = QImage::Format_ARGB32_Premultiplied;
        break;
#endif
    default:
        format = QImage::Format_ARGB32;
        break;
    }
    return format;
}

inline MatColorOrder getColorOrderOfRGB32Format()
{
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
    return MCO_BGRA;
#else
    return MCO_ARGB;
#endif
}

/**
 @ingroup visualization
 @brief Convert [`QImage`](https://doc.qt.io/qt-5/qimage.html) to [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) without data copy.

 - Supported QImage formats and cv::Mat types are:
   - QImage::Format_Indexed8               <==> CV_8UC1
   - QImage::Format_Alpha8                 <==> CV_8UC1
   - QImage::Format_Grayscale8             <==> CV_8UC1
   - QImage::Format_RGB888                 <==> CV_8UC3 (R G B)
   - QImage::Format_RGB32                  <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_ARGB32                 <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_ARGB32_Premultiplied   <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_RGBX8888               <==> CV_8UC4 (R G B A)
   - QImage::Format_RGBA8888               <==> CV_8UC4 (R G B A)
   - QImage::Format_RGBA8888_Premultiplied <==> CV_8UC4 (R G B A)

 - For QImage::Format_RGB32 ,QImage::Format_ARGB32
   and QImage::Format_ARGB32_Premultiplied, the
   color channel order of cv::Mat will be (B G R A) in little
   endian system or (A R G B) in big endian system.

 @note User must make sure that the color channels order is the same as the color channels order requried by QImage.
 @see https://github.com/dbzhang800/QtOpenCV
 */
inline cv::Mat qimage2cvMat_shared(const QImage& img, MatColorOrder* order = 0)
{
    if (img.isNull())
        return cv::Mat();
    switch (img.format())
    {
    case QImage::Format_Indexed8:
        break;
#if QT_VERSION >= 0x040400
    case QImage::Format_RGB888:
        if (order)
            *order = MCO_RGB;
        break;
#endif
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
        if (order)
            *order = getColorOrderOfRGB32Format();
        break;
#if QT_VERSION >= 0x050200
    case QImage::Format_RGBX8888:
    case QImage::Format_RGBA8888:
    case QImage::Format_RGBA8888_Premultiplied:
        if (order)
            *order = MCO_RGBA;
        break;
#endif
#if QT_VERSION >= 0x050500
    case QImage::Format_Alpha8:
    case QImage::Format_Grayscale8:
        break;
#endif
    default:
        return cv::Mat();
    }
    return cv::Mat(img.height(), img.width(), CV_8UC(img.depth() / 8), (uchar*)img.bits(), img.bytesPerLine());
}

/**
 @ingroup visualization
 @brief Convert [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) to [`QImage`](https://doc.qt.io/qt-5/qimage.html) without data copy.

 - Supported QImage formats and cv::Mat types are:
   - QImage::Format_Indexed8               <==> CV_8UC1
   - QImage::Format_Alpha8                 <==> CV_8UC1
   - QImage::Format_Grayscale8             <==> CV_8UC1
   - QImage::Format_RGB888                 <==> CV_8UC3 (R G B)
   - QImage::Format_RGB32                  <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_ARGB32                 <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_ARGB32_Premultiplied   <==> CV_8UC4 (A R G B or B G R A)
   - QImage::Format_RGBX8888               <==> CV_8UC4 (R G B A)
   - QImage::Format_RGBA8888               <==> CV_8UC4 (R G B A)
   - QImage::Format_RGBA8888_Premultiplied <==> CV_8UC4 (R G B A)

 - For QImage::Format_RGB32 ,QImage::Format_ARGB32
   and QImage::Format_ARGB32_Premultiplied, the
   color channel order of cv::Mat will be (B G R A) in little
   endian system or (A R G B) in big endian system.

 @note User must make sure that the color channels order is the same as the color channels order requried by QImage.
 @see https://github.com/dbzhang800/QtOpenCV
 */
inline QImage cvMat2QImage_shared(const cv::Mat& mat, QImage::Format formatHint = QImage::Format_Invalid)
{
    Q_ASSERT(mat.type() == CV_8UC1 || mat.type() == CV_8UC3 || mat.type() == CV_8UC4);
    if (mat.empty())
        return QImage();
    // Adjust formatHint if needed.
    if (mat.type() == CV_8UC1)
    {
        if (formatHint != QImage::Format_Indexed8
#if QT_VERSION >= 0x050500
            && formatHint != QImage::Format_Alpha8 && formatHint != QImage::Format_Grayscale8
#endif
        )
        {
            formatHint = QImage::Format_Indexed8;
        }
#if QT_VERSION >= 0x040400
    }
    else if (mat.type() == CV_8UC3)
    {
        formatHint = QImage::Format_RGB888;
#endif
    }
    else if (mat.type() == CV_8UC4)
    {
        if (formatHint != QImage::Format_RGB32 && formatHint != QImage::Format_ARGB32 && formatHint != QImage::Format_ARGB32_Premultiplied
#if QT_VERSION >= 0x050200
            && formatHint != QImage::Format_RGBX8888 && formatHint != QImage::Format_RGBA8888 && formatHint != QImage::Format_RGBA8888_Premultiplied
#endif
        )
        {
            formatHint = QImage::Format_ARGB32;
        }
    }
    QImage img(mat.data, mat.cols, mat.rows, mat.step, formatHint);
    // Should we add directly support for user-customed-colorTable?
    if (formatHint == QImage::Format_Indexed8)
    {
        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; ++i)
            colorTable.append(qRgb(i, i, i));
        img.setColorTable(colorTable);
    }
    return img;
}

/**
 @ingroup visualization
 @brief Method to convert [`QImage`](https://doc.qt.io/qt-5/qimage.html) to [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
   - cv::Mat
     - Supported channels
       - 1 channel
       - 3 channels (B G R), (R G B)
       - 4 channels (B G R A), (R G B A), (A R G B)
     - Supported depth
       - CV_8U  [0, 255]
       - CV_16U [0, 65535]
       - CV_32F [0, 1.0]
   - QImage
     - All of the formats of QImage are supported.

  @see https://github.com/dbzhang800/QtOpenCV
 */
inline cv::Mat qimage2cvMat(const QImage& img, int requiredMatType = CV_8UC(uint(0)),
                            MatColorOrder requiredOrder = MCO_BGR)
{
    int targetDepth    = CV_MAT_DEPTH(requiredMatType);
    int targetChannels = CV_MAT_CN(requiredMatType);
    Q_ASSERT(targetChannels == CV_CN_MAX || targetChannels == 1 || targetChannels == 3 || targetChannels == 4);
    Q_ASSERT(targetDepth == CV_8U || targetDepth == CV_16U || targetDepth == CV_32F);
    if (img.isNull())
        return cv::Mat();
    // Find the closest image format that can be used in image2Mat_shared()
    QImage::Format format = lib3d::findClosestFormat(img.format());
    QImage image          = (format == img.format()) ? img : img.convertToFormat(format);
    lib3d::MatColorOrder srcOrder;
    cv::Mat mat0 = qimage2cvMat_shared(image, &srcOrder);
    // Adjust mat channells if needed.
    cv::Mat mat_adjustCn;
    const float maxAlpha = targetDepth == CV_8U ? 255 : (targetDepth == CV_16U ? 65535 : 1.0);
    if (targetChannels == CV_CN_MAX)
        targetChannels = mat0.channels();
    switch (targetChannels)
    {
    case 1:
        if (mat0.channels() == 3)
        {
            cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2GRAY);
        }
        else if (mat0.channels() == 4)
        {
            if (srcOrder == MCO_BGRA)
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_BGRA2GRAY);
            else if (srcOrder == MCO_RGBA)
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGBA2GRAY);
            else // MCO_ARGB
                cv::cvtColor(argb2bgra(mat0), mat_adjustCn, cv::COLOR_BGRA2GRAY);
        }
        break;
    case 3:
        if (mat0.channels() == 1)
        {
            cv::cvtColor(mat0, mat_adjustCn, requiredOrder == MCO_BGR ? cv::COLOR_GRAY2BGR : cv::COLOR_GRAY2RGB);
        }
        else if (mat0.channels() == 3)
        {
            if (requiredOrder != srcOrder)
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2BGR);
        }
        else if (mat0.channels() == 4)
        {
            if (srcOrder == MCO_ARGB)
            {
                mat_adjustCn   = cv::Mat(mat0.rows, mat0.cols, CV_MAKE_TYPE(mat0.type(), 3));
                int ARGB2RGB[] = {1, 0, 2, 1, 3, 2};
                int ARGB2BGR[] = {1, 2, 2, 1, 3, 0};
                cv::mixChannels(&mat0, 1, &mat_adjustCn, 1, requiredOrder == MCO_BGR ? ARGB2BGR : ARGB2RGB, 3);
            }
            else if (srcOrder == MCO_BGRA)
            {
                cv::cvtColor(mat0, mat_adjustCn, requiredOrder == MCO_BGR ? cv::COLOR_BGRA2BGR : cv::COLOR_BGRA2RGB);
            }
            else
            { // RGBA
                cv::cvtColor(mat0, mat_adjustCn, requiredOrder == MCO_BGR ? cv::COLOR_RGBA2BGR : cv::COLOR_RGBA2RGB);
            }
        }
        break;
    case 4:
        if (mat0.channels() == 1)
        {
            if (requiredOrder == MCO_ARGB)
            {
                cv::Mat alphaMat(mat0.rows, mat0.cols, CV_MAKE_TYPE(mat0.type(), 1), cv::Scalar(maxAlpha));
                mat_adjustCn  = cv::Mat(mat0.rows, mat0.cols, CV_MAKE_TYPE(mat0.type(), 4));
                cv::Mat in[]  = {alphaMat, mat0};
                int from_to[] = {0, 0, 1, 1, 1, 2, 1, 3};
                cv::mixChannels(in, 2, &mat_adjustCn, 1, from_to, 4);
            }
            else if (requiredOrder == MCO_RGBA)
            {
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_GRAY2RGBA);
            }
            else
            { // MCO_BGRA
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_GRAY2BGRA);
            }
        }
        else if (mat0.channels() == 3)
        {
            if (requiredOrder == MCO_ARGB)
            {
                cv::Mat alphaMat(mat0.rows, mat0.cols, CV_MAKE_TYPE(mat0.type(), 1), cv::Scalar(maxAlpha));
                mat_adjustCn  = cv::Mat(mat0.rows, mat0.cols, CV_MAKE_TYPE(mat0.type(), 4));
                cv::Mat in[]  = {alphaMat, mat0};
                int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
                cv::mixChannels(in, 2, &mat_adjustCn, 1, from_to, 4);
            }
            else if (requiredOrder == MCO_RGBA)
            {
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2RGBA);
            }
            else
            { // MCO_BGRA
                cv::cvtColor(mat0, mat_adjustCn, cv::COLOR_RGB2BGRA);
            }
        }
        else if (mat0.channels() == 4)
        {
            if (srcOrder != requiredOrder)
                mat_adjustCn = adjustChannelsOrder(mat0, srcOrder, requiredOrder);
        }
        break;
    default:
        break;
    }
    // Adjust depth if needed.
    if (targetDepth == CV_8U)
        return mat_adjustCn.empty() ? mat0.clone() : mat_adjustCn;
    if (mat_adjustCn.empty())
        mat_adjustCn = mat0;
    cv::Mat mat_adjustDepth;
    mat_adjustCn.convertTo(mat_adjustDepth, CV_MAKE_TYPE(targetDepth, mat_adjustCn.channels()), targetDepth == CV_16U ? 255.0 : 1 / 255.0);
    return mat_adjustDepth;
}

/**
 @ingroup visualization
 @brief Method to convert [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) to [`QImage`](https://doc.qt.io/qt-5/qimage.html).
   - cv::Mat
     - Supported channels
       - 1 channel
       - 3 channels (B G R), (R G B)
       - 4 channels (B G R A), (R G B A), (A R G B)
     - Supported depth
       - CV_8U  [0, 255]
       - CV_16U [0, 65535]
       - CV_32F [0, 1.0]
   - QImage
     - All of the formats of QImage are supported.

  @see https://github.com/dbzhang800/QtOpenCV
 */
inline QImage cvMat2QImage(const cv::Mat& mat, MatColorOrder order = MCO_BGR, QImage::Format formatHint = QImage::Format_Invalid)
{
    Q_ASSERT(mat.channels() == 1 || mat.channels() == 3 || mat.channels() == 4);
    Q_ASSERT(mat.depth() == CV_8U || mat.depth() == CV_16U || mat.depth() == CV_32F);
    if (mat.empty())
        return QImage();
    // Adjust mat channels if needed, and find proper QImage format.
    QImage::Format format = formatHint;
    cv::Mat mat_adjustCn;
    if (mat.channels() == 1)
    {
        format = formatHint;
        if (formatHint != QImage::Format_Indexed8
#if QT_VERSION >= 0x050500
            && formatHint != QImage::Format_Alpha8 && formatHint != QImage::Format_Grayscale8
#endif
        )
        {
            format = QImage::Format_Indexed8;
        }
    }
    else if (mat.channels() == 3)
    {
#if QT_VERSION >= 0x040400
        format = QImage::Format_RGB888;
        if (order == MCO_BGR)
            cv::cvtColor(mat, mat_adjustCn, cv::COLOR_BGR2RGB);
#else
        format = QImage::Format_RGB32;
        cv::Mat mat_tmp;
        cv::cvtColor(mat, mat_tmp, order == MCO_BGR ? cv::COLOR_BGR2BGRA : cv::COLOR_RGB2BGRA);
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
        mat_adjustCn = mat_tmp;
#else
        mat_adjustCn = argb2bgra(mat_tmp);
#endif
#endif
    }
    else if (mat.channels() == 4)
    {
        // Find best format if the formatHint can not be applied.
        format = findClosestFormat(formatHint);
        if (format != QImage::Format_RGB32 && format != QImage::Format_ARGB32 && format != QImage::Format_ARGB32_Premultiplied
#if QT_VERSION >= 0x050200
            && format != QImage::Format_RGBX8888 && format != QImage::Format_RGBA8888 && format != QImage::Format_RGBA8888_Premultiplied
#endif
        )
        {
#if QT_VERSION >= 0x050200
            format = order == MCO_RGBA ? QImage::Format_RGBA8888 : QImage::Format_ARGB32;
#else
            format = QImage::Format_ARGB32;
#endif
        }
        // Channel order requried by the target QImage
        MatColorOrder requiredOrder = getColorOrderOfRGB32Format();
#if QT_VERSION >= 0x050200
        if (formatHint == QImage::Format_RGBX8888 || formatHint == QImage::Format_RGBA8888 || formatHint == QImage::Format_RGBA8888_Premultiplied)
        {
            requiredOrder = MCO_RGBA;
        }
#endif
        if (order != requiredOrder)
            mat_adjustCn = adjustChannelsOrder(mat, order, requiredOrder);
    }
    if (mat_adjustCn.empty())
        mat_adjustCn = mat;
    // Adjust mat depth if needed.
    cv::Mat mat_adjustDepth = mat_adjustCn;
    if (mat.depth() != CV_8U)
        mat_adjustCn.convertTo(mat_adjustDepth, CV_8UC(mat_adjustCn.channels()), mat.depth() == CV_16U ? 1 / 255.0 : 255.0);
    // Should we convert the image to the format specified by formatHint?
    QImage image = cvMat2QImage_shared(mat_adjustDepth, format);
    if (format == formatHint || formatHint == QImage::Format_Invalid)
        return image.copy();
    else
        return image.convertToFormat(formatHint);
}
#endif

inline cv::Mat normalizeRangeImg(const cv::Mat iRangeImg, cv::Mat& oNaThresholdMask,
                                 cv::Mat& oMinThresholdMask, cv::Mat& oMaxThresholdMask,
                                 float* iopMinRange, float* iopMaxRange)
{
    cv::Mat colorizedRangeImg;

    // convert values to float and copy to colorized depthmap
    iRangeImg.convertTo(colorizedRangeImg, CV_32FC1);

    //--- if minDepth is not set or set to -1, clamp negative values to 0 ---
    if (!iopMinRange || std::fabs(*iopMinRange - -1.f) < 0.0001)
    {
        cv::threshold(iRangeImg, oNaThresholdMask, 0., 1., cv::THRESH_BINARY_INV);
        oNaThresholdMask.convertTo(oNaThresholdMask, CV_8UC1);
    }

    //--- NORMALIZE RANGE IMAGE ---

    //--- get min max of depth map for areas which are outside of naThreshold ---
    double minVal, maxVal;
    cv::minMaxLoc(oNaThresholdMask, &minVal, &maxVal);
    cv::minMaxLoc(colorizedRangeImg, &minVal, &maxVal, 0, 0,
                  (std::fabs(maxVal) < 0.0001) ? cv::noArray() : cv::Mat((oNaThresholdMask - 1) * -1));

    //--- minDepth = -1, maxDepth = -1 -> use min max of depth map ---
    if ((!iopMinRange && !iopMaxRange) ||
        (std::fabs(*iopMinRange - -1.f) < 0.0001 && std::fabs(*iopMaxRange - -1.f) < 0.0001))
    {
        colorizedRangeImg -= minVal;
        colorizedRangeImg /= (maxVal - minVal);

        if (iopMinRange)
            *iopMinRange = minVal;
        if (iopMaxRange)
            *iopMaxRange = maxVal;
    }
    //--- minDepth != -1, maxDepth = -1 -> use given min, use max of depth map ---
    else if (iopMinRange && std::fabs(*iopMinRange - -1.f) >= 0.0001 &&
             (!iopMaxRange || std::fabs(*iopMaxRange - -1.f) < 0.0001))
    {
        // get mask of pixels that are below minVal
        cv::threshold(iRangeImg, oMinThresholdMask, *iopMinRange, 1., cv::THRESH_BINARY_INV);
        oMinThresholdMask.convertTo(oMinThresholdMask, CV_8UC1);

        colorizedRangeImg -= *iopMinRange;
        colorizedRangeImg /= (maxVal - *iopMinRange);

        if (iopMaxRange)
            *iopMaxRange = maxVal;
    }
    //--- minDepth = -1, maxDepth != -1 -> use given max, use min of depth map ---
    else if (iopMaxRange && std::fabs(*iopMaxRange - -1.f) >= 0.0001 &&
             (!iopMinRange || std::fabs(*iopMinRange - -1.f) < 0.0001))
    {
        // get mask of pixels that are above maxVal
        cv::threshold(iRangeImg, oMaxThresholdMask, *iopMaxRange, 1., cv::THRESH_BINARY);
        oMaxThresholdMask.convertTo(oMaxThresholdMask, CV_8UC1);

        colorizedRangeImg -= minVal;
        colorizedRangeImg /= (*iopMaxRange - minVal);

        if (iopMinRange)
            *iopMinRange = minVal;
    }
    //--- minDepth != -1, maxDepth != -1 -> use given min and max ---
    else
    {
        // get mask of pixels that are below minVal
        cv::threshold(iRangeImg, oMinThresholdMask, *iopMinRange, 1., cv::THRESH_BINARY_INV);
        oMinThresholdMask.convertTo(oMinThresholdMask, CV_8UC1);

        // get mask of pixels that are above maxVal
        cv::threshold(iRangeImg, oMaxThresholdMask, *iopMaxRange, 1., cv::THRESH_BINARY);
        oMaxThresholdMask.convertTo(oMaxThresholdMask, CV_8UC1);

        colorizedRangeImg -= *iopMinRange;
        colorizedRangeImg /= (*iopMaxRange - *iopMinRange);
    }

    return colorizedRangeImg;
}

/**
 @ingroup visualization
 @brief Method to colorize range image and generate scale image.
 @param[in] iRangeImg Range image to colorize.
 Since lib3d::types::DepthMap and lib3d::types::DisparityMap are a subclass of
 [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can be passed directly to this function.
 @param[in] iColorMap Color map to use. Default = COLORMAP_JET.
 @param[in] iInvert If true, invert range. Default = false.
 @param[in,out] iopMinRange Minimum depth to be assumed. If nullptr or value equals -1 then min value of iRangeImg is assumed.
 @param[in,out] iopMaxRange Maximum depth to be assumed. If nullptr nullptr or value equals -1 then max value of iRangeImg is assumed.
 @param[in] iBackgroundImg Image to use as background. Pixels with no depth will be fully transparrent.
 Pixel with depth will be overlayed with a blending factor of alpha (\f$\alpha = 1\f$ is opaque).
 @param[in] iAlpha Blending factor used for pixel with depth data (\f$\alpha = 1\f$ is opaque).
 @return Colorized range image as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) in format CV_8UC3.
 Pixels for which no data is available or for which the data is outside [iMinRange, iMaxRange] are black.
 */
inline cv::Mat colorizeRangeImg(const cv::Mat& iRangeImg,
                                const EColorMaps iColorMap = COLORMAP_JET,
                                const bool iInvert         = false,
                                float* iopMinRange = nullptr, float* iopMaxRange = nullptr,
                                const cv::Mat& iBackgroundImg = cv::Mat(), const float iAlpha = 0.8f)
{
    //--- check that input data is correct ---
    assert(!iRangeImg.empty());
    assert((iopMinRange == nullptr && iopMaxRange == nullptr) || (iopMinRange != nullptr && iopMaxRange != nullptr));

    cv::Mat naThresholdMask  = cv::Mat::zeros(iRangeImg.size(), CV_8UC1);
    cv::Mat minThresholdMask = cv::Mat::zeros(iRangeImg.size(), CV_8UC1);
    cv::Mat maxThresholdMask = cv::Mat::zeros(iRangeImg.size(), CV_8UC1);

    //--- normalize ---
    cv::Mat normalizedRangeImg = normalizeRangeImg(iRangeImg, naThresholdMask, minThresholdMask,
                                                   maxThresholdMask, iopMinRange, iopMaxRange);

    //--- invert if required
    if (iInvert)
    {
        normalizedRangeImg *= -1.f;
        normalizedRangeImg += 1.f;
    }

    //--- colorize
    cv::Mat colorizedRangeImg;
    normalizedRangeImg.copyTo(colorizedRangeImg);
    if (iColorMap == COLORMAP_GRAY)
    {
        // color with grayscale
        colorizedRangeImg *= 255.;
        colorizedRangeImg.convertTo(colorizedRangeImg, CV_8UC1);
    }
    else
    {
        //--- apply colormap
        colorizedRangeImg.convertTo(colorizedRangeImg, CV_8UC1, 255);
        cv::applyColorMap(colorizedRangeImg, colorizedRangeImg, iColorMap);
    }

    //--- blend with background image
    if (!iBackgroundImg.empty())
    {
        cv::Mat backImg;
        if (colorizedRangeImg.size() != iBackgroundImg.size())
            cv::resize(iBackgroundImg, backImg, colorizedRangeImg.size());
        else
            iBackgroundImg.copyTo(backImg);

        //--- if background image is mono convert to CV_8UC3
        if (backImg.type() == CV_8UC1)
            cv::cvtColor(backImg, backImg, cv::COLOR_GRAY2BGR);

        //--- if colormap is gray, convert colorized depth map to CV_8UC3
        if (iColorMap == COLORMAP_GRAY)
            cv::cvtColor(colorizedRangeImg, colorizedRangeImg, cv::COLOR_GRAY2BGR);

        float alpha = std::min(iAlpha, 1.f);
        float beta  = 1.f - alpha;
        cv::addWeighted(colorizedRangeImg, alpha, backImg, beta, 0.0, colorizedRangeImg);

        //--- set values outside threshold to background ---
        backImg.copyTo(colorizedRangeImg, naThresholdMask | minThresholdMask | maxThresholdMask);
    }
    else
    {
        //--- set values outside threshold to black ---
        colorizedRangeImg.setTo(cv::Scalar(0), naThresholdMask | minThresholdMask | maxThresholdMask);
    }

    return colorizedRangeImg;
}

/**
 @ingroup visualization
 @brief Method to generate a scale image with a given colormap.
 @param[in] iMinRange Minimum depth.
 @param[in] iMaxRange Maximum depth.
 @param[in] iColorMap Colormap to use. Default = COLORMAP_JET.
 @param[in] iInvert If true, invert range. Default = false.
 @param[in] iScaleHeight Height of scale image. Default = 480.
 @return [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) image showing a scale.
 This will hold a color scale/legend with markings of the minimum and maximum value.
 */
inline cv::Mat generateScaleImg(const float iMinRange, const float iMaxRange,
                                const EColorMaps iColorMap = COLORMAP_JET,
                                const bool iInvert         = false,
                                const int iScaleHeight     = 480)
{
    cv::Mat scaleImage;

    //--- get min max of depth map for areas which are outside of naThreshold ---
    std::stringstream minTxt, maxTxt;
    minTxt << std::fixed << std::setprecision(2) << iMinRange;
    maxTxt << std::fixed << std::setprecision(2) << iMaxRange;

    //--- create scale image ---
    scaleImage = cv::Mat::zeros(iScaleHeight, 200, CV_32FC1);
    {
        for (int y = 0; y <= iScaleHeight; y++)
        {
            cv::line(scaleImage, cv::Point2i(0, y), cv::Point2i(60, y),
                     1 - ((float)(y) / (float)(iScaleHeight)));
        }
    }

    // invert if required
    if (iInvert)
    {
        scaleImage *= -1.f;
        scaleImage += 1.f;
    }

    //--- colorize
    if (iColorMap == COLORMAP_GRAY)
    {
        //--- color with grayscale
        scaleImage *= 255.f;
        scaleImage.convertTo(scaleImage, CV_8UC1);
    }
    else
    {
        //--- apply colormap
        scaleImage.convertTo(scaleImage, CV_8UC1, 255);
        cv::applyColorMap(scaleImage, scaleImage, iColorMap);
    }

    //--- FINISH SCALE ---

    //--- white background ---
    cv::rectangle(scaleImage, cv::Point2i(60, 0), cv::Point2i(scaleImage.size().width, scaleImage.size().height),
                  CV_RGB(255, 255, 255), -1);

    //--- black border ---
    cv::rectangle(scaleImage, cv::Point2i(0, 0), cv::Point2i(60, scaleImage.size().height - 1),
                  CV_RGB(0, 0, 0));

    //--- write labels ---
    cv::line(scaleImage, cv::Point2i(65, 0), cv::Point2i(90, 0),
             CV_RGB(0, 0, 0));
    cv::line(scaleImage, cv::Point2i(65, scaleImage.size().height - 1),
             cv::Point2i(90, scaleImage.size().height - 1),
             CV_RGB(0, 0, 0));
    cv::putText(scaleImage, maxTxt.str(), cv::Point2i(70, 25), cv::FONT_HERSHEY_COMPLEX, 1,
                CV_RGB(0, 0, 0), 2);
    cv::putText(scaleImage, minTxt.str(), cv::Point2i(70, scaleImage.size().height - 5),
                cv::FONT_HERSHEY_COMPLEX, 1,
                CV_RGB(0, 0, 0), 2);

    return scaleImage;
}

/**
 @ingroup visualization
 @brief Method to colorize range image and generate scale image.
 @param[in] iRangeImg Range image to colorize.
 Since lib3d::types::DepthMap and lib3d::types::DisparityMap are a subclass of
 [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can be passed directly to this function.
 @param[out] oScaleImg [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) image showing a scale.
 This will hold a color scale/legend with markings of the minimum and maximum value.
 @param[in] iColorMap Color map to use. Default = COLORMAP_JET.
 @param[in] iInvert If true, invert range. Default = false.
 @param[in] iMinRange Minimum depth to be assumed. If -1 then min value of iRangeImg is assumed.
 Default = -1.
 @param[in] iMaxRange Maximum depth to be assumed. If -1 then max value of iRangeImg is assumed.
 Default = -1.
 @param[in] iBackgroundImg Image to use as background. Pixels with no depth will be fully transparrent.
 Pixel with depth will be overlayed with a blending factor of alpha (\f$\alpha = 1\f$ is opaque).
 @param[in] iAlpha Blending factor used for pixel with depth data (\f$\alpha = 1\f$ is opaque).
 @return Colorized range image as [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) in format CV_8UC3.
 Pixels for which no data is available or for which the data is outside [iMinRange, iMaxRange] are black.

 @note Since lib3d::types::DepthMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
 be passed directly to this function.
 */
inline cv::Mat colorizeRangeImg(const cv::Mat& iRangeImg, cv::Mat& oScaleImg,
                                const EColorMaps iColorMap = COLORMAP_JET,
                                const bool iInvert         = false,
                                const float iMinRange = -1.f, const float iMaxRange = -1.f,
                                const cv::Mat& iBackgroundImg = cv::Mat(), const float iAlpha = 0.8f)
{
    //--- return variable ---
    cv::Mat coloredRangeImg;

    float minRange = iMinRange;
    float maxRange = iMaxRange;

    coloredRangeImg = colorizeRangeImg(iRangeImg, iColorMap, iInvert, &minRange, &maxRange,
                                       iBackgroundImg, iAlpha);
    oScaleImg       = generateScaleImg(minRange, maxRange, iColorMap, iInvert, iRangeImg.size().height);

    return coloredRangeImg;
}

/**
 @ingroup visualization
 @brief Method to colorize normal map for visualization.

 Colorization is performed in RGB. Vectors in normal map need to be normalized.
 Before colorization the normal vectors are shifted into the range of [0,1].
 It is assumed that the normal vectors are given in the coordinate system of the camera
 (i.e. x - right, y - down, z - forward).
 @param[in] iNormalMap Normal map to colorize.
 Since lib3d::types::NormalMap is a subclass of [`cv::Mat`](https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html) they can
 be passed directly to this function.
 @param[in] iCvt2OpenglConvention Flag to convert normal into opengl convention prior to colorization.
 This will rotate the reference coordinate system by 180&deg; around the x-axis. Areas in which the
 normal vector points towards the camera will apear in a violet tone. Upwards facing areas will apear
 light green.

 @return Colorized normal map. In format CV_8UC3.

 <b>Example:</b>

 @image html vis-normal-ref.png "Reference image"  @image html vis-normal-cv.png "NormalMap - Camera coordinate system"  @image html vis-normal-ogl.png "NormalMap - OpenGL convention"
 */
inline cv::Mat colorizeNormalMap(const cv::Mat& iNormalMap, const bool iCvt2OpenglConvention = true)
{
    cv::Mat noDataMask = cv::Mat::zeros(iNormalMap.size(), CV_8UC1);
    cv::inRange(iNormalMap, cv::Scalar(0, 0, 0), cv::Scalar(1), noDataMask);

    cv::Mat normalMapCopy;
    iNormalMap.copyTo(normalMapCopy);
    if (iCvt2OpenglConvention)
        cv::multiply(normalMapCopy, cv::Scalar(1.f, -1.f, -1.f), normalMapCopy);

    cv::Mat colorizedNormalMap = normalMapCopy * 0.5 + 0.5;
    colorizedNormalMap.setTo(0, noDataMask);

    colorizedNormalMap.convertTo(colorizedNormalMap, CV_8UC3, 255.f);
    cv::cvtColor(colorizedNormalMap, colorizedNormalMap, cv::COLOR_RGB2BGR);

    return colorizedNormalMap;
}

} // namespace lib3d

#endif // LIB3D_VIS_RANGE_IMG_H
