/// @file FeatureDetector.h
/// @brief Defines the FeatureDetector class for detecting keypoints in images.

#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @class FeatureDetector
 * @brief Detects keypoints in images using the FAST algorithm.
 */
class FeatureDetector
{
public:
    /**
     * @brief Constructor initializing the FAST feature detector parameters.
     * @param fastThreshold Intensity threshold for FAST detector.
     * @param nonmaxSuppression Whether to apply non-maximal suppression.
     */
    FeatureDetector(int fastThreshold = 20, bool nonmaxSuppression = true);

    /**
     * @brief Detects keypoints in an image.
     * @param img Input image.
     * @param points Output vector of detected feature points.
     */
    void detectFeatures(const cv::Mat &img, std::vector<cv::Point2f> &points);

private:
    int fastThreshold;      ///< FAST detector threshold value.
    bool nonmaxSuppression; ///< Flag for non-maximal suppression.
};

#endif // FEATUREDETECTOR_H
