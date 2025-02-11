/// @file FeatureDetector.cpp
/// @brief Implements the FeatureDetector class for detecting keypoints.

#include "FeatureDetector.h"

/**
 * @brief Constructor initializing the FAST feature detector parameters.
 * @param fastThreshold Threshold for the FAST detector.
 * @param nonmaxSuppression Whether to apply non-maximal suppression.
 */
FeatureDetector::FeatureDetector(int fastThreshold, bool nonmaxSuppression)
    : fastThreshold(fastThreshold), nonmaxSuppression(nonmaxSuppression) {}

/**
 * @brief Detects features in an input image.
 * @param img Input image.
 * @param points Output vector of detected feature points.
 */
void FeatureDetector::detectFeatures(const cv::Mat &img, std::vector<cv::Point2f> &points)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(img, keypoints, fastThreshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points);
}
