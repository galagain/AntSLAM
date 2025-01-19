#include "FeatureDetector.h"

FeatureDetector::FeatureDetector(int fastThreshold, bool nonmaxSuppression)
    : fastThreshold(fastThreshold), nonmaxSuppression(nonmaxSuppression) {}

void FeatureDetector::detectFeatures(const cv::Mat &img, std::vector<cv::Point2f> &points)
{
    std::vector<cv::KeyPoint> keypoints;

    // Detect keypoints using FAST algorithm
    cv::FAST(img, keypoints, fastThreshold, nonmaxSuppression);

    // Convert keypoints to Point2f format
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}
