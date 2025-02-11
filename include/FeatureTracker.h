/// @file FeatureTracker.h
/// @brief Defines the FeatureTracker class for tracking keypoints across frames.

#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @class FeatureTracker
 * @brief Tracks features between consecutive images.
 */
class FeatureTracker
{
public:
    /// @brief Default constructor.
    FeatureTracker();

    /**
     * @brief Tracks features between two images.
     * @param img1 First input image.
     * @param img2 Second input image.
     * @param points1 Feature points in the first image.
     * @param points2 Corresponding feature points in the second image.
     * @param status Output status vector indicating successful tracking.
     */
    void trackFeatures(const cv::Mat &img1, const cv::Mat &img2,
                       std::vector<cv::Point2f> &points1,
                       std::vector<cv::Point2f> &points2,
                       std::vector<uchar> &status) const;

private:
    /**
     * @brief Removes poorly tracked or out-of-frame points.
     * @param points1 First set of points.
     * @param points2 Corresponding points in the second image.
     * @param status Status vector of tracking results.
     */
    void removeInvalidPoints(std::vector<cv::Point2f> &points1,
                             std::vector<cv::Point2f> &points2,
                             std::vector<uchar> &status) const;
};

#endif // FEATURETRACKER_H
