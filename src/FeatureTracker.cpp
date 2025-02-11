/// @file FeatureTracker.cpp
/// @brief Implements the FeatureTracker class for tracking keypoints.

#include "FeatureTracker.h"

/**
 * @brief Default constructor.
 */
FeatureTracker::FeatureTracker() {}

/**
 * @brief Tracks features between two consecutive images.
 * @param img1 First input image.
 * @param img2 Second input image.
 * @param points1 Feature points detected in img1.
 * @param points2 Corresponding tracked feature points in img2.
 * @param status Status vector indicating success of tracking.
 */
void FeatureTracker::trackFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                   std::vector<cv::Point2f> &points1,
                                   std::vector<cv::Point2f> &points2,
                                   std::vector<uchar> &status) const
{
    std::vector<float> err;
    cv::Size winSize(21, 21);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
    removeInvalidPoints(points1, points2, status);
}

/**
 * @brief Removes invalid points that failed tracking or moved out of the frame.
 * @param points1 Feature points in the first image.
 * @param points2 Corresponding points in the second image.
 * @param status Status vector for tracking results.
 */
void FeatureTracker::removeInvalidPoints(std::vector<cv::Point2f> &points1,
                                         std::vector<cv::Point2f> &points2,
                                         std::vector<uchar> &status) const
{
    int indexCorrection = 0;
    for (size_t i = 0; i < status.size(); i++)
    {
        cv::Point2f pt = points2.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
        {
            status.at(i) = 0;
            points1.erase(points1.begin() + i - indexCorrection);
            points2.erase(points2.begin() + i - indexCorrection);
            indexCorrection++;
        }
    }
}
