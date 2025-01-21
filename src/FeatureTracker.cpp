#include "FeatureTracker.h"

FeatureTracker::FeatureTracker() {}

void FeatureTracker::trackFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                   std::vector<cv::Point2f> &points1,
                                   std::vector<cv::Point2f> &points2,
                                   std::vector<uchar> &status) const
{
    std::vector<float> err;
    cv::Size winSize(21, 21);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // Compute optical flow using Lucas-Kanade method
    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    // Remove invalid or out-of-frame points
    removeInvalidPoints(points1, points2, status);
}

void FeatureTracker::removeInvalidPoints(std::vector<cv::Point2f> &points1,
                                         std::vector<cv::Point2f> &points2,
                                         std::vector<uchar> &status) const
{
    // Iterate through points and remove those that failed tracking or moved out of frame
    int indexCorrection = 0;
    for (size_t i = 0; i < status.size(); i++)
    {
        cv::Point2f pt = points2.at(i - indexCorrection);

        // Check if tracking failed or if the point is out of bounds
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
        {
            if ((pt.x < 0) || (pt.y < 0))
            {
                status.at(i) = 0; // Mark the point as invalid
            }

            // Remove invalid points from both lists
            points1.erase(points1.begin() + i - indexCorrection);
            points2.erase(points2.begin() + i - indexCorrection);
            indexCorrection++;
        }
    }
}
