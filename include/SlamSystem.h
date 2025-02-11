#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>

#include "DataLoader.h"
#include "FeatureDetector.h"
#include "FeatureTracker.h"
#include "MotionEstimator.h"
#include "Visualization.h"
#include "Visualizer3D.h"
#include "UtilsTransform.h"

/**
 * @class SlamSystem
 * @brief A class encapsulating the VO/SLAM pipeline, with optional ground-truth usage
 *        and optional visualization.
 */
class SlamSystem
{
public:
    /**
     * @brief Constructor with optional ground truth path and visualization toggle.
     *
     * @param datasetImagesPath Path to the folder containing images.
     * @param datasetPosesPath  Optional path to ground truth file. If empty, no GT used.
     * @param enableVisualization Whether to show the 2D window (OpenCV) and 3D window (OpenGL).
     *                           If false, runs headless: no UI is created.
     * @param savePosesPath Whether to save the poses to a text file.
     */
    SlamSystem(const std::string &datasetImagesPath,
               const std::string &datasetPosesPath = "",
               bool enableVisualization = true,
               const std::string savePosesPath = "");

    /**
     * @brief run: the main function that initializes the pipeline and processes frames.
     *        If visualization is enabled, will create the 3D window and show 2D images,
     *        otherwise runs headless.
     *
     * @param argc, argv Command line arguments to pass to GLUT if needed.
     */
    void run(int argc, char **argv);

    /**
     * @brief savePoseToFile: save the current pose to a text file.
     *
     * @param R, t Rotation and translation matrices to save.
     */
    void savePoseToFile(const cv::Mat &R, const cv::Mat &t);

private:
    // -----------------------------------------
    // Helper methods
    // -----------------------------------------
    void processOneFrame();
    std::vector<double> loadAbsoluteScales(const std::vector<cv::Point3f> &gt);

private:
    // -----------------------------------------
    // Data paths
    // -----------------------------------------
    std::string dataset_images_location_;
    std::string dataset_poses_location_;

    std::string save_poses_path_;

    // -----------------------------------------
    // Visualization toggle
    // -----------------------------------------
    bool enableVisualization_; ///< If false, skip all 2D/3D displays.

    // -----------------------------------------
    // Pipeline components
    // -----------------------------------------
    DataLoader dataLoader_;
    FeatureDetector featureDetector_;
    FeatureTracker featureTracker_;
    MotionEstimator motionEstimator_;
    Visualizer visualizer_; ///< For 2D (OpenCV) display, used only if enableVisualization_ = true

    // -----------------------------------------
    // Image lists and counters
    // -----------------------------------------
    std::vector<std::string> images_;
    int numFrame_ = 0;
    int totalFrames_ = 0;

    // -----------------------------------------
    // Global VO pose
    // -----------------------------------------
    cv::Mat R_f_, t_f_;

    // -----------------------------------------
    // Feature tracking
    // -----------------------------------------
    cv::Mat prevImage_;
    cv::Mat currImage_;
    std::vector<cv::Point2f> prevFeatures_;
    std::vector<cv::Point2f> currFeatures_;
    std::vector<uchar> status_;

    // -----------------------------------------
    // Ground truth: full vs. incremental
    // If no ground truth is provided, these remain empty
    // and we skip all scale logic.
    // -----------------------------------------
    bool hasGroundTruth_ = false; ///< Tells us if we have a valid GT file
    std::vector<cv::Point3f> groundTruthAll_;
    std::vector<cv::Point3f> groundTruth3D_;
    std::vector<double> groundScales_;

    // -----------------------------------------
    // Predicted 3D (VO)
    // -----------------------------------------
    std::vector<cv::Point3f> predicted3D_;

    // -----------------------------------------
    // 3D Visualizer
    // -----------------------------------------
    Visualizer3D *visualizer3D_;

    // -----------------------------------------
    // Constants
    // -----------------------------------------
    static constexpr int MAX_FRAME = 2000;
    static constexpr int MIN_NUM_FEAT = 2000;
};

#endif // SLAM_SYSTEM_H
