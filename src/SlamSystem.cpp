/// @file SlamSystem.cpp
/// @brief Implements the SlamSystem class for handling the SLAM pipeline.

#include "SlamSystem.h"

/**
 * @brief Constructs the SLAM system.
 *
 * Initializes dataset paths, visualization settings, and system components such as
 * feature detectors, motion estimators, and visualizers.
 *
 * @param datasetImagesPath Path to the folder containing input images.
 * @param datasetPosesPath  Optional path to ground truth file. If empty, no GT is used.
 * @param enableVisualization If true, enables 2D and 3D visualization.
 * @param savePosesPath Path where estimated poses will be saved (optional).
 */
SlamSystem::SlamSystem(const std::string &datasetImagesPath,
                       const std::string &datasetPosesPath,
                       bool enableVisualization,
                       const std::string savePosesPath)
    : dataset_images_location_(datasetImagesPath),
      dataset_poses_location_(datasetPosesPath),
      enableVisualization_(enableVisualization),
      save_poses_path_(savePosesPath),
      motionEstimator_(718.8560, cv::Point2d(607.1928, 185.2157)),
      visualizer_(),
      hasGroundTruth_(false),
      visualizer3D_(nullptr)
{
    // If datasetPosesPath is provided, ground truth may be loaded in run().
    // If empty, system runs without ground truth.
}

/**
 * @brief Main function that initializes the SLAM pipeline and processes frames.
 *
 * Loads images, initializes visualization (if enabled), and starts the main loop.
 *
 * @param argc Command-line argument count (for GLUT).
 * @param argv Command-line argument vector.
 */
void SlamSystem::run(int argc, char **argv)
{
    // 1) Load images
    images_ = dataLoader_.getImages(dataset_images_location_);
    totalFrames_ = (int)images_.size();
    if (totalFrames_ < 2)
    {
        std::cerr << "Not enough images found in " << dataset_images_location_ << std::endl;
        return;
    }

    // 2) Attempt ground-truth load if not empty
    if (!dataset_poses_location_.empty())
    {
        groundTruthAll_ = dataLoader_.loadGroundPoses3D(dataset_poses_location_);
        if (!groundTruthAll_.empty())
        {
            hasGroundTruth_ = true;
            groundScales_ = loadAbsoluteScales(groundTruthAll_);
            groundTruth3D_.clear();
            std::cout << "GT loaded successfully. Number of poses: " << groundTruthAll_.size() << std::endl;
        }
        else
        {
            std::cerr << "[Warning] Could not load ground truth. Running without GT.\n";
            hasGroundTruth_ = false;
        }
    }
    else
    {
        hasGroundTruth_ = false;
    }

    // 3) Load first two images
    cv::Mat img1_c = dataLoader_.loadImage(images_[0]);
    cv::Mat img2_c = dataLoader_.loadImage(images_[1]);
    if (img1_c.empty() || img2_c.empty())
    {
        std::cerr << "Error reading the first two images.\n";
        return;
    }

    cv::Mat img1, img2;
    cv::cvtColor(img1_c, img1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2_c, img2, cv::COLOR_BGR2GRAY);

    // 4) Detect & track initial features
    featureDetector_.detectFeatures(img1, prevFeatures_);
    featureTracker_.trackFeatures(img1, img2, prevFeatures_, currFeatures_, status_);

    // 5) Estimate first motion
    {
        cv::Mat R, t;
        motionEstimator_.estimateMotion(prevFeatures_, currFeatures_, R, t);
        R_f_ = R.clone();
        t_f_ = t.clone();

        // Save the first pose
        if (save_poses_path_ != "")
        {
            savePoseToFile(R_f_, t_f_);
        }
    }

    // 6) Store the first predicted pose
    {
        float px = float(t_f_.at<double>(0));
        float py = float(t_f_.at<double>(1));
        float pz = float(t_f_.at<double>(2));
        predicted3D_.push_back(cv::Point3f(px, py, pz));
    }

    // 7) If GT is available, add the first GT pose
    if (hasGroundTruth_ && !groundTruthAll_.empty())
    {
        groundTruth3D_.push_back(groundTruthAll_[0]);
    }

    // 8) Prepare for main loop
    prevImage_ = img2.clone();
    prevFeatures_ = currFeatures_;
    numFrame_ = 2;

    // ---------------------------
    // Visualization Setup
    // ---------------------------
    if (enableVisualization_)
    {
        // Initialize GLUT for 3D, create window
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(800, 600);
        glutCreateWindow("3D Real-Time Visualization - optional");

        // Create a 3D viewer
        visualizer3D_ = new Visualizer3D(
            &R_f_, &t_f_,
            &groundTruth3D_, // might be empty if no GT
            &predicted3D_);
        visualizer3D_->initGL();
        visualizer3D_->registerCallbacks();
    }

    // Main loop
    clock_t startClock = clock();
    while (true)
    {
        // Limit frames with or without GT
        int limit = (hasGroundTruth_) ? std::min(totalFrames_, (int)groundTruthAll_.size()) : totalFrames_;

        if (numFrame_ >= limit || numFrame_ >= MAX_FRAME)
            break;

        processOneFrame();

        // Update windows if enabled
        if (enableVisualization_)
        {
            glutMainLoopEvent();
            glutPostRedisplay();

            // Update 2D window
            int key = cv::waitKey(1);
            if (key == 27) // ESC
                break;
        }
        else
        {
            // If no visualization, we can do a minimal wait or no wait
            // to prevent a tight CPU loop
            // e.g., std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    clock_t endClock = clock();
    double secs = double(endClock - startClock) / CLOCKS_PER_SEC;
    std::cout << "Total time: " << secs << " s" << std::endl;

    // Keep 3D window open (only if we created it)
    if (enableVisualization_ && visualizer3D_)
    {
        while (true)
        {
            glutMainLoopEvent();
            glutPostRedisplay();

            if (!glutGetWindow())
                break;

            int k = cv::waitKey(10);
            if (k == 27) // ESC
                break;
        }

        delete visualizer3D_;
        visualizer3D_ = nullptr;
    }
}

/**
 * @brief Processes a single frame in the SLAM pipeline.
 */
void SlamSystem::processOneFrame()
{
    if (numFrame_ >= totalFrames_)
        return;

    cv::Mat currImage_c = dataLoader_.loadImage(images_[numFrame_]);
    if (currImage_c.empty())
        return;

    cv::Mat currGray;
    cv::cvtColor(currImage_c, currGray, cv::COLOR_BGR2GRAY);

    // Track
    featureTracker_.trackFeatures(prevImage_, currGray,
                                  prevFeatures_, currFeatures_,
                                  status_);

    // Estimate motion
    cv::Mat R, t;
    motionEstimator_.estimateMotion(prevFeatures_, currFeatures_, R, t);

    // If GT => scale. Otherwise, raw monocular
    if (hasGroundTruth_ && numFrame_ < (int)groundScales_.size())
    {
        double scale = groundScales_[numFrame_];
        if ((scale > 0.1) &&
            (t.at<double>(2) > t.at<double>(0)) &&
            (t.at<double>(2) > t.at<double>(1)))
        {
            t_f_ = t_f_ + scale * (R_f_ * t);
            R_f_ = R * R_f_;
        }
    }
    else
    {
        // No GT => no scale
        t_f_ = t_f_ + (R_f_ * t);
        R_f_ = R * R_f_;
    }

    // Save the pose
    if (save_poses_path_ != "")
    {
        savePoseToFile(R_f_, t_f_);
    }

    // If too few features => detect new
    if (prevFeatures_.size() < MIN_NUM_FEAT)
    {
        featureDetector_.detectFeatures(prevImage_, prevFeatures_);
        featureTracker_.trackFeatures(prevImage_, currGray,
                                      prevFeatures_, currFeatures_,
                                      status_);
    }

    // Store predicted pose
    {
        float px = float(t_f_.at<double>(0));
        float py = float(t_f_.at<double>(1));
        float pz = float(t_f_.at<double>(2));
        predicted3D_.push_back(cv::Point3f(px, py, pz));
    }

    // If GT => add incremental
    if (hasGroundTruth_ && numFrame_ < (int)groundTruthAll_.size())
    {
        groundTruth3D_.push_back(groundTruthAll_[numFrame_]);
    }

    // Show 2D image only if visualization is enabled
    if (enableVisualization_)
    {
        visualizer_.showImage(currImage_c);
    }

    prevImage_ = currGray.clone();
    prevFeatures_ = currFeatures_;

    numFrame_++;
}

/**
 * @brief Computes the absolute scale between consecutive ground-truth positions.
 *
 * This function calculates the Euclidean distance between each pair of consecutive
 * ground-truth 3D positions and stores these values in a vector.
 *
 * @param gt Vector of ground-truth 3D positions (cv::Point3f).
 * @return Vector of computed scale values between consecutive positions.
 */
std::vector<double> SlamSystem::loadAbsoluteScales(const std::vector<cv::Point3f> &gt)
{
    std::vector<double> scales;

    // Return empty vector if no ground-truth data is provided
    if (gt.empty())
        return scales;

    // First scale value is 0 since there's no previous position
    scales.push_back(0.0);

    // Compute Euclidean distance between each pair of consecutive GT positions
    for (size_t i = 1; i < gt.size(); i++)
    {
        float dx = gt[i].x - gt[i - 1].x;
        float dy = gt[i].y - gt[i - 1].y;
        float dz = gt[i].z - gt[i - 1].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        scales.push_back(dist);
    }

    return scales;
}

/**
 * @brief Saves the estimated pose (rotation and translation) to a file.
 *
 * The pose is stored in the KITTI dataset format:
 *
 * ```
 * R11 R12 R13 t1 R21 R22 R23 t2 R31 R32 R33 t3
 * ```
 *
 * Each line represents a 3x4 transformation matrix consisting of a
 * 3x3 rotation matrix and a 3x1 translation vector.
 *
 * @param R Rotation matrix (3x3).
 * @param t Translation vector (3x1).
 */
void SlamSystem::savePoseToFile(const cv::Mat &R, const cv::Mat &t)
{
    // Open the file in append mode
    std::ofstream file(save_poses_path_, std::ios::app);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open the pose file." << std::endl;
        return;
    }

    // Ensure the matrices are continuous before accessing their elements
    cv::Mat R_copy = R;
    cv::Mat t_copy = t;
    if (!R_copy.isContinuous())
        R_copy = R_copy.clone();
    if (!t_copy.isContinuous())
        t_copy = t_copy.clone();

    // Write the rotation matrix and translation vector to file
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            file << R_copy.at<double>(i, j) << " ";

        file << t_copy.at<double>(i) << " ";
    }

    file << "\n";
    file.close();
}
