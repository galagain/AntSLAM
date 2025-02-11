#include "SlamSystem.h"
#include "Metrics.h"

int main(int argc, char **argv)
{
    // The user can set their data paths
    std::string imagesPath = "/media/calvin/Extreme_Pro9/odometry/KITTI/05/image_1";
    std::string posesPath = "/media/calvin/Extreme_Pro9/odometry/kitti_poses/poses/05.txt";
    std::string savePosesPath = "poses2.txt";

    // Create the slam system
    SlamSystem slam(imagesPath, posesPath, true, savePosesPath);

    // Run the pipeline, passing arguments for GLUT
    slam.run(argc, argv);

    return 0;
}