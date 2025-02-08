#ifndef VISUALIZER3D_H
#define VISUALIZER3D_H

#include <opencv2/opencv.hpp>
#include <GL/freeglut.h>
#include <vector>

/**
 * @class Visualizer3D
 *
 * @brief A class that handles real-time 3D visualization of VO (Visual Odometry) data
 *        using OpenGL (via FreeGLUT).
 *
 * It displays:
 *   - A global axis,
 *   - The ground-truth trajectory (if provided),
 *   - The predicted (VO) trajectory.
 *
 * It also supports user interaction for camera rotation (via mouse drag)
 * and zoom (mouse wheel).
 */
class Visualizer3D
{
public:
    /**
     * @brief Constructs the Visualizer3D with pointers to rotation, translation, and two trajectories.
     *
     * @param R_f         Pointer to a CV_64F 3x3 matrix representing the camera rotation.
     * @param t_f         Pointer to a CV_64F 3x1 matrix representing the camera translation.
     * @param groundTruth Pointer to the ground-truth 3D trajectory (vector of cv::Point3f).
     * @param predicted   Pointer to the predicted 3D trajectory (vector of cv::Point3f).
     */
    Visualizer3D(cv::Mat *R_f,
                 cv::Mat *t_f,
                 std::vector<cv::Point3f> *groundTruth,
                 std::vector<cv::Point3f> *predicted);

    /**
     * @brief Performs basic OpenGL initialization, such as setting the clear color and enabling depth testing.
     */
    void initGL();

    /**
     * @brief Registers all the needed GLUT callbacks for displaying, reshaping, and handling mouse input.
     *
     * This must be called after creating the GLUT window (e.g., glutCreateWindow).
     */
    void registerCallbacks();

private:
    // ------------------ Static callbacks required by GLUT ------------------
    // GLUT must call static or free functions, so we store a static pointer to the instance.

    /**
     * @brief Static display callback that forwards to the instance's displayCallback().
     */
    static void s_displayCallback();

    /**
     * @brief Static reshape callback that forwards to the instance's reshapeCallback().
     */
    static void s_reshapeCallback(int w, int h);

    /**
     * @brief Static mouse callback that forwards to the instance's mouseCallback().
     */
    static void s_mouseCallback(int button, int state, int x, int y);

    /**
     * @brief Static motion callback that forwards to the instance's motionCallback().
     */
    static void s_motionCallback(int x, int y);

    // ------------------ Actual instance methods ------------------

    /**
     * @brief The main display routine, drawing axes and trajectories.
     */
    void displayCallback();

    /**
     * @brief Handles window resize events to adjust the viewport and projection.
     */
    void reshapeCallback(int w, int h);

    /**
     * @brief Handles mouse button presses, including scroll wheel.
     */
    void mouseCallback(int button, int state, int x, int y);

    /**
     * @brief Handles mouse movement while a button is pressed (drag).
     */
    void motionCallback(int x, int y);

    /**
     * @brief Draws a simple 3D axis (X = red, Y = green, Z = blue).
     */
    void drawAxis(float length = 1.0f);

    /**
     * @brief Draws a 3D trajectory as a connected line plus points.
     */
    void drawTrajectory(const std::vector<cv::Point3f> &traj, float r, float g, float b);

private:
    /// A static pointer referencing the active Visualizer3D instance.
    static Visualizer3D *s_instance;

    /// Pointer to rotation matrix (3x3).
    cv::Mat *m_R_f;
    /// Pointer to translation vector (3x1).
    cv::Mat *m_t_f;
    /// Pointer to ground-truth trajectory vector.
    std::vector<cv::Point3f> *m_groundTruth;
    /// Pointer to predicted trajectory vector.
    std::vector<cv::Point3f> *m_predicted;

    // Camera interaction parameters
    float m_camAngleX = 20.f;   ///< Current rotation around X-axis (degrees)
    float m_camAngleY = 30.f;   ///< Current rotation around Y-axis (degrees)
    float m_camDistance = 50.f; ///< Distance from origin (zoom factor)

    bool m_leftButtonDown = false; ///< True if left mouse button is currently pressed.
    int m_lastMouseX = 0;          ///< Last mouse X position (for dragging).
    int m_lastMouseY = 0;          ///< Last mouse Y position (for dragging).
};

#endif // VISUALIZER3D_H
