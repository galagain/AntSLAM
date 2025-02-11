/// @file Visualizer3D.h
/// @brief Defines the Visualizer3D class for real-time 3D visualization of visual odometry (VO) data.

#ifndef VISUALIZER3D_H
#define VISUALIZER3D_H

#include <opencv2/opencv.hpp>
#include <GL/freeglut.h>
#include <vector>

/**
 * @class Visualizer3D
 * @brief Handles real-time 3D visualization of visual odometry (VO) data using OpenGL (via FreeGLUT).
 *
 * The visualizer displays:
 *   - A global axis for reference.
 *   - The ground-truth trajectory (if provided).
 *   - The predicted visual odometry (VO) trajectory.
 *
 * It also supports user interaction, allowing camera rotation (via mouse drag)
 * and zooming (via mouse wheel).
 */
class Visualizer3D
{
public:
    /**
     * @brief Constructs the Visualizer3D object.
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
     * @brief Initializes OpenGL settings such as background color, depth testing, and lighting.
     */
    void initGL();

    /**
     * @brief Registers all required GLUT callbacks for handling display, reshaping, and mouse input.
     *
     * This method should be called after the GLUT window is created (e.g., `glutCreateWindow`).
     */
    void registerCallbacks();

private:
    // ------------------ Static callbacks required by GLUT ------------------
    // GLUT requires static or free functions, so we store a static pointer to the active instance.

    /**
     * @brief Static display callback forwarding to the instance's displayCallback().
     */
    static void s_displayCallback();

    /**
     * @brief Static reshape callback forwarding to the instance's reshapeCallback().
     * @param w New window width.
     * @param h New window height.
     */
    static void s_reshapeCallback(int w, int h);

    /**
     * @brief Static mouse button callback forwarding to the instance's mouseCallback().
     * @param button Mouse button (e.g., left, right, scroll).
     * @param state Button state (pressed/released).
     * @param x Mouse cursor x-coordinate.
     * @param y Mouse cursor y-coordinate.
     */
    static void s_mouseCallback(int button, int state, int x, int y);

    /**
     * @brief Static mouse motion callback forwarding to the instance's motionCallback().
     * @param x Mouse cursor x-coordinate during movement.
     * @param y Mouse cursor y-coordinate during movement.
     */
    static void s_motionCallback(int x, int y);

    // ------------------ Instance methods ------------------

    /**
     * @brief Main rendering function. Draws axes, trajectories, and updates the viewport.
     */
    void displayCallback();

    /**
     * @brief Adjusts the viewport and projection settings when the window is resized.
     * @param w New window width.
     * @param h New window height.
     */
    void reshapeCallback(int w, int h);

    /**
     * @brief Handles mouse button press events (including zoom via scroll wheel).
     * @param button Mouse button identifier.
     * @param state Pressed or released state.
     * @param x Cursor x-coordinate at the moment of the event.
     * @param y Cursor y-coordinate at the moment of the event.
     */
    void mouseCallback(int button, int state, int x, int y);

    /**
     * @brief Handles mouse movement when a button is pressed, used for rotating the view.
     * @param x Current mouse x-coordinate.
     * @param y Current mouse y-coordinate.
     */
    void motionCallback(int x, int y);

    /**
     * @brief Draws a simple 3D coordinate axis system (X = red, Y = green, Z = blue).
     * @param length Length of each axis line.
     */
    void drawAxis(float length = 1.0f);

    /**
     * @brief Draws a trajectory as a connected series of points in 3D space.
     * @param traj The trajectory points to be drawn.
     * @param r Red color component (0-1).
     * @param g Green color component (0-1).
     * @param b Blue color component (0-1).
     */
    void drawTrajectory(const std::vector<cv::Point3f> &traj, float r, float g, float b);

private:
    /// A static pointer to the currently active Visualizer3D instance, required for GLUT callbacks.
    static Visualizer3D *s_instance;

    /// Pointer to the rotation matrix (3x3) representing the camera's current orientation.
    cv::Mat *m_R_f;

    /// Pointer to the translation vector (3x1) representing the camera's current position.
    cv::Mat *m_t_f;

    /// Pointer to the ground-truth 3D trajectory (if available).
    std::vector<cv::Point3f> *m_groundTruth;

    /// Pointer to the predicted 3D trajectory.
    std::vector<cv::Point3f> *m_predicted;

    // Camera interaction parameters

    float m_camAngleX = 20.f;   ///< Current rotation angle around the X-axis (degrees).
    float m_camAngleY = 30.f;   ///< Current rotation angle around the Y-axis (degrees).
    float m_camDistance = 50.f; ///< Distance from the origin (zoom level).

    bool m_leftButtonDown = false; ///< True if the left mouse button is currently pressed.
    int m_lastMouseX = 0;          ///< Last recorded X position of the mouse (for drag rotation).
    int m_lastMouseY = 0;          ///< Last recorded Y position of the mouse (for drag rotation).
};

#endif // VISUALIZER3D_H
