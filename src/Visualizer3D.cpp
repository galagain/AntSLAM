/// @file Visualizer3D.cpp
/// @brief Implements the Visualizer3D class for real-time 3D visualization.

#include "Visualizer3D.h"
#include <iostream>
#include <cmath>

// A static pointer to hold the singleton-like instance required for GLUT callbacks.
Visualizer3D *Visualizer3D::s_instance = nullptr;

/**
 * @brief Constructs a Visualizer3D object with references to rotation, translation,
 *        and two 3D trajectories (ground truth and predicted).
 *
 * @param R_f         Pointer to the camera rotation matrix (3x3).
 * @param t_f         Pointer to the camera translation vector (3x1).
 * @param groundTruth Pointer to the ground-truth 3D trajectory.
 * @param predicted   Pointer to the predicted 3D trajectory (VO result).
 */
Visualizer3D::Visualizer3D(cv::Mat *R_f,
                           cv::Mat *t_f,
                           std::vector<cv::Point3f> *groundTruth,
                           std::vector<cv::Point3f> *predicted)
    : m_R_f(R_f), m_t_f(t_f), m_groundTruth(groundTruth), m_predicted(predicted)
{
}

/**
 * @brief Initializes basic OpenGL settings such as background color and depth testing.
 */
void Visualizer3D::initGL()
{
    glClearColor(0.f, 0.f, 0.f, 1.f); // Set background color to black.
    glEnable(GL_DEPTH_TEST);          // Enable depth testing for correct 3D rendering.
}

/**
 * @brief Registers all GLUT callbacks (display, reshape, mouse, motion).
 */
void Visualizer3D::registerCallbacks()
{
    s_instance = this;
    glutDisplayFunc(s_displayCallback);
    glutReshapeFunc(s_reshapeCallback);
    glutMouseFunc(s_mouseCallback);
    glutMotionFunc(s_motionCallback);
}

//------------------------------------------------------------------------------
// Static callback methods forwarding to instance methods
//------------------------------------------------------------------------------

void Visualizer3D::s_displayCallback()
{
    if (s_instance)
        s_instance->displayCallback();
}

void Visualizer3D::s_reshapeCallback(int w, int h)
{
    if (s_instance)
        s_instance->reshapeCallback(w, h);
}

void Visualizer3D::s_mouseCallback(int button, int state, int x, int y)
{
    if (s_instance)
        s_instance->mouseCallback(button, state, x, y);
}

void Visualizer3D::s_motionCallback(int x, int y)
{
    if (s_instance)
        s_instance->motionCallback(x, y);
}

/**
 * @brief Main display callback function for rendering the 3D scene.
 *
 * This function:
 * 1) Clears the screen,
 * 2) Applies transformations (zoom, user rotation, VO rotation/translation),
 * 3) Draws axes and trajectories (ground truth in green, predicted in red),
 * 4) Swaps buffers to display the updated scene.
 */
void Visualizer3D::displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Apply zoom based on camera distance
    glTranslatef(0.f, 0.f, -m_camDistance);

    // Apply user-controlled rotation (via mouse dragging)
    glRotatef(m_camAngleX, 1.f, 0.f, 0.f);
    glRotatef(m_camAngleY, 0.f, 1.f, 0.f);

    // Apply VO rotation (transpose required for OpenGL transformation)
    if (m_R_f && !m_R_f->empty())
    {
        cv::Mat Rvo = m_R_f->t();
        GLfloat glMat[16] =
            {
                (GLfloat)Rvo.at<double>(0, 0), (GLfloat)Rvo.at<double>(1, 0), (GLfloat)Rvo.at<double>(2, 0), 0.f,
                (GLfloat)Rvo.at<double>(0, 1), (GLfloat)Rvo.at<double>(1, 1), (GLfloat)Rvo.at<double>(2, 1), 0.f,
                (GLfloat)Rvo.at<double>(0, 2), (GLfloat)Rvo.at<double>(1, 2), (GLfloat)Rvo.at<double>(2, 2), 0.f,
                0.f, 0.f, 0.f, 1.f};
        glMultMatrixf(glMat);
    }

    // Apply VO translation (negative for camera-space transform)
    if (m_t_f && !m_t_f->empty())
    {
        float px = (float)m_t_f->at<double>(0);
        float py = (float)m_t_f->at<double>(1);
        float pz = (float)m_t_f->at<double>(2);
        glTranslatef(-px, -py, -pz);
    }

    // Draw reference axis and trajectories
    drawAxis(5.f);
    if (m_groundTruth)
        drawTrajectory(*m_groundTruth, 0.f, 1.f, 0.f);
    if (m_predicted)
        drawTrajectory(*m_predicted, 1.f, 0.f, 0.f);

    glutSwapBuffers();
}

/**
 * @brief Handles window resizing events to adjust the OpenGL viewport.
 * @param w New window width.
 * @param h New window height.
 */
void Visualizer3D::reshapeCallback(int w, int h)
{
    if (h == 0)
        h = 1;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / (double)h, 0.1, 100000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/**
 * @brief Handles mouse button interactions (clicks and scroll wheel).
 */
void Visualizer3D::mouseCallback(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            m_leftButtonDown = true;
            m_lastMouseX = x;
            m_lastMouseY = y;
        }
        else
        {
            m_leftButtonDown = false;
        }
    }
    else if (button == 3) // Scroll up => zoom in
    {
        m_camDistance = std::max(1.f, m_camDistance - 1.0f);
    }
    else if (button == 4) // Scroll down => zoom out
    {
        m_camDistance = std::min(10000.f, m_camDistance + 1.0f);
    }

    glutPostRedisplay();
}

/**
 * @brief Handles mouse motion when a button is pressed (for rotating the view).
 */
void Visualizer3D::motionCallback(int x, int y)
{
    if (m_leftButtonDown)
    {
        int dx = x - m_lastMouseX;
        int dy = y - m_lastMouseY;
        m_camAngleY += dx * 0.5f;
        m_camAngleX += dy * 0.5f;
        m_lastMouseX = x;
        m_lastMouseY = y;
        glutPostRedisplay();
    }
}

/**
 * @brief Draws a simple 3D coordinate axis system.
 * @param length Length of each axis.
 */
void Visualizer3D::drawAxis(float length)
{
    glBegin(GL_LINES);

    // X axis (Red)
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(length, 0.f, 0.f);

    // Y axis (Green)
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, length, 0.f);

    // Z axis (Blue)
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, length);

    glEnd();
}

/**
 * @brief Draws a trajectory in 3D space.
 * @param traj The trajectory points.
 * @param r Red color component.
 * @param g Green color component.
 * @param b Blue color component.
 */
void Visualizer3D::drawTrajectory(const std::vector<cv::Point3f> &traj,
                                  float r, float g, float b)
{
    if (traj.size() < 2)
        return;

    glColor3f(r, g, b);
    glLineWidth(2.f);

    glBegin(GL_LINE_STRIP);
    for (const auto &p : traj)
        glVertex3f(p.x, p.y, p.z);
    glEnd();

    glPointSize(4.f);
    glBegin(GL_POINTS);
    for (const auto &p : traj)
        glVertex3f(p.x, p.y, p.z);
    glEnd();
}
