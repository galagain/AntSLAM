#include "Visualizer3D.h"
#include <iostream>
#include <cmath>

// A static pointer to hold the singleton-like instance that GLUT callbacks require.
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
 * @brief Initializes basic OpenGL state, such as background color and depth testing.
 */
void Visualizer3D::initGL()
{
    glClearColor(0.f, 0.f, 0.f, 1.f); // Set the clear (background) color to black.
    glEnable(GL_DEPTH_TEST);          // Enable depth testing for 3D rendering.
}

/**
 * @brief Registers all the static callbacks (display, reshape, mouse, motion)
 *        so that this class can handle them internally.
 *
 * We store 'this' in a static pointer so that GLUT’s C-style callbacks can
 * forward calls to our instance methods.
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
// Static callback methods that forward to instance methods
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
 * @brief The main display callback for drawing the 3D scene.
 *
 * This method:
 * 1) Clears the screen,
 * 2) Applies transformations (zoom, mouse rotation, VO rotation/translation),
 * 3) Draws axes and trajectories (ground truth in green, predicted in red),
 * 4) Swaps buffers to display the updated scene.
 */
void Visualizer3D::displayCallback()
{
    // Clear the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // 1) Zoom out or in, based on current camera distance
    glTranslatef(0.f, 0.f, -m_camDistance);

    // 2) Apply user-controlled rotation (via mouse dragging)
    glRotatef(m_camAngleX, 1.f, 0.f, 0.f);
    glRotatef(m_camAngleY, 0.f, 1.f, 0.f);

    // 3) Apply VO rotation (we transpose m_R_f to transform properly)
    if (m_R_f && !m_R_f->empty())
    {
        cv::Mat Rvo = m_R_f->t(); // Transpose the 3x3 rotation from VO
        GLfloat glMat[16] =
            {
                (GLfloat)Rvo.at<double>(0, 0), (GLfloat)Rvo.at<double>(1, 0), (GLfloat)Rvo.at<double>(2, 0), 0.f,
                (GLfloat)Rvo.at<double>(0, 1), (GLfloat)Rvo.at<double>(1, 1), (GLfloat)Rvo.at<double>(2, 1), 0.f,
                (GLfloat)Rvo.at<double>(0, 2), (GLfloat)Rvo.at<double>(1, 2), (GLfloat)Rvo.at<double>(2, 2), 0.f,
                0.f, 0.f, 0.f, 1.f};
        glMultMatrixf(glMat);
    }

    // 4) Apply VO translation (negative for camera-space transform)
    if (m_t_f && !m_t_f->empty())
    {
        float px = (float)m_t_f->at<double>(0);
        float py = (float)m_t_f->at<double>(1);
        float pz = (float)m_t_f->at<double>(2);
        glTranslatef(-px, -py, -pz);
    }

    // Draw a reference axis at the origin
    drawAxis(5.f);

    // Draw ground-truth trajectory in green
    if (m_groundTruth)
        drawTrajectory(*m_groundTruth, 0.f, 1.f, 0.f);

    // Draw predicted trajectory in red
    if (m_predicted)
        drawTrajectory(*m_predicted, 1.f, 0.f, 0.f);

    // Display the result
    glutSwapBuffers();
}

/**
 * @brief Called when the window is resized; sets the new viewport and aspect ratio.
 *
 * @param w The new window width.
 * @param h The new window height.
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
 * @brief Handles mouse button events (left click or scroll wheel).
 *
 * This lets us detect:
 *  - Left button pressed for camera rotation.
 *  - Scroll wheel for zooming in/out.
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
    else if (button == 3) // Wheel up => zoom in
    {
        m_camDistance -= 1.0f;
        if (m_camDistance < 1.f)
            m_camDistance = 1.f;
    }
    else if (button == 4) // Wheel down => zoom out
    {
        m_camDistance += 1.0f;
        if (m_camDistance > 10000.f)
            m_camDistance = 10000.f;
    }

    glutPostRedisplay();
}

/**
 * @brief Handles mouse motion while a button is pressed.
 *
 * Rotates the camera in response to mouse drag on the X or Y axis,
 * changing m_camAngleX / m_camAngleY accordingly.
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
 * @brief Draws a simple axis in the 3D scene: X = red, Y = green, Z = blue.
 *
 * @param length The length of each axis arm.
 */
void Visualizer3D::drawAxis(float length)
{
    glBegin(GL_LINES);

    // X axis in red
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(length, 0.f, 0.f);

    // Y axis in green
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, length, 0.f);

    // Z axis in blue
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, length);

    glEnd();
}

/**
 * @brief Draws a trajectory in 3D space, represented as a line strip plus points.
 *
 * @param traj The vector of 3D points to be drawn as a continuous path.
 * @param r    Red component [0..1].
 * @param g    Green component [0..1].
 * @param b    Blue component [0..1].
 */
void Visualizer3D::drawTrajectory(const std::vector<cv::Point3f> &traj,
                                  float r, float g, float b)
{
    if (traj.size() < 2)
        return;

    // Set color and line width
    glColor3f(r, g, b);
    glLineWidth(2.f);

    // Draw a connected line through all points
    glBegin(GL_LINE_STRIP);
    for (auto &p : traj)
        glVertex3f(p.x, p.y, p.z);
    glEnd();

    // Draw points as small squares
    glPointSize(4.f);
    glBegin(GL_POINTS);
    for (auto &p : traj)
        glVertex3f(p.x, p.y, p.z);
    glEnd();
}
