#ifndef DUALVISUALIZER_H
#define DUALVISUALIZER_H

#include <string>
#include <opencv2/opencv.hpp>

class DualVisualizer
{
public:
    /**
     * @brief Constructeur
     * @param imageWindowName Nom de la fenêtre pour l’image (2D)
     * @param poseWindowName  Nom de la fenêtre pour la pose (3D)
     *
     * Les fenêtres ne sont créées qu’au premier appel de showImage ou showPose.
     */
    DualVisualizer(const std::string &imageWindowName = "Image Window",
                   const std::string &poseWindowName = "Pose Window");

    /**
     * @brief Affiche l'image 2D dans la fenêtre correspondante.
     *        La fenêtre est créée automatiquement avec la taille de la première image reçue.
     * @param image Image à afficher
     */
    void showImage(const cv::Mat &image);

    /**
     * @brief Affiche une “image” (ou placeholder) pour la pose 3D.
     *        La fenêtre est créée automatiquement avec la taille de la première “image” reçue.
     *
     * Note: Dans une implémentation future, on pourra utiliser la bibliothèque
     *       OpenCV Viz (ou une autre) pour l'affichage 3D. Ici, on se contente
     *       d'un Mat, par exemple une visualisation 2D de la pose.
     *
     * @param poseViz Image (ou placeholder) représentant la pose 3D
     */
    void showPose(const cv::Mat &poseViz);

private:
    // Noms de fenêtres
    std::string imageWindowName_;
    std::string poseWindowName_;

    // Pour savoir si les fenêtres ont déjà été créées
    bool imageWindowCreated_;
    bool poseWindowCreated_;
};

#endif // DUALVISUALIZER_H
#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/opencv.hpp>

class Visualizer
{
public:
    // The window is named "Image Visualizer" and is not created until showImage() is called
    Visualizer();

    // Displays the image in the "Image Visualizer" window
    void showImage(const cv::Mat &image);

private:
    bool isWindowCreated_;
};

#endif // VISUALIZATION_H
