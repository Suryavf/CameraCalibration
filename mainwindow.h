#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QMessageBox>
#include <QLabel>
#include <QElapsedTimer>

#include <QtCore>
#include <QtGui>
#include <QFileDialog>

#include <QDialogButtonBox>
#include <QComboBox>
#include <QVBoxLayout>
#include <QLineEdit>

#include <QGroupBox>

#include "setting.h"
#include "calibratedgrid.h"

#include <random>
#include <iostream>

#include <thread>
#include <chrono>
#include <math.h>
#include "opencv2/opencv.hpp"
#include <omp.h>

#define FRAMES_BY_CALIBRATE 30

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void on_actionOpen_triggered();
    void on_actionModeTest_triggered();
    void on_actionModeCalibration_triggered();

    void on_pushButtonCapture_clicked();
    void on_pushButtonCalibrationMode_clicked();

private:
    Ui::MainWindow *ui;
    // Graphics Pixmap
    QGraphicsPixmapItem PixOriginal  ;
    QGraphicsPixmapItem PixBinarized ;
    QGraphicsPixmapItem PixMorphology;
    QGraphicsPixmapItem PixEllipses  ;
    QGraphicsPixmapItem PixResult    ;

    setting *w;

    // CV::Video
    cv::VideoCapture video;

    // Parameters
    QString   pathTo;   // Path video file
    QString     type;   // Pattern type
    int width,height;   // Pattern dimension (width x height)
    int   squareSize;
    int          fps;   // Frame per second

    size_t gridSize;    // Tamaño de la malla (gridSize x gridSize)
                        // [Automatica Calibration]

    bool            mode; // Test: 0, Calibration: 1
    bool         capture; // Capture points
    bool calibrationMode; // Manual: 0, Automatic: 1

    int n_centers;  // Number of centers (gridSize * gridSize)

    void drawWindows(cv::Mat &sec1, cv::Mat &sec2,cv::Mat &sec3, cv::Mat &sec4, cv::Mat &_main);
    void testRoutine();
    void calibrationRoutine();
    float frameSelection(vector<Point2f> &pts, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize, double &totalArea);
};

float probabilityByAngle   (vector<Point2f> &pts);
float probabilityByArea    (vector<Point2f> &pts, double &totalArea);
float probabilityByPosition(vector<Point2f> &pts, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize);

void addPointsToCloudPoints(vector<Point2f> &pts, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize);

#endif // MAINWINDOW_H
