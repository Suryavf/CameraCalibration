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

#include <thread>
#include <chrono>
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
    QString   pathTo;
    QString     type;
    int width,height;
    int   squareSize;
    int          fps;

    bool            mode; // Test: 0, Calibration: 1
    bool         capture; // Capture points
    bool calibrationMode; // Manual: 0, Automatic: 1

    int n_centers;

    void drawWindows(cv::Mat &sec1, cv::Mat &sec2,cv::Mat &sec3, cv::Mat &sec4, cv::Mat &_main);
    void testRoutine();
    void calibrationRoutine();
    void frameSelection();
};

#endif // MAINWINDOW_H
