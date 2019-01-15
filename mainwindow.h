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

#include "opencv2/opencv.hpp"
#include <omp.h>

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
    void browseVideoClicked();
    void browseSettingClicked();

    void on_pushButton_clicked();
    void on_actionOpen_triggered();

    void on_actionGrid3x4_triggered();
    void on_actionGrid4x5_triggered();

private:
    Ui::MainWindow *ui;
    void dialogSetting();
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
    QString pathTo;
    QString   type;
    int  rows,cols;
    int squareSize;

    int n_centers;
};

#endif // MAINWINDOW_H
