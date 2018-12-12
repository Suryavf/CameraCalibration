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

#include "opencv2/opencv.hpp"

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
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    //QGraphicsPixmapItem pixmap;

    // Graphics Pixmap
    QGraphicsPixmapItem PixOriginal  ;
    QGraphicsPixmapItem PixBinarized ;
    QGraphicsPixmapItem PixMorphology;
    QGraphicsPixmapItem PixEllipses  ;
    QGraphicsPixmapItem PixResult    ;

    // CV::Video
    cv::VideoCapture video;
};

#endif // MAINWINDOW_H
