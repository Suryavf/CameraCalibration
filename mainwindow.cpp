#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Preprocessing/preprocessing.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow){
    ui->setupUi(this);

    // Create Original scene
    ui->original->setScene(new QGraphicsScene(this));
    ui->original->scene()->addItem(&PixOriginal);

    // Create Binarized scene
    ui->binarized->setScene(new QGraphicsScene(this));
    ui->binarized->scene()->addItem(&PixBinarized);

    // Create Morphology scene
    ui->morphology->setScene(new QGraphicsScene(this));
    ui->morphology->scene()->addItem(&PixMorphology);

    // Create Ellipses scene
    ui->ellipses->setScene(new QGraphicsScene(this));
    ui->ellipses->scene()->addItem(&PixEllipses);

    // Create Result scene
    ui->result->setScene(new QGraphicsScene(this));
    ui->result->scene()->addItem(&PixResult);
}

MainWindow::~MainWindow(){
    delete ui;
}


void MainWindow::closeEvent(QCloseEvent *event){
    if(video.isOpened()){
        QMessageBox::warning(this,"Warning", "Stop the video before closing the application!");
        event->ignore();
    }
    else{
        event->accept();
    }
}

void MainWindow::on_pushButton_clicked(){

    video = cv::VideoCapture("/home/victor/Documentos/Imagenes/CameraCalibration/data/padron2.avi");

    if (!video.isOpened()){
        printf("Failed to open the video");
    }

    cv::Mat frame,binarized,morphology,ellipses,result;

    while(video.isOpened()){
        video >> frame;

        if(!frame.empty()){
            // Grid detection
            gridDetection(frame,binarized,morphology,ellipses,result);

            // Frame (Original)
            QImage qframe(frame.data,frame.cols,frame.rows,int(frame.step),QImage::Format_RGB888);

            PixOriginal.setPixmap( QPixmap::fromImage(qframe.rgbSwapped()) );
            ui->original->fitInView(&PixOriginal, Qt::KeepAspectRatio);

            // Binarized
            QImage qbinarized(binarized.data,binarized.cols,binarized.rows,int(binarized.step),QImage::Format_Grayscale8);

            PixBinarized.setPixmap( QPixmap::fromImage(qbinarized.rgbSwapped()) );
            ui->binarized->fitInView(&PixBinarized, Qt::KeepAspectRatio);

            // Morphology
            QImage qmorphology(morphology.data,morphology.cols,morphology.rows,int(morphology.step),QImage::Format_Grayscale8);

            PixMorphology.setPixmap( QPixmap::fromImage(qmorphology.rgbSwapped()) );
            ui->morphology->fitInView(&PixMorphology, Qt::KeepAspectRatio);

            // Ellipses
            QImage qellipses(ellipses.data,ellipses.cols,ellipses.rows,int(ellipses.step),QImage::Format_RGB888);

            PixEllipses.setPixmap( QPixmap::fromImage(qellipses.rgbSwapped()) );
            ui->ellipses->fitInView(&PixEllipses, Qt::KeepAspectRatio);

        }
        qApp->processEvents();
    }

}
