#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Preprocessing/preprocessing.h"

#include <fstream>

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
    float timeLapse;
    int ellipseCount;
    double start_time;
    video = cv::VideoCapture("/home/victor/Documentos/Imagenes/CameraCalibration/data/padron2.avi");

    std::ofstream outfile;
    outfile.open("time.txt", std::ios_base::app);

    if (!video.isOpened()){
        printf("Failed to open the video");
    }

    cv::Mat frame,binarized,morphology,ellipses,result;
    cv::RotatedRect minRect;

    video >> frame;
    minRect = cv::RotatedRect(cv::Point(frame.rows,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,frame.cols));

    int countTrue = 0, countFrames = 0;
    while(video.isOpened()){
        video >> frame;

        if(!frame.empty()){
            //
            // Grid detection
            // ..............
            start_time = omp_get_wtime();
            gridDetection(frame,binarized,morphology,ellipses,result,minRect,ellipseCount);
            timeLapse = float( (omp_get_wtime() - start_time)*1000 );

            outfile << std::to_string(timeLapse) + "\n";

            ++countFrames;
            if(ellipseCount == 20) ++countTrue;

            //
            // Drawing
            // .......

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

            // Result
            QImage qresult(result.data,result.cols,result.rows,int(result.step),QImage::Format_RGB888);

            PixResult.setPixmap( QPixmap::fromImage(qresult.rgbSwapped()) );
            ui->result->fitInView(&PixResult, Qt::KeepAspectRatio);

            //
            // Print text
            // ..........
            ui->timeFrame->setText(QString::number( int(timeLapse)) + " ms");
            ui->ellipseCount->setText(QString::number(ellipseCount));

            if(ellipseCount == 20){
                ui->status->setText("Correct detection!");
                ui->status->setStyleSheet("QLabel { color : black; }");
            }else if( ellipseCount < 20 ){
                ui->status->setText("Loss ellipses");
                 ui->status->setStyleSheet("QLabel { color : red; }");
            }else{
                ui->status->setText("Noise ellipse");
                 ui->status->setStyleSheet("QLabel { color : red; }");
            }

            ui->accuracy->setText(QString::number( int(float(countTrue)*100/float(countFrames))  ) + " %");
        }
        else{
             break;
        }

        qApp->processEvents();
    }
    outfile.close();

}
