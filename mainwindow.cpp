#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Preprocessing/preprocessing.h"

#include <fstream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
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

    pathTo = "/home/victor/Documentos/Imagenes/CameraCalibration/data/padron1.avi";
    mode = 0;   // test | calibrated
}


MainWindow::~MainWindow(){
    delete ui;
}


void MainWindow::closeEvent(QCloseEvent *event){
    event->accept();
}

void MainWindow::testRoutine(){
    float timeLapse;
    int ellipseCount;
    double start_time;
    video = cv::VideoCapture(pathTo.toUtf8().constData());

    if (!video.isOpened()){
        printf("Failed to open the video");
    }

    cv::Mat frame,binarized,morphology,ellipses,result;
    cv::RotatedRect minRect;

    video >> frame;
    minRect = cv::RotatedRect(cv::Point(frame.rows,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,frame.cols));
    n_centers = 0;
    vector<Point3f>  realPoints;
    vector<Point2f> framePoints;

    int countTrue = 0, countFrames = 0;
    while(video.isOpened()){
        video >> frame;

        if(!frame.empty()){

            // Grid detection
            start_time = omp_get_wtime();
            gridDetection(frame,binarized,morphology,ellipses,result,minRect,framePoints,ellipseCount);
            timeLapse = float( (omp_get_wtime() - start_time)*1000 );

            if(n_centers == 0) n_centers = ellipseCount;    // Ellipse count
            ++countFrames;                                  // Frames count
            if(ellipseCount == n_centers) ++countTrue;      // True detection count

            // Drawing
            drawWindows(frame,binarized,morphology,ellipses,result);

            // Print text
            ui->timeFrame   ->setText(QString::number( int(timeLapse)) + " ms");
            ui->ellipseCount->setText(QString::number(ellipseCount));

            if(ellipseCount == n_centers){
                ui->status->setText("Correct detection!");
                ui->status->setStyleSheet("QLabel { color : black; }");
            }else if( ellipseCount < n_centers ){
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
}


void MainWindow::calibrationRoutine(){
    /*
    ui->focalLength->setText(  "(" + QString::number( cameraMatrix.at<double>(0,0) )
                             + "," + QString::number( cameraMatrix.at<double>(1,1) ) + ")" );

    ui->principalPoint->setText(  "(" + QString::number( cameraMatrix.at<double>(0,2) )
                                + "," + QString::number( cameraMatrix.at<double>(1,2) ) + ")" );

    ui->totalAvgErr->setText(  QString::number( totalAvgErr ) );

    std::cout << "(" <<  cameraMatrix.at<double>(0,0) << ","  <<  cameraMatrix.at<double>(1,1) << ")" << std::endl;
    std::cout << "(" <<  cameraMatrix.at<double>(0,2) << ","  <<  cameraMatrix.at<double>(1,2) << ")" << std::endl;

    std::cout << "totalAvgErr: " << totalAvgErr << std::endl;
    */
}


void MainWindow::on_actionOpen_triggered(){
    setting *w = new setting();
    w->show();

    if(w->exec()) w->getValues(pathTo,type,squareSize,rows,cols);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::drawWindows(cv::Mat &sec1, cv::Mat &sec2,cv::Mat &sec3, cv::Mat &sec4, cv::Mat &_main){

    // Frame (Original)
    QImage qframe(sec1.data,sec1.cols,sec1.rows,int(sec1.step),QImage::Format_RGB888);

    PixOriginal.setPixmap( QPixmap::fromImage(qframe.rgbSwapped()) );
    ui->original->fitInView(&PixOriginal, Qt::KeepAspectRatio);

    // Binarized
    QImage qbinarized(sec2.data,sec2.cols,sec2.rows,int(sec2.step),QImage::Format_Grayscale8);

    PixBinarized.setPixmap( QPixmap::fromImage(qbinarized.rgbSwapped()) );
    ui->binarized->fitInView(&PixBinarized, Qt::KeepAspectRatio);

    // Morphology
    QImage qmorphology(sec3.data,sec3.cols,sec3.rows,int(sec3.step),QImage::Format_Grayscale8);

    PixMorphology.setPixmap( QPixmap::fromImage(qmorphology.rgbSwapped()) );
    ui->morphology->fitInView(&PixMorphology, Qt::KeepAspectRatio);

    // Ellipses
    QImage qellipses(sec4.data,sec4.cols,sec4.rows,int(sec4.step),QImage::Format_RGB888);

    PixEllipses.setPixmap( QPixmap::fromImage(qellipses.rgbSwapped()) );
    ui->ellipses->fitInView(&PixEllipses, Qt::KeepAspectRatio);

    // Result
    QImage qresult(_main.data,_main.cols,_main.rows,int(_main.step),QImage::Format_RGB888);

    PixResult.setPixmap( QPixmap::fromImage(qresult.rgbSwapped()) );
    ui->result->fitInView(&PixResult, Qt::KeepAspectRatio);
}
