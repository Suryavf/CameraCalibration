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
    video = cv::VideoCapture(pathTo.toUtf8().constData());

    std::ofstream outfile;
    outfile.open("/home/victor/Documentos/Imagenes/CameraCalibration/data/time20.txt", std::ios_base::app);

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

    vector<vector<Point3f>>  vRealPoints;
    vector<vector<Point2f>> vFramePoints;
    Size boardSize;


    int countTrue = 0, countFrames = 0, countCalibration = 0;
    while(video.isOpened()){
        video >> frame;

        if(!frame.empty()){
            //
            // Grid detection
            // ..............
            start_time = omp_get_wtime();
            gridDetection(frame,binarized,morphology,ellipses,result,minRect,framePoints,ellipseCount);
            timeLapse = float( (omp_get_wtime() - start_time)*1000 );


            // Ellipse count
            if(n_centers == 0) n_centers = ellipseCount;
            outfile << std::to_string(timeLapse) + "\t";

            // True detection
            ++countFrames;

            //std::cout << ellipseCount << endl;

            if(ellipseCount == n_centers) ++countTrue;

            //
            // Calibration
            // ...........

            if(countTrue%2==0 &&  ellipseCount == n_centers && countCalibration < 20){
                if(ellipseCount == 12) boardSize = Size(4,3);
                if(ellipseCount == 20) boardSize = Size(5,4);

                calcBoardCornerPositions(boardSize, 4.8f, realPoints);
                 vRealPoints.push_back( realPoints);
                vFramePoints.push_back(framePoints);

                ++countCalibration;

            }else if( ellipseCount == n_centers && countCalibration == 20 ){

                Mat cameraMatrix = Mat::  eye(3, 3, CV_64F);
                Mat distCoeffs   = Mat::zeros(8, 1, CV_64F);
                vector<Mat> rvecs, tvecs;

                double totalAvgErr = calibrateCamera(vRealPoints,
                                vFramePoints,
                                frame.size(),
                                cameraMatrix,
                                distCoeffs,
                                rvecs, tvecs,
                                CALIB_FIX_ASPECT_RATIO | CALIB_RATIONAL_MODEL |  CALIB_FIX_PRINCIPAL_POINT | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K4 | CALIB_FIX_K5); // CALIB_FIX_PRINCIPAL_POINT
                vector<float> reprojErrs;
                //double totalAvgErr = computeReprojectionErrors(vRealPoints, vFramePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

                ui->focalLength->setText(  "(" + QString::number( cameraMatrix.at<double>(0,0) )
                                         + "," + QString::number( cameraMatrix.at<double>(1,1) ) + ")" );

                ui->principalPoint->setText(  "(" + QString::number( cameraMatrix.at<double>(0,2) )
                                            + "," + QString::number( cameraMatrix.at<double>(1,2) ) + ")" );

                ui->totalAvgErr->setText(  QString::number( totalAvgErr ) );

                std::cout << "(" <<  cameraMatrix.at<double>(0,0) << ","  <<  cameraMatrix.at<double>(1,1) << ")" << std::endl;
                std::cout << "(" <<  cameraMatrix.at<double>(0,2) << ","  <<  cameraMatrix.at<double>(1,2) << ")" << std::endl;

                std::cout << "totalAvgErr: " << totalAvgErr << std::endl;

                ++countCalibration;
            }



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

            outfile << std::to_string(ellipseCount) + "\n";

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

    outfile.close();

}

void MainWindow::on_actionOpen_triggered(){
    pathTo = QFileDialog::getOpenFileName(this, tr("Open video"), "",
                                                tr("Video (*.avi *.mp4);;All Files (*)"));
    //if(!pathTo.isNull()){
    //    dialogSetting();
    //}
}

void MainWindow::on_actionGrid3x4_triggered(){
    pathTo = "/home/victor/Documentos/Imagenes/CameraCalibration/data/padron1.avi";

    dialogSetting();
}

void MainWindow::on_actionGrid4x5_triggered(){
    pathTo = "/home/victor/Documentos/Imagenes/CameraCalibration/data/padron2.avi";

    setting *w = new setting();
    w->show();
}

void MainWindow::dialogSetting(){

    // Structure
    QDialog * d = new QDialog();
    QGridLayout *vbox = new QGridLayout();

    // Define Labels
    QLabel *labelVidFile = new QLabel();
    QLabel *labelSttFile = new QLabel();
    QLabel *labelTypGrid = new QLabel();
    QLabel *labelWidth   = new QLabel();
    QLabel *labelHeight  = new QLabel();
    QLabel *labelSqSize  = new QLabel();

    labelVidFile->setText("Video file:"  );
    labelSttFile->setText("Setting file:");
    labelTypGrid->setText("Type grid:"   );
    labelWidth  ->setText("Width:"       );
    labelHeight ->setText("Height:"      );
    labelSqSize ->setText("Square size:" );


    // Define Line Edits
    QLineEdit * lineEdVidFile = new QLineEdit();
    QLineEdit * lineEdSttFile = new QLineEdit();
    QLineEdit * lineEdWidth   = new QLineEdit();
    QLineEdit * lineEdHeight  = new QLineEdit();
    QLineEdit * lineEdSqSize  = new QLineEdit();

    lineEdVidFile->setFixedWidth(220);
    lineEdSttFile->setFixedWidth(220);

    // Define Combo Box
    QComboBox * comboBoxType = new QComboBox();
    comboBoxType->addItems(QStringList() << "Rings grid"
                                         << "Chessboard grid"
                                         << "Circles grid"
                                         << "Asymmetric circles grid");

    // Define bottons
    QPushButton * buttonBrowseVidFile = new QPushButton("Browse", this);
    QPushButton * buttonBrowseSttFile = new QPushButton("Browse", this);

    QObject::connect(buttonBrowseVidFile, SIGNAL(released()), this, SLOT(browseVideoClicked()  ));
    QObject::connect(buttonBrowseSttFile, SIGNAL(released()), this, SLOT(browseSettingClicked()));

    QDialogButtonBox * buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));

    //
    // Add Widget: Video file
    // ----------
    vbox->addWidget(       labelVidFile,20, 20);
    vbox->addWidget(      lineEdVidFile,20,110);
    vbox->addWidget(buttonBrowseVidFile,20,340);

    //
    // Add Widget: Setting file
    // ----------
    vbox->addWidget(       labelSttFile,60, 20);
    vbox->addWidget(      lineEdSttFile,60,110);
    vbox->addWidget(buttonBrowseSttFile,60,340);

    //
    // Add Widget: Type grid
    // ----------
    vbox->addWidget(       labelTypGrid,3000, 20);
    vbox->addWidget(       comboBoxType,3000, 80);



    //QGroupBox *groupBoxSetting = new QGroupBox(tr("Settings"));
    //groupBoxSetting->setMinimumWidth(380);
    //groupBoxSetting->setMinimumWidth(120);

    //vbox->addWidget(groupBoxSetting,90,20);




    d->setLayout(vbox);
    int result = d->exec();
    /*if(result == QDialog::Accepted)
    {
        // handle values from d
        qDebug() << a->text() << "Grid settings:"
      //           << "Rows:" << lineEditGridRow->text()
                 << "Cols:" << lineEditGridCol->text()
                 << "Type:" << comboBoxType->currentText();
    }
*/
//    type = comboBoxType->currentText();
//    rows = lineEditGridRow->text().toInt();
//    cols = lineEditGridCol->text().toInt();
}

void MainWindow::browseVideoClicked(){
    pathTo = QFileDialog::getOpenFileName(this, tr("Open video"), "",
                                                tr("Video (*.avi *.mp4);;All Files (*)"));
}

void MainWindow::browseSettingClicked(){
    pathTo = QFileDialog::getOpenFileName(this, tr("Open setting file"), "",
                                                tr("Video (*.xml);;All Files (*)"));
}
