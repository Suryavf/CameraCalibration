#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Preprocessing/preprocessing.h"

#include <fstream>

/*
 * Basic Functions MainWindow
 * -----------------------------------------------------------------------------------------------
 */
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

    pathTo  = "/home/victor/Documentos/Imagenes/CameraCalibration/data/padron1.avi";
    mode    = 0;   // test | calibrated
    capture = false;

    ui->actionModeTest       ->setEnabled(false);
    ui->actionModeCalibration->setEnabled( true);
    ui->pushButtonCapture    ->setVisible(false);
}


MainWindow::~MainWindow(){
    delete ui;
}


void MainWindow::closeEvent(QCloseEvent *event){
    event->accept();
}


/*
 * Actions
 * -----------------------------------------------------------------------------------------------
 */
void MainWindow::on_actionOpen_triggered(){
    setting *w = new setting();
    w->show();

    if(w->exec()) w->getValues(pathTo,type,squareSize,width,height);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::on_actionModeTest_triggered(){
    mode = false;
    ui->actionModeTest       ->setEnabled(false);
    ui->actionModeCalibration->setEnabled( true);
    ui->pushButtonCapture    ->setVisible(false);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::on_actionModeCalibration_triggered(){
    mode = true;
    ui->actionModeTest       ->setEnabled( true);
    ui->actionModeCalibration->setEnabled(false);
    ui->pushButtonCapture    ->setVisible( true);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::on_pushButtonCapture_clicked(){
    capture = true;
}


/*
 * Runtines
 * -----------------------------------------------------------------------------------------------
 */
void MainWindow::testRoutine(){
    float timeLapse;
    int ellipseCount;
    double start_time;
    video = cv::VideoCapture(pathTo.toUtf8().constData());

    if (!video.isOpened()) printf("Failed to open the video");

    cv::Mat frame,binarized,morphology,ellipses,result;
    cv::RotatedRect minRect;

    video >> frame;
    minRect = cv::RotatedRect(cv::Point(frame.rows,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,frame.cols));
    n_centers = width*height;
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
 *  Parameters
 *  ----------
 */
    float timeLapse;
    int countPoints;

    cv::Mat frame,binarized,morphology,ellipses,result;
    cv::Mat cloudPoints,cloudPointsOut;
    cv::RotatedRect minRect;

    vector<Point3f>  realPoints;
    vector<Point2f> framePoints;
    vector<vector<Point2f> > calibratePoints;

    bool loadingCalibrate = true;

/*
 *  Initialize
 *  ----------
 */
    minRect = cv::RotatedRect(cv::Point(frame.rows,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,frame.cols));
    n_centers = width*height;


    cv::Size patternSize = cv::Size(width,height);


    std::cout << "width: " << width << ". height: " << height << std::endl;
    std::cout << "main: patternSize: (" << patternSize.width << "," << patternSize.height << ")" << std::endl;


/*
 *  Video
 *  -----
 */
    video = cv::VideoCapture(pathTo.toUtf8().constData());
    if (!video.isOpened()) printf("Failed to open the video");
    video >> frame;
    cloudPoints = cv::Mat(frame.size(), CV_8UC3, Scalar(0,0,0)); ;
    cloudPoints.copyTo(cloudPointsOut);

/*
 *  Main loop
 *  ---------
 */
    int countTrue = 0, countFrames = 0;
    Mat cameraMatrix, distCoeffs;
    while(video.isOpened()){
        video >> frame;

        if(!frame.empty()) {

        /*
         *  Capture frames
         *  ....................................................................................................
         *
         */
            if( calibratePoints.size()<FRAMES_BY_CALIBRATE ){

                if( findPattern(frame,binarized,
                                morphology,ellipses,
                                result,
                                patternSize,
                                minRect,
                                framePoints,type,
                                countPoints,
                                timeLapse) ){

                    // Mejora la precisión de las coordenadas de las esquinas encontradas para el tablero de ajedrez
                    if( type == "Chessboard grid" ){
                        Mat viewGray;
                        cvtColor(frame, viewGray, COLOR_BGR2GRAY);
                        cornerSubPix( viewGray, cloudPoints, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                    }

                    if( type != "Ring grid" ){
                        result = frame;
                        drawChessboardCorners( result, result.size(), Mat(framePoints), true );
                    }

                //- Capture frame
                    if(capture){

                        // Save points
                        calibratePoints.push_back(framePoints);

                        // Draw points
                        for (size_t i = 0; i < framePoints.size(); ++i)
                            circle(cloudPoints, framePoints[i], 2, Scalar(221, 255, 51), -1, 8, 0);

                        capture = false;
                    }
                    else{
                        cloudPoints.copyTo(cloudPointsOut);

                        // Draw points
                        for (size_t i = 0; i < framePoints.size(); ++i)
                            circle(cloudPointsOut, framePoints[i], 2, Scalar(0, 0, 255), -1, 8, 0);
                    }


                    ++countTrue;  // True detection count
                }else{
                    if( type != "Ring grid" ){
                        result = frame;
                        drawChessboardCorners( result, result.size(), Mat(framePoints), false );
                    }
                }

                ++countFrames;  // Frames count

                // Drawing
                drawWindows(frame,binarized,morphology,result,cloudPointsOut);

                // Print text
                ui->timeFrame   ->setText(QString::number( int(timeLapse)) + " ms");
                ui->ellipseCount->setText(QString::number(countPoints));

                QString status = "Capture frame " + QString::number( calibratePoints.size() ) + "/" + QString::number( FRAMES_BY_CALIBRATE );
                ui->status->setText(status);
                ui->status->setStyleSheet("QLabel { color : black; }");

                ui->accuracy->setText(QString::number( int(float(countTrue)*100/float(countFrames))  ) + " %");
            }

        /*
         *  Calibrate Camara
         *  ....................................................................................................
         *
         */
            else{

            //
            //- Calibrate execute
            //  -----------------
                if(loadingCalibrate){

                    // Warning
                    {
                    ui->status->setText("Loading...");
                    ui->status->setStyleSheet("QLabel { color : red; }");
                    }


                    double totalAvgErr = 0;
                    float sqSize = float(squareSize);
                    Size imageSize = frame.size();

                    // Calibrated
                    runCalibrationAndSave(calibratePoints,
                                          type,sqSize,
                                          patternSize,imageSize,
                                          cameraMatrix,distCoeffs,
                                          totalAvgErr);

                    // Update text
                    ui->focalLength   ->setText(  "(" + QString::number( cameraMatrix.at<double>(0,0) )
                                                + "," + QString::number( cameraMatrix.at<double>(1,1) ) + ")" );
                    ui->principalPoint->setText(  "(" + QString::number( cameraMatrix.at<double>(0,2) )
                                                + "," + QString::number( cameraMatrix.at<double>(1,2) ) + ")" );
                    ui->totalAvgErr->setText(  QString::number( totalAvgErr ) );

                    ui->status->setText("Calibrated image");
                    ui->status->setStyleSheet("QLabel { color : black; }");

                    loadingCalibrate = false;
                }

            //
            //- Post-Calibrate
            //  -----------------
                else{

                    // Procesamiento
                    findPattern(frame,binarized,
                                morphology,ellipses,
                                result,
                                patternSize,
                                minRect,
                                framePoints,type,
                                countPoints,
                                timeLapse);


                    // Calcular imagen rectificada (undistorted)
                    Mat rview, map1, map2;
                    Mat opt = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frame.size(), 1);
                    initUndistortRectifyMap( cameraMatrix, distCoeffs, Mat(), opt,
                                             frame.size(),CV_16SC2, map1, map2);
                    remap(frame, rview, map1, map2, INTER_LINEAR);

                    /*
                    // Fronto-Parallel
                    Mat temp = result.clone();
                    undistort(temp, result, cameraMatrix, distCoeffs);

                    float radioPeque = float(result.cols)/24.0f;
                    vector<Point2f> dst_vertices;
                    dst_vertices.push_back( Point(0, 0) );
                    dst_vertices.push_back( Point(result.cols, 0) );
                    dst_vertices.push_back( Point(0, result.rows) );
                    dst_vertices.push_back( Point(result.cols, result.rows) );

                    vector<Point2f> src_vertices = getExtremePoints(result, framePoints, dst_vertices, int(radioPeque*3));

                    for(size_t i=0; i < src_vertices.size(); i++){
                        circle(result, src_vertices[i], 4,cv::Scalar(35,255,75), -1, 8, 0);
                    }
                    radioPeque = float(result.cols)/30.0f;

                    Mat rotated;
                    Mat H = findHomography(src_vertices, dst_vertices);
                    //warpPerspective(view_auxiliary, rotated, H, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);
                    warpPerspective(result, rotated, H, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);
*/
                    // Draw
                    drawWindows(frame,binarized,result,result,rview);

                    //ui->status->setText("Calibrated image");
                    //ui->status->setStyleSheet("QLabel { color : black; }");
                }
            }
        }
        else{
             break;
        }

        qApp->processEvents();

    }


    /*

bool findPattern(cv::Mat &frame     , cv::Mat &binarized,
                 cv::Mat &morphology, cv::Mat &ellipses ,
                 cv::Mat &result    ,
                 cv::RotatedRect &minRec,
                 vector<Point2f> &framePoints,
                 QString &type);

    */







    /*


    std::cout << "(" <<  cameraMatrix.at<double>(0,0) << ","  <<  cameraMatrix.at<double>(1,1) << ")" << std::endl;
    std::cout << "(" <<  cameraMatrix.at<double>(0,2) << ","  <<  cameraMatrix.at<double>(1,2) << ")" << std::endl;

    std::cout << "totalAvgErr: " << totalAvgErr << std::endl;
    */
}


/*
 * General function
 * -----------------------------------------------------------------------------------------------
 */
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

