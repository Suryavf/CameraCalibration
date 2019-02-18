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
    mode            = false;  // test | calibrated
    calibrationMode = false;  // manual | automatic
    capture         = false;

    gridSize = 10;

    ui->actionModeTest           ->setEnabled(false);
    ui->actionModeCalibration    ->setEnabled( true);
    ui->pushButtonCapture        ->setVisible(false);
    ui->pushButtonCalibrationMode->setVisible(false);

    // Not visible options (Calibration)
    ui->     totalAvgErr   ->setVisible(false);
    ui->labelTotalAvgErr   ->setVisible(false);
    ui->     focalLength   ->setVisible(false);
    ui->labelFocalLength   ->setVisible(false);
    ui->     principalPoint->setVisible(false);
    ui->labelPrincipalPoint->setVisible(false);
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
    ui->actionModeTest           ->setEnabled(false);
    ui->actionModeCalibration    ->setEnabled( true);
    ui->pushButtonCapture        ->setVisible(false);
    ui->pushButtonCalibrationMode->setVisible(false);

    // Not visible options (Calibration)
    ui->     totalAvgErr   ->setVisible(false);
    ui->labelTotalAvgErr   ->setVisible(false);
    ui->     focalLength   ->setVisible(false);
    ui->labelFocalLength   ->setVisible(false);
    ui->     principalPoint->setVisible(false);
    ui->labelPrincipalPoint->setVisible(false);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::on_actionModeCalibration_triggered(){
    mode = true;
    ui->actionModeTest           ->setEnabled( true);
    ui->actionModeCalibration    ->setEnabled(false);
    ui->pushButtonCapture        ->setVisible(!calibrationMode);
    ui->pushButtonCalibrationMode->setVisible(            mode);

    // Not visible options (Calibration)
    ui->     totalAvgErr   ->setVisible(true);
    ui->labelTotalAvgErr   ->setVisible(true);
    ui->     focalLength   ->setVisible(true);
    ui->labelFocalLength   ->setVisible(true);
    ui->     principalPoint->setVisible(true);
    ui->labelPrincipalPoint->setVisible(true);

    if (mode) calibrationRoutine();
    else             testRoutine();
}

void MainWindow::on_pushButtonCapture_clicked(){
    capture = true;
}

void MainWindow::on_pushButtonCalibrationMode_clicked(){
//  Manual: 0, Automatic: 1
    if(calibrationMode){
        calibrationMode = false;
        ui->pushButtonCalibrationMode->setText("Manual");
        ui->pushButtonCapture ->setVisible(true);

    }else{
        calibrationMode =  true;
        ui->pushButtonCalibrationMode->setText("Automatic");
        ui->pushButtonCapture ->setVisible(false);
    }

}


/*
 * Runtines
 * -----------------------------------------------------------------------------------------------
 */
void MainWindow::testRoutine(){
    float   timeLapse = 0.0f;
    int  patternCount =   -1;
    double start_time;
    video = cv::VideoCapture(pathTo.toUtf8().constData());

    if (!video.isOpened()) printf("Failed to open the video");

    cv::Mat frame,binarized,morphology,pattern,result;
    cv::Mat gray;
    cv::RotatedRect minRect;

    fps = int(video.get(CV_CAP_PROP_FPS));
    cout << "Frame per seconds: " << fps << endl;

    video >> frame;
    minRect = cv::RotatedRect(cv::Point(frame.rows,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,frame.cols));
    n_centers = width*height;
    vector<Point3f>  realPoints;
    vector<Point2f> framePoints;

    bool found;
    int countTrue = 0, countFrames = 0;
    std::chrono::system_clock::time_point timeNext;
    while(video.isOpened()){
        video >> frame;
        timeNext = std::chrono::system_clock::now() + std::chrono::milliseconds( int(800/fps) );

        if(!frame.empty()){
            // Frames count
            ++countFrames;

            /*
             * Ring grid routine
             * -----------------
             */
            if(type == "Ring grid"){

                // Grid detection
                start_time = omp_get_wtime();
                gridDetection(frame,
                              binarized,morphology,pattern,
                              result,
                              minRect,
                              framePoints,patternCount);
                timeLapse = float( (omp_get_wtime() - start_time)*1000 );
                if(patternCount == n_centers) ++countTrue;  // True detection count
            }

            /*
             * Chessboard grid routine
             * -----------------------
             */
            if(type == "Chessboard grid"){

                // Find chessboard
                start_time = omp_get_wtime();
                found = findChessboardCorners( frame,
                                               Size(width,height),
                                               framePoints,
                                               CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                timeLapse = float( (omp_get_wtime() - start_time)*1000 );
                if(found) ++countTrue;
                patternCount = found? width*height:0;

                // Create binarized, morphology
                cvtColor(frame, binarized, CV_BGR2GRAY);
                GaussianBlur(binarized, binarized, Size(9, 9), 2, 2);
                adaptiveThreshold(binarized, morphology, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);

                // Pattern
                pattern = cv::Mat::zeros(frame.size(),CV_8UC3);
                for(size_t i=0; i < framePoints.size(); i++)
                    circle(pattern, framePoints[i], 4,cv::Scalar(35,255,75), -1, 8, 0);

                // Result
                frame.copyTo(result);
                for(size_t i=0; i < framePoints.size(); i++){
                    circle(result, framePoints[i], 4,cv::Scalar(253,35,255), -1, 8, 0);
                    putText(result,    std::to_string(i+1),
                                            framePoints[i], // Coordinates
                                   cv::FONT_HERSHEY_DUPLEX, // Font
                                                       0.5, // Scale. 2.0 = 2x bigger
                                     cv::Scalar(35,255,75), // BGR Color
                                                        1); // Line Thickness (Optional)
                }
            }


            /*
             * Circles grid routine
             * --------------------
             */
            if(type == "Circles grid"){

                // Find chessboard
                start_time = omp_get_wtime();
                found = findCirclesGrid( frame,
                                         Size(width,height),
                                         framePoints );
                timeLapse = float( (omp_get_wtime() - start_time)*1000 );
                if(found) ++countTrue;
                patternCount = found? width*height:0;

                // Create binarized, morphology
                cvtColor(frame, binarized, CV_BGR2GRAY);
                GaussianBlur(binarized, binarized, Size(9, 9), 2, 2);
                adaptiveThreshold(binarized, morphology, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);

                // Pattern
                pattern = cv::Mat::zeros(frame.size(),CV_8UC3);
                for(size_t i=0; i < framePoints.size(); i++)
                    circle(pattern, framePoints[i], 4,cv::Scalar(35,255,75), -1, 8, 0);

                // Result
                frame.copyTo(result);
                for(size_t i=0; i < framePoints.size(); i++){
                    circle(result, framePoints[i], 4,cv::Scalar(253,35,255), -1, 8, 0);
                    putText(result,    std::to_string(i+1),
                                            framePoints[i], // Coordinates
                                   cv::FONT_HERSHEY_DUPLEX, // Font
                                                      0.65, // Scale. 2.0 = 2x bigger
                                     cv::Scalar(35,255,75), // BGR Color
                                                        1); // Line Thickness (Optional)
                }
            }


            /*
             * Asymmetric Circles grid routine
             * -------------------------------
             */
            if(type == "Asymmetric Circles grid"){

                // Find chessboard
                start_time = omp_get_wtime();
                found = findCirclesGrid( frame,
                                         Size(width,height),
                                         framePoints,
                                         CALIB_CB_ASYMMETRIC_GRID );

                timeLapse = float( (omp_get_wtime() - start_time)*1000 );
                if(found) ++countTrue;
                patternCount = found? width*height:0;

                // Create binarized, morphology
                cvtColor(frame, binarized, CV_BGR2GRAY);
                GaussianBlur(binarized, binarized, Size(9, 9), 2, 2);
                adaptiveThreshold(binarized, morphology, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);

                // Pattern
                pattern = cv::Mat::zeros(frame.size(),CV_8UC3);
                for(size_t i=0; i < framePoints.size(); i++)
                    circle(pattern, framePoints[i], 4,cv::Scalar(35,255,75), -1, 8, 0);

                // Result
                frame.copyTo(result);
                for(size_t i=0; i < framePoints.size(); i++){
                    circle(result, framePoints[i], 4,cv::Scalar(253,35,255), -1, 8, 0);
                    putText(result,    std::to_string(i+1),
                                            framePoints[i], // Coordinates
                                   cv::FONT_HERSHEY_DUPLEX, // Font
                                                       0.5, // Scale. 2.0 = 2x bigger
                                     cv::Scalar(35,255,75), // BGR Color
                                                        1); // Line Thickness (Optional)
                }
            }


            //
            // Drawing
            // .......
            drawWindows(frame,binarized,morphology,pattern,result);

            //
            // Print text
            // ..........
            ui->timeFrame   ->setText(QString::number( int(timeLapse)) + " ms");
            ui->ellipseCount->setText(QString::number(patternCount));

            if(patternCount == n_centers){
                ui->status->setText("Correct detection!");
                ui->status->setStyleSheet("QLabel { color : black; }");
            }else if( patternCount < n_centers ){
                ui->status->setText("Loss ellipses");
                ui->status->setStyleSheet("QLabel { color : red; }");
            }else{
                ui->status->setText("Noise ellipse");
                ui->status->setStyleSheet("QLabel { color : red; }");
            }
            ui->accuracy->setText(QString::number( int(float(countTrue)*100/float(countFrames))  ) + " %");

            // For next
            found = false;
        }
        else{
             break;
        }

        // Pause per time
        std::this_thread::sleep_until(timeNext);
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

    vector<cv::Mat> selectFrames;
    vector<Point3f>  realPoints;
    vector<Point2f> framePoints;
    vector<vector<Point2f> > calibratePoints;

    //int  **gridCloudPoints;
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

    vector<vector<int>> gridCloudPoints(gridSize, vector<int> (gridSize, 0));

    vector<Point2f> gridRealPoints;
    calcBoardCornerPositions(patternSize, squareSize, gridRealPoints, type);

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,100000);

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
    Mat cameraMatrix, distCoeffs;
    Mat imgCorregida;
    Mat frontoParallel;
    int countTrue = 0, countFrames = 0;
    int lastSelect = 0; // Only automatic selection for camera calibration
    std::chrono::system_clock::time_point timeNext;
    while(video.isOpened()){
        video >> frame;
        timeNext = std::chrono::system_clock::now() + std::chrono::milliseconds( int(800/fps) );

        if(!frame.empty()) {

        /*
         *  Capture frames
         *  ....................................................................................................
         *
         */
            if( calibratePoints.size()<FRAMES_BY_CALIBRATE ){

            //
            //- Grid detection
            //  **************
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
                        frame.copyTo(result);
                        drawChessboardCorners( result, result.size(), Mat(framePoints), true );
                    }


                //- Automatic frame selection
                    if(calibrationMode){
                        double area = frame.rows*frame.cols;
                        Size windowsize = frame.size();
                        int timelapse = countFrames - lastSelect;

                        float roulette = float(dist(rng))/100000.0f;
                        float prob = frameSelection(framePoints, gridRealPoints, gridCloudPoints, gridSize, windowsize, area,timelapse);

                        cout << "Roulette: " << roulette << ". Probability: " << prob << ". Capture: " << capture << endl << endl << endl;

                        capture = ( roulette<prob );
                    }

                //- Capture frame
                    if(capture){
                        lastSelect = countFrames;

                        // Automatic frame selection
                        if(calibrationMode){
                            Size windowsize = frame.size();
                            addPointsToCloudPoints(framePoints, gridCloudPoints, gridSize, windowsize);
                        }

                        // Save points
                        calibratePoints.push_back(framePoints);
                        selectFrames   .push_back(frame      );

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

            //
            //- Grid detection
            //  **************
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
                    ui->status->setText("Loading...");
                    ui->status->setStyleSheet("QLabel { color : red; }");

                    double totalAvgErr = 0;
                    float sqSize = float(squareSize);
                    Size imageSize = frame.size();

                    //
                    // Basic Calibrated
                    // ----------------
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


                    //
                    // Iterative method
                    // ----------------
                    if(type == "Ring grid"){

                        size_t n_iteration = 50;
                        for (size_t ite = 0; ite < n_iteration; ++ite){

                            // Loop images
                            for(size_t s = 0; s < selectFrames.size(); ++s){
                                // Select Frame
                                frame = selectFrames[s];
                                vector<Point2f> puntosOriginales = calibratePoints[s];

                                // Fin pattern
                                findPattern(frame,binarized, morphology,ellipses,result,patternSize,
                                            minRect,framePoints,type,countPoints,timeLapse);

                                // Un-distorting
                                imgCorregida = frame.clone();
                                undistort(frame, framePoints, cameraMatrix, distCoeffs);

                                Mat imgCorregida2 = imgCorregida.clone();

                                vector<Point2f> framePointsCorregidos;
                                undistortPoints(puntosOriginales, framePointsCorregidos, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix); // ------ ???
                                drawChessboardCorners( imgCorregida, patternSize, Mat(framePointsCorregidos), true );

                                //---------Obteniendo Fronto-Parallel-------------
                                float radioPeque = frame.cols/24;
                                vector<Point2f> dst_vertices;
                                dst_vertices.push_back( Point(0, 0) );
                                dst_vertices.push_back( Point(frame.cols,          0) );
                                dst_vertices.push_back( Point(         0, frame.rows) );
                                dst_vertices.push_back( Point(frame.cols, frame.rows) );

                                vector<Point2f> src_vertices = getExtremePoints(imgCorregida, framePointsCorregidos, dst_vertices, int(radioPeque*3));

                                Mat H = findHomography(src_vertices, dst_vertices);


                                //FRONTO-PARALLEL FOR THE IMAGE
                                warpPerspective(imgCorregida2, frontoParallel, H, frontoParallel.size(), INTER_LINEAR, BORDER_CONSTANT);

                                //FRONTO-PARALLEL FOR THE
                                vector<Point2f>new_vertices;
                                perspectiveTransform( framePointsCorregidos, new_vertices, H);
                                for (size_t i = 0; i < framePointsCorregidos.size(); ++i){
                                    circle(frontoParallel, new_vertices[i], 8, Scalar(100, 0, 100), -1, 8, 0);
                                }

                                Mat frontoParallel2 = frontoParallel.clone();
                                Mat binarized2, morphology2, ellipses2, result2;
                                vector<Point2f> puntosFrontoP;
                                bool found2 = findRingsGrid2( frontoParallel2, binarized2, morphology2, ellipses2, result2, minRect, patternSize, puntosFrontoP);

                                if (found2){
                                    drawChessboardCorners( frontoParallel2, patternSize, Mat(puntosFrontoP), found2 );
                                }


                                vector<Point2f> pointBof_result;
                                if (found2){
                                    float x;
                                    float y;
                                    for (size_t i = 0; i < puntosFrontoP.size(); ++i){
                                        x = puntosFrontoP[i].x;
                                        y = puntosFrontoP[i].y;

                                        pointBof_result.push_back(Point2f(x,y));
                                    }
                                }
                                else{
                                    for (size_t i = 0; i < new_vertices.size(); ++i){

                                        pointBof_result.push_back(new_vertices[i]);
                                    }
                                }


                                drawChessboardCorners( frontoParallel, patternSize, Mat(pointBof_result), true );
                                //imshow("FRONTO-PARALLEL IMAGE", frontoParallel);

                                //return point to original image corregida
                                perspectiveTransform( pointBof_result, pointBof_result, H.inv());
                                drawChessboardCorners( imgCorregida2, patternSize, Mat(pointBof_result), true );
                                //imshow("Retutn Imagen Corregida", imgCorregida2);

                                //--------Regresando los puntos a la imagen con distorcion, osea a la capturada por la camara
                                float xDist, yDist, x, y;
                                float k1 = float(distCoeffs.at<double>(0,0));
                                float k2 = float(distCoeffs.at<double>(0,1));
                                float k3 = float(  distCoeffs.at<double>(0,4));
                                float cx = float(cameraMatrix.at<double>(0,2));
                                float cy = float(cameraMatrix.at<double>(1,2));
                                float fx = float(cameraMatrix.at<double>(0,0));
                                float fy = float(cameraMatrix.at<double>(1,1));
                                float r;

                                for(uint i=0; i<pointBof_result.size(); i++){
                                    x = (pointBof_result[i].x-cx)/fx;
                                    y = (pointBof_result[i].y-cy)/fy;
                                    r = sqrt(x*x+y*y);

                                    xDist = x*( 1 + k1*float(pow(r,2)) + k2*float(pow(r,4)) + k3*float(pow(r,6)) );
                                    yDist = y*( 1 + k1*float(pow(r,2)) + k2*float(pow(r,4)) + k3*float(pow(r,6)) );

                                    xDist = xDist * fx + cx;
                                    yDist = yDist * fy + cy;

                                    pointBof_result[i].x = xDist;
                                    pointBof_result[i].y = yDist;
                                }


                                vector<Point2f> pointBof_result_Final;
                                for (size_t i = 0; i < pointBof_result.size(); ++i){
                                    float x = (pointBof_result[i].x + puntosOriginales[i].x)/2;
                                    float y = (pointBof_result[i].y + puntosOriginales[i].y)/2;

                                    //x = pointBof_result[i].x;
                                    //y = pointBof_result[i].y;

                                    pointBof_result_Final.push_back(Point2f(x,y));
                                }

                                calibratePoints[s].clear();
                                calibratePoints[s] = pointBof_result_Final;

                            }

                            // Update text
                            ui->status->setText("Iteration " + QString::number(ite) + "/" + QString::number(n_iteration));
                            ui->focalLength   ->setText(  "(" + QString::number( cameraMatrix.at<double>(0,0) )
                                                        + "," + QString::number( cameraMatrix.at<double>(1,1) ) + ")" );
                            ui->principalPoint->setText(  "(" + QString::number( cameraMatrix.at<double>(0,2) )
                                                        + "," + QString::number( cameraMatrix.at<double>(1,2) ) + ")" );
                            ui->totalAvgErr->setText(  QString::number( totalAvgErr ) );

                        }

                    }

                    ui->status->setText("Calibrated image");
                    ui->status->setStyleSheet("QLabel { color : black; }");

                    loadingCalibrate = false;
                }

            //
            //- Ready rutine
            //  -----------------
                else {

                }
            }
        }
        else{
             break;
        }

        // Pause per time
        std::this_thread::sleep_until(timeNext);
        qApp->processEvents();
    }
}


/*
 * General function
 * -----------------------------------------------------------------------------------------------
 */
void MainWindow::drawWindows(cv::Mat &sec1, cv::Mat &sec2,cv::Mat &sec3, cv::Mat &sec4, cv::Mat &_main){
    QImage::Format format;

    // Frame (Original)
    if(sec1.channels() == 1) format = QImage::Format_Grayscale8;
    else                     format = QImage::Format_RGB888    ;
    QImage qframe(sec1.data,sec1.cols,sec1.rows,int(sec1.step),format);

    PixOriginal.setPixmap( QPixmap::fromImage(qframe.rgbSwapped()) );
    ui->original->fitInView(&PixOriginal, Qt::KeepAspectRatio);

    // Binarized
    if(sec2.channels() == 1) format = QImage::Format_Grayscale8;
    else                     format = QImage::Format_RGB888    ;
    QImage qbinarized(sec2.data,sec2.cols,sec2.rows,int(sec2.step),format);

    PixBinarized.setPixmap( QPixmap::fromImage(qbinarized.rgbSwapped()) );
    ui->binarized->fitInView(&PixBinarized, Qt::KeepAspectRatio);

    // Morphology
    if(sec3.channels() == 1) format = QImage::Format_Grayscale8;
    else                     format = QImage::Format_RGB888    ;
    QImage qmorphology(sec3.data,sec3.cols,sec3.rows,int(sec3.step),format);

    PixMorphology.setPixmap( QPixmap::fromImage(qmorphology.rgbSwapped()) );
    ui->morphology->fitInView(&PixMorphology, Qt::KeepAspectRatio);

    // Ellipses
    if(sec4.channels() == 1) format = QImage::Format_Grayscale8;
    else                     format = QImage::Format_RGB888    ;
    QImage qellipses(sec4.data,sec4.cols,sec4.rows,int(sec4.step),format);

    PixEllipses.setPixmap( QPixmap::fromImage(qellipses.rgbSwapped()) );
    ui->ellipses->fitInView(&PixEllipses, Qt::KeepAspectRatio);

    // Result
    if(_main.channels() == 1) format = QImage::Format_Grayscale8;
    else                      format = QImage::Format_RGB888    ;
    QImage qresult(_main.data,_main.cols,_main.rows,int(_main.step),format);

    PixResult.setPixmap( QPixmap::fromImage(qresult.rgbSwapped()) );
    ui->result->fitInView(&PixResult, Qt::KeepAspectRatio);
}

float MainWindow::frameSelection(vector<Point2f> &pts, vector<Point2f> &gridRealPoints, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize, double &totalArea,int &timelapse){
    float prob0 = probabilityByTime    (timelapse     );
    float prob1 = probabilityByArea    (pts, totalArea);
    float prob2 = probabilityByPosition(pts, gridCloudPoints,
                                        gridSize, windowsize);
    float prob3 = probabilityByAngle   (pts, gridRealPoints);

    cout << "Prob. Time: " << prob0 << ". Prob. Area: " << prob1 << ". Prob. Position: " << prob2  << ". Prob. Angle: " << prob3 << endl;
    return prob0*prob1*prob2;//*prob3;
}



float probabilityByAngle(vector<Point2f> &pts,vector<Point2f> &gridRealPoints){
    Mat H;

    H = findHomography(pts,gridRealPoints);

    double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                       H.at<double>(1,0)*H.at<double>(1,0) +
                       H.at<double>(2,0)*H.at<double>(2,0));

    H /= norm;
    Mat c1  = H.col(0);
    Mat c2  = H.col(1);
    Mat c3 = c1.cross(c2);
    Mat tvec = H.col(2);
    Mat R(3, 3, CV_64F);
    for (int i = 0; i < 3; i++){
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }

    double trazaR = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    double angle = acos( (abs(trazaR)-1)/2 ) * 180.0 / 3.14159265;

    return float(exp( -(angle-60)*(angle-60)/(2*24*24) )+exp( -(angle-120)*(angle-120)/(2*24*24) ));
}


float probabilityByArea(vector<Point2f> &pts, double &totalArea){
    // Corners Pattern
    double x1 = double(pts[ 0].x), y1 = double(pts[ 0].y);
    double x2 = double(pts[ 4].x), y2 = double(pts[ 4].y);
    double x3 = double(pts[15].x), y3 = double(pts[15].y);
    double x4 = double(pts[19].x), y4 = double(pts[19].y);

    double sum;
    sum  = (x1*y2 - x2*y1);
    sum += (x2*y3 - x3*y2);
    sum += (x3*y4 - x4*y2);

    double area = abs( sum/2 )/totalArea;

    return float( exp( -(area-0.1710)*(area-0.1710)/(2*0.0874*0.0874) ) );
}


float probabilityByPosition(vector<Point2f> &pts, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize){

    float stepRow = float(windowsize. width)/float(gridSize);
    float stepCol = float(windowsize.height)/float(gridSize);

    size_t row,col;

    // Check
    int minGrid=999999999, maxGrid = -1, sumGrid = 0;
    int value;
    size_t i,j,n;
    for(i=0; i<gridSize; ++i){
        for(j=0; j<gridSize; ++j){
            value = gridCloudPoints[i][j];
            if(value < minGrid) minGrid = value;
            if(value > maxGrid) maxGrid = value;
            sumGrid += value;
        }
    }

    // Update
    float prob = 0.0f;
    for (n=0; n<pts.size(); ++n){
        row = size_t(pts[n].x/stepRow);
        col = size_t(pts[n].y/stepCol);

        prob += gridCloudPoints[row][col];
    }
    prob = (float( maxGrid*int(pts.size()) ) - prob )/float( maxGrid*10*10 - sumGrid ); //3.0f*

    if (maxGrid == 0) prob = 1;
    if (prob <= 0.0f) prob = 1;
    return prob;
}


float probabilityByTime(int &timelapse){
    return 1.0f/( 1.0f + float(exp( -float(timelapse-30)/6.0f )));
}


void addPointsToCloudPoints(vector<Point2f> &pts, vector<vector<int>> &gridCloudPoints, size_t &gridSize, Size &windowsize){
    size_t row,col,n;

    float stepRow = float(windowsize. width)/float(gridSize);
    float stepCol = float(windowsize.height)/float(gridSize);

    for (n=0; n<pts.size(); ++n){
        row = size_t(pts[n].x/stepRow);
        col = size_t(pts[n].y/stepCol);

        ++gridCloudPoints[row][col];
    }
}
