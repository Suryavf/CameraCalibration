#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: camera_calibration [configuration_file -- default ./default.xml]"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, RINGS_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
                  << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << writePoints
                  << "Write_extrinsicParameters"   << writeExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        node["Fix_K1"] >> fixK1;
        node["Fix_K2"] >> fixK2;
        node["Fix_K3"] >> fixK3;
        node["Fix_K4"] >> fixK4;
        node["Fix_K5"] >> fixK5;

        validate();
    }
    void validate(){
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (isListOfImages(input) && readStringList(input, imageList))
                {
                    inputType = IMAGE_LIST;
                    nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Input does not exist: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
        if(fixK1)                  flag |= CALIB_FIX_K1;
        if(fixK2)                  flag |= CALIB_FIX_K2;
        if(fixK3)                  flag |= CALIB_FIX_K3;
        if(fixK4)                  flag |= CALIB_FIX_K4;
        if(fixK5)                  flag |= CALIB_FIX_K5;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
            if(fixK1)                   flag |= fisheye::CALIB_FIX_K1;
            if(fixK2)                   flag |= fisheye::CALIB_FIX_K2;
            if(fixK3)                   flag |= fisheye::CALIB_FIX_K3;
            if(fixK4)                   flag |= fisheye::CALIB_FIX_K4;
            if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (!patternToUse.compare("RINGS_GRID")) calibrationPattern = RINGS_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;

    }
    Mat nextImage(){
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < imageList.size() )
            result = imread(imageList[atImageList++], IMREAD_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l ){
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    static bool isListOfImages( const string& filename){
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient

    int cameraID;
    vector<string> imageList;
    size_t atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};



bool findRingsGrid( Mat &view, Mat &binarized, Mat &morphology, Mat &ellipses, Mat &result, RotatedRect &minRect, Size &boardSize, vector<Point2f> &puntosOriginales);
bool findRingsGrid2( Mat &view, Mat &binarized, Mat &morphology, Mat &ellipses, Mat &result, RotatedRect &minRect, Size &boardSize, vector<Point2f> &puntosOriginales);

vector<Point2f>getExtremePoints(Mat view, vector<Point2f> puntosOriginales, vector<Point2f> dst_vertices, int offset);

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );


vector<Mat> cameraMatriz_All; // Camera Coefficients
vector<float> error_All; // RMS error

int main(int argc, char* argv[]){
    help();

    //! [file_read]
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;

    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;
    bool press_key_capture = false;


    Mat cameraMatrix;
    Mat distCoeffs;

    // Reading the initial parameters for the calibration
    //Mat cameraMatrix;
    //Mat distCoeffs;

    {
        string filename = "out_camera_data.xml";
        FileStorage fs;
        fs.open(filename, FileStorage::READ);

        fs["camera_matrix"] >> cameraMatrix;                                   
        fs["distortion_coefficients"] >> distCoeffs;
        
    }

    cout << endl
            << "cameraMatrix = " << cameraMatrix << endl;
        cout << "distCoeffs = " << distCoeffs << endl << endl;

    // =============================== Variables in order to get the rings detection ========================================

    cv::Mat frame,binarized,morphology,ellipses,result;
    cv::RotatedRect minRect;


    Mat view;
    
    //view = s.nextImage();
    minRect = cv::RotatedRect(cv::Point(480,         0),
                              cv::Point(         0,         0),
                              cv::Point(         0,640));

    float timeLapse;
    double start_time;

     Mat cloud_points(480, 640, CV_8UC3, Scalar(0,0,0));

    // =======================================================================================================================



    // NUMBER OF ITERATIONS
    for (int ite = 0; ite < 50; ++ite)
    {
        cout<<"====== Iteracion " << ite << " ======"<<endl;
        s.atImageList = 0;
        mode = CAPTURING;
        imagePoints.clear();
        // code 

        //! [get_input]
        for(;;){
            Mat view;
            bool blinkOutput = false;
            view = s.nextImage();


            
            if(view.empty())          // If there are no more images stop the loop
            {
                // if calibration threshold was not reached yet, calibrate now
                
                if( mode != CALIBRATED && !imagePoints.empty() )
                    runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);

             
                break;
            }
            //! [get_input]

            imageSize = view.size();  // Format input image.
            if( s.flipVertical )    flip( view, view, 0 );

            //! [find_pattern]
            vector<Point2f> puntosOriginales;

            bool found;

            int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

            if(!s.useFisheye) {
                // fast check erroneously fails with high distortions like fisheye
                chessBoardFlags |= CALIB_CB_FAST_CHECK;
            }

            switch( s.calibrationPattern ) // Find feature points on the input format
            {
            case Settings::CHESSBOARD:
                found = findChessboardCorners( view, s.boardSize, puntosOriginales, chessBoardFlags);
                break;
            case Settings::CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, puntosOriginales );
                break;
            case Settings::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, puntosOriginales, CALIB_CB_ASYMMETRIC_GRID );
                break;
            case Settings::RINGS_GRID:
                found = findRingsGrid( view, binarized, morphology, ellipses, result, minRect, s.boardSize, puntosOriginales);
                break;
            default:
                found = false;
                break;
            }
            //! [find_pattern]
            //! [pattern_found]

            Mat imgCorregida;
            if(found && s.calibrationPattern== Settings::RINGS_GRID)
            {
                Mat view2 = view.clone();
                imgCorregida = view.clone();
                undistort(view, imgCorregida, cameraMatrix, distCoeffs);


                Mat imgCorregida2 = imgCorregida.clone();

                vector<Point2f> puntosOriginales_Corregidos;
                undistortPoints(puntosOriginales, puntosOriginales_Corregidos, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
                /*for(uint i=0; i<puntosOriginales_Corregidos.size(); i++)
                {
                    puntosOriginales_Corregidos[i].x = cameraMatrix.at<double>(0,2) + cameraMatrix.at<double>(0,0)*puntosOriginales_Corregidos[i].x;
                    puntosOriginales_Corregidos[i].y = cameraMatrix.at<double>(1,2) + cameraMatrix.at<double>(1,1)*puntosOriginales_Corregidos[i].y;
                    //circle(view, pointBuf[i], 3, Scalar(0,255,255), -1);
                }*/

                drawChessboardCorners( imgCorregida, s.boardSize, Mat(puntosOriginales_Corregidos), found );
            

                //---------Obteniendo Fronto-Parallel-------------
                float radioPeque = view.cols/24;
                vector<Point2f> dst_vertices;
                dst_vertices.push_back( Point(0, 0) );
                dst_vertices.push_back( Point(view.cols, 0) );
                dst_vertices.push_back( Point(0, view.rows) );
                dst_vertices.push_back( Point(view.cols, view.rows) );

                vector<Point2f> src_vertices = getExtremePoints(imgCorregida, puntosOriginales_Corregidos, dst_vertices, radioPeque*3);

                //drawChessboardCorners( view, s.boardSize, Mat(puntosOriginales), found );
                radioPeque = view.cols/30;

                //FRONTO-PARALLEL
                Mat H = findHomography(src_vertices, dst_vertices);
                //FRONTO-PARALLEL FOR THE IMAGE
                Mat img_frontoParallel;
                warpPerspective(imgCorregida2, img_frontoParallel, H, img_frontoParallel.size(), INTER_LINEAR, BORDER_CONSTANT); 
                Mat img_frontoParallel2 = img_frontoParallel.clone();

                vector<Point2f>new_vertices;

                //FRONTO-PARALLEL FOR THE POINTS
                perspectiveTransform( puntosOriginales_Corregidos, new_vertices, H);
                for (int i = 0; i < puntosOriginales_Corregidos.size(); ++i)
                {
                    circle(img_frontoParallel, new_vertices[i], 8, Scalar(100, 0, 100), -1, 8, 0);
                }


                Mat binarized2, morphology2, ellipses2, result2;
                vector<Point2f> puntosFrontoP;
                bool found2 = findRingsGrid2( img_frontoParallel2, binarized2, morphology2, ellipses2, result2, minRect, s.boardSize, puntosFrontoP);
                imshow("binarized", binarized2);
                imshow("morphology", morphology2);
                imshow("ellipses", ellipses2);
                
                //cout<<"Encontro: "<< found2 <<endl;
                if (found2)
                {
                    drawChessboardCorners( img_frontoParallel, s.boardSize, Mat(puntosFrontoP), found2 );
                }

                vector<Point2f> pointBof_result;
                if (found2)
                {
                    float x; 
                    float y;
                    for (int i = 0; i < puntosFrontoP.size(); ++i)
                    {
                        //x = (new_vertices[i].x + puntosFrontoP[i].x)/2;
                        //y = (new_vertices[i].y + puntosFrontoP[i].y)/2;

                        x = puntosFrontoP[i].x;
                        y = puntosFrontoP[i].y;

                        pointBof_result.push_back(Point2f(x,y));
                    }
                }
                else
                {
                    for (int i = 0; i < new_vertices.size(); ++i)
                    {
                        
                        pointBof_result.push_back(new_vertices[i]);
                    }
                }

                drawChessboardCorners( img_frontoParallel, s.boardSize, Mat(pointBof_result), found );
                imshow("FRONTO-PARALLEL IMAGE", img_frontoParallel);
                
                //return point to original image corregida
                perspectiveTransform( pointBof_result, pointBof_result, H.inv());                
                drawChessboardCorners( imgCorregida2, s.boardSize, Mat(pointBof_result), found );
                imshow("Retutn Imagen Corregida", imgCorregida2);



                //perspectiveTransform( pointBof_result, pointBof_result, H.inv());    
                //--------Regresando los puntos a la imagen con distorcion, osea a la capturada por la camara
                float xDist, yDist, x, y;
                float k1 = distCoeffs.at<double>(0,0);
                float k2 = distCoeffs.at<double>(0,1);
                //float p1 = distCoeffs.at<double>(0,2);
                //float p2 = distCoeffs.at<double>(0,3);
                float k3 = distCoeffs.at<double>(0,4);
                float cx = cameraMatrix.at<double>(0,2);
                float cy = cameraMatrix.at<double>(1,2);
                float fx = cameraMatrix.at<double>(0,0);
                float fy = cameraMatrix.at<double>(1,1);
                float r;

                for(uint i=0; i<pointBof_result.size(); i++)
                {

                    x = (pointBof_result[i].x-cx)/fx;
                    y = (pointBof_result[i].y-cy)/fy;
                    r = sqrt(x*x+y*y);

                    xDist = x*( 1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6) );
                    yDist = y*( 1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6) );

                    //xDist += x + (2*p1*y + p2*(pow(r,2) + 2*pow(x,2)) );
                    //yDist += y + ( p1*(pow(r,2) + 2*pow(y,2))  +  2*p2*y);

                    xDist = xDist * fx + cx;
                    yDist = yDist * fy + cy;

                    pointBof_result[i].x = xDist;
                    pointBof_result[i].y = yDist;
                }


                vector<Point2f> pointBof_result_Final;
                 
                
                for (int i = 0; i < pointBof_result.size(); ++i)
                {
                    float x = (pointBof_result[i].x + puntosOriginales[i].x)/2;
                    float y = (pointBof_result[i].y + puntosOriginales[i].y)/2;

                    //x = pointBof_result[i].x;
                    //y = pointBof_result[i].y;

                    pointBof_result_Final.push_back(Point2f(x,y));
                }

                drawChessboardCorners( view2, s.boardSize, Mat(pointBof_result_Final), found );
                imshow("Return Imagen Original", view2);

                puntosOriginales.clear();
                puntosOriginales = pointBof_result_Final;

            }

            // ============================================ FRAMES ====================================
            String filename = "Frames_Iterative/frame";
            String filename2 = "Frames_Color_Iterative/frame";
            
            bool color_frame = false;
            //Mat view_auxiliary;
            //view_auxiliary = view.clone();

            if ( found )                // If done with success,
            {
                  // improve the found corners' coordinate accuracy for chessboard
                    if( s.calibrationPattern == Settings::CHESSBOARD)
                    {
                        Mat viewGray;
                        cvtColor(view, viewGray, COLOR_BGR2GRAY);
                        cornerSubPix( viewGray, puntosOriginales, Size(11,11),
                            Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                    }

                    if( mode == CAPTURING &&  // For camera only take new samples after delay time
                        (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC))
                    {
                        imagePoints.push_back(puntosOriginales);
                        prevTimestamp = clock();
                        blinkOutput = s.inputCapture.isOpened();
                        imwrite(filename + to_string(imagePoints.size()) + ".png", view);
                        color_frame = true;

                    }

                    

                    // Draw the corners.
                    drawChessboardCorners( view, s.boardSize, Mat(puntosOriginales), found );

                    if( color_frame)
                    {
                        imwrite(filename2 + to_string(imagePoints.size()) + ".png", view);
                        press_key_capture = false;

                        for (int i = 0; i < puntosOriginales.size(); ++i)
                        {
                            circle(cloud_points, puntosOriginales[i], 2, Scalar(221, 255, 51), -1, 8, 0);
                        }

                    }
            }
            //! [pattern_found]
            //----------------------------- Output Text ------------------------------------------------
            //! [output_text]
            string msg = (mode == CAPTURING) ? "100/100" :
                          mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
            Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

            if( mode == CAPTURING )
            {
                if(s.showUndistorsed)
                    msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
                else
                    msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
            }

            putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

            if( blinkOutput )
                bitwise_not(view, view);
            //! [output_text]
           
            //! [output_undistorted]
            //------------------------------ Show image and check for input commands -------------------
            //! [await_input]
            
            imshow("Image cloud points", cloud_points);
            imshow("Imagen Original", view);
            imshow("Imagen Corregida", imgCorregida);
            

            if( (char)waitKey(1)  == ESC_KEY )
            {
                ite = 50;    
                break;
            }
            
            char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

            if( key  == ESC_KEY )
            {
                ite = 50;
                break;
            }

            if( key == 'u' && mode == CALIBRATED )
               s.showUndistorsed = !s.showUndistorsed;

            if( s.inputCapture.isOpened() && key == 'g' )
            {
                mode = CAPTURING;
                imagePoints.clear();
            }


            if(key == 'c' )
                press_key_capture = true;

            //! [await_input]
        }


        //PARAMETERS UPDATE
        cameraMatrix = cameraMatrix.clone();
        distCoeffs = distCoeffs.clone();
        

    }
    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;

        if (s.useFisheye)
        {
            Mat newCamMat;
            fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                                Matx33d::eye(), newCamMat, 1);
            fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                             CV_16SC2, map1, map2);
        }
        else
        {
            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                CV_16SC2, map1, map2);
        }

        for(size_t i = 0; i < s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], IMREAD_COLOR);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR); //show the image in the correct way(undistorted)
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye){
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/){
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
    case Settings::RINGS_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}
//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr){
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    }

    cout << "f = (" << cameraMatrix.at<double>(0,0) << "," << cameraMatrix.at<double>(1,1) << ")" << endl;
    cout << "c = (" << cameraMatrix.at<double>(0,2) << "," << cameraMatrix.at<double>(1,2) << ")" << endl;
    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

double colinealidad(vector<Point2f> tmp)
{
    Point2f p1=tmp[0];
    Point2f p2=tmp[tmp.size()-1];

    double m=(p2.y-p1.y)/(p2.x-p1.x);
    double yCal;


    double co=0.0;
    for(uint i=1;i<tmp.size()-1;i++)
    {
        yCal  = m*(tmp[i].x-p1.x) + p1.y;
        co += abs(yCal-tmp[i].y);
    }
    return co/(tmp.size()-2);

}

vector<double> colidadlidaList;
// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{

    double totalColinialidad = 0.0;

    
    for(uint i=0; i<imagePoints.size(); i++)
    {
        vector<Point2f> pointBuf;
        undistortPoints(imagePoints[i], pointBuf, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

        vector<Point2f> pointBufSuperior;
        for(uint j=0; j<5; j++)
        {
            pointBufSuperior.push_back(Point2f(pointBuf[j].x, pointBuf[j].y));
        }
        totalColinialidad += colinealidad(pointBufSuperior);
    }

    totalColinialidad /= imagePoints.size();
    colidadlidaList.push_back(totalColinialidad);


    cameraMatriz_All.push_back(cameraMatrix.clone());
    error_All.push_back(totalAvgErr);
    string nombreP= "resultRing.txt";


    fstream archivo(nombreP);
    for(uint i=0; i<cameraMatriz_All.size(); i++)
    {
        archivo << cameraMatriz_All[i].at<double>(0,0) <<"\t" << cameraMatriz_All[i].at<double>(1,1) << "\t";
        archivo << cameraMatriz_All[i].at<double>(0,2) <<"\t" << cameraMatriz_All[i].at<double>(1,2) << "\t";
        archivo << error_All[i] <<  "\t";
        archivo << colidadlidaList[i] <<  "\t" <<endl;
    }

    archivo.close();



    FileStorage fs( "out_camera_data2.xml", FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints){
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr);
    return ok;
}
//! [run_and_save]













void ellipsePurge(Mat &morphology, Mat &ellipses,
                  const std::vector<RotatedRect> &elipses,
                  vector<Point2f> &centers,
                  int width, int height){
    vector<int> ellipses_to_verify( elipses.size() );
    Point2f center;
    int counter_verify;
    float x_t0, y_t0;
    float x_t1, y_t1;
    float disX, disY;
    float sumx, sumy;

    int counter = 0;
    float holgura = 1;
    float distance;

    // Draw ellipse|
    ellipses = Mat::zeros( morphology.size(), CV_8UC3 );

    for( size_t i = 0; i< elipses.size()-1; i++ ){
        if( elipses[i].size.width > width/8 || elipses[i].size.height > height/6 ||
            elipses[i].size.width < 6       || elipses[i].size.height < 6        ||
            ellipses_to_verify[i] == -1){}
        else{
            x_t0 = elipses[i].center.x;
            y_t0 = elipses[i].center.y;

            counter_verify = 1;
            sumx = x_t0;
            sumy = y_t0;

            for( size_t j = i+1; j< elipses.size()-1; j++ ){
                x_t1 = float(elipses[j].center.x);
                y_t1 = float(elipses[j].center.y);

                disX = x_t0 - x_t1;
                disY = y_t0 - y_t1;
                distance = sqrt(disX*disX + disY*disY);

                if(distance <= holgura){
                    counter++;
                    counter_verify++;
                    sumx += x_t1;
                    sumy += y_t1;
                    ellipses_to_verify[j] = -1;
                }
            }

            if(counter_verify > 1){
                center = Point2f(sumx/counter_verify, sumy/counter_verify);
                circle(ellipses, center, 2,cv::Scalar(35,255,75), -1, 8, 0);
                ellipse( ellipses, elipses[i], Scalar(253,35,255), 2, 8 );

                // Add to ellipse list
                centers.push_back(center);
            }

        }
    }
}

void ellipsePurge2(Mat &morphology, Mat &ellipsesMat,
                  const std::vector<RotatedRect> &elipses,
                  vector<Point2f> &centers,
                  int width, int height)
{
    vector<int> ellipses_to_verify( elipses.size() );
    Point2f center;
    int counter_verify;
    float x_t0, y_t0;
    float x_t1, y_t1;
    float disX, disY;
    float sumx, sumy;

    int counter = 0;
    float holgura = 1;
    float distance;

    // Draw ellipse|
    
    for( size_t i = 0; i< elipses.size()-1; i++ )
    {
        if( elipses[i].size.width < width/12 || elipses[i].size.height < width/12 ||
            elipses[i].center.x == 0 || elipses[i].center.y == 0 ||
            elipses[i].size.width > width/2 || elipses[i].size.height>  height/2
            
            )
        {}
        else
        {
            x_t0 = elipses[i].center.x;
            y_t0 = elipses[i].center.y;

            counter_verify = 1;
            sumx = x_t0;
            sumy = y_t0;

            for( size_t j = i+1; j< elipses.size()-1; j++ ){
                x_t1 = float(elipses[j].center.x);
                y_t1 = float(elipses[j].center.y);

                disX = x_t0 - x_t1;
                disY = y_t0 - y_t1;
                distance = sqrt(disX*disX + disY*disY);

                if(distance <= holgura){
                    counter++;
                    counter_verify++;
                    sumx += x_t1;
                    sumy += y_t1;
                    ellipses_to_verify[j] = -1;
                }
            }

            if(counter_verify >= 1){
                center = Point2f(sumx/counter_verify, sumy/counter_verify);
                circle(ellipsesMat, center, 2,cv::Scalar(35,255,75), -1, 8, 0);
                ellipse( ellipsesMat, elipses[i], Scalar(253,35,255), 2, 8 );

                // Add to ellipse list
                centers.push_back(center);
            }
            //centers.push_back(Point2f(elipses[i].center.x, elipses[i].center.y));

        }
    }

}

bool DoesntRectangleContainPoint(RotatedRect &rectangle, Point2f &point) {
    //Get the corner points.
    Point2f corners[4];
    rectangle.points(corners);

    //Convert the point array to a vector.
    //https://stackoverflow.com/a/8777619/1997617
    Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    vector<Point2f> contour(corners, lastItemPointer);

    //Check if the point is within the rectangle.
    double indicator = pointPolygonTest(contour, point, false);
    return (indicator < 0);
}


vector<Point2f> get_intermediate_sorted_points(Point p, vector<Point2f> intermediate_vectors)
{
    vector<Point2f> result;
    if(intermediate_vectors.size() == 1)
    {
        result.push_back(intermediate_vectors[0]);
        return result;
    }

    if(intermediate_vectors.size() == 3 )
    {
        float d1 = sqrt(pow(p.x - intermediate_vectors[0].x,2) + pow(p.y - intermediate_vectors[0].y,2) );
        float d2 = sqrt(pow(p.x - intermediate_vectors[1].x,2) + pow(p.y - intermediate_vectors[1].y,2) );
        float d3 = sqrt(pow(p.x - intermediate_vectors[2].x,2) + pow(p.y - intermediate_vectors[2].y,2) );

        if(d1 < d2 && d1 < d3)
        {
            result.push_back(intermediate_vectors[0]);

            if(d2 < d3)
            {
                result.push_back(intermediate_vectors[1]);
                result.push_back(intermediate_vectors[2]);
            }

            else
            {
                result.push_back(intermediate_vectors[2]);
                result.push_back(intermediate_vectors[1]);
            }
        }

        if(d2 < d1 && d2 < d3)
        {
            result.push_back(intermediate_vectors[1]);

            if(d1 < d3)
            {
                result.push_back(intermediate_vectors[0]);
                result.push_back(intermediate_vectors[2]);
            }

            else
            {
                result.push_back(intermediate_vectors[2]);
                result.push_back(intermediate_vectors[0]);
            }
        }

        if(d3 < d1 && d3< d2)
        {
            result.push_back(intermediate_vectors[2]);

            if(d1 < d2)
            {
                result.push_back(intermediate_vectors[0]);
                result.push_back(intermediate_vectors[1]);
            }

            else
            {
                result.push_back(intermediate_vectors[1]);
                result.push_back(intermediate_vectors[0]);
            }
        }

    }

    if(intermediate_vectors.size() == 2)
    {
        float d1 = sqrt(pow(p.x - intermediate_vectors[0].x,2) + pow(p.y - intermediate_vectors[0].y,2) );
        float d2 = sqrt(pow(p.x - intermediate_vectors[1].x,2) + pow(p.y - intermediate_vectors[1].y,2) );

        if(d1 < d2)
        {
            result.push_back(intermediate_vectors[0]);
            result.push_back(intermediate_vectors[1]);
        }

        else
        {
            result.push_back(intermediate_vectors[1]);
            result.push_back(intermediate_vectors[0]);
        }

    }

    return result;

}
vector<Point2f> get_intermediate_points(Point p1, Point p2, vector<Point2f> good_ellipses, int holgura)
{
    vector<Point2f> intermediate_vectors;
    float m = (p2.y - p1.y)/(p2.x - p1.x + 0.0001);
    float y_eq, x_eq;

    

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if( !(
                (abs(good_ellipses[i].x-p1.x)<1 && abs(good_ellipses[i].y - p1.y)<1)
                || (abs(good_ellipses[i].x - p2.x)<1 && abs(good_ellipses[i].y - p2.y)<1)
                    ))
        {
            if(abs(good_ellipses[i].x - p1.x) > abs(good_ellipses[i].y - p1.y))
            {
                y_eq = m*(good_ellipses[i].x - p1.x) + p1.y;

                if(abs(y_eq - good_ellipses[i].y) <= holgura)
                {
                    intermediate_vectors.push_back(good_ellipses[i]);
                }

            }
            else
            {
                x_eq = (good_ellipses[i].y - p1.y)/m + p1.x;

                if(abs(x_eq - good_ellipses[i].x) <= holgura)
                {
                    intermediate_vectors.push_back(good_ellipses[i]);
                }

            }
        }
    }
    return intermediate_vectors;
}


vector<Point2f> ellipses_order20(vector<Point2f> good_ellipses, Mat ellipses, int origen)
{
    vector<Point2f> convex_hull;
    vector<Point2f> sorted_ellipses;
    float holgura = 5;
    

    convexHull(good_ellipses, convex_hull, true);
    convex_hull.push_back(convex_hull[0]);
    convex_hull.push_back(convex_hull[1]);

    Mat ellipses2 (480, 640, CV_8UC3, Scalar(0,0,0));
    if(origen==1)
    {
        holgura = 20;
        /*for (int i = 0; i < convex_hull.size()-2; ++i)
        {
            circle(ellipses2, convex_hull[i], 10,cv::Scalar(0,0,255), -1, 8, 0);
            putText(ellipses2,      std::to_string(i+1),
                              convex_hull[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                          0.65, // Scale. 2.0 = 2x bigger
                         cv::Scalar(35,255,75), // BGR Color
                                            1); // Line Thickness (Optional)
        }
        imshow("ellipsesOtro", ellipses2);*/
    }

    float x_s = convex_hull[0].x;
    float y_s = convex_hull[0].y;
    float m = (convex_hull[1].y - y_s)/(convex_hull[1].x - x_s + 0.0001);
    float y_eq = 0;
    float x_eq = 0;

    for(uint i=1; i<convex_hull.size(); i++)
    {
        if(abs(convex_hull[i].x - x_s) > abs(convex_hull[i].y - y_s))
        {
            y_eq = m*(convex_hull[i].x - x_s) + y_s;

            if(abs(y_eq - convex_hull[i].y) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }
        else
        {
            x_eq = (convex_hull[i].y - y_s)/m + x_s;

            if(abs(x_eq - convex_hull[i].x) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }


    }

    if(origen==1)
    {
        /*for (int i = 0; i < sorted_ellipses.size(); ++i)
        {
            circle(ellipses2, sorted_ellipses[i], 5,cv::Scalar(255,0,0), -1, 8, 0);
            putText(ellipses2,      std::to_string(i+1),
                              Point(sorted_ellipses[i].x-10, sorted_ellipses[i].y-10), // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                          0.65, // Scale. 2.0 = 2x bigger
                         cv::Scalar(255,255,75), // BGR Color
                                            1); // Line Thickness (Optional)
        }
        imshow("ellipsesOtro", ellipses2);*/
    }

    if(sorted_ellipses.size()!=4)
    {
        vector<Point2f> sorted_ellipses_final;
        return sorted_ellipses_final;
    }

    //cout<<sorted_ellipses.size()<<endl;
    x_s = sorted_ellipses[0].x;
    y_s = sorted_ellipses[0].y;
    m = (sorted_ellipses[1].y - y_s)/(sorted_ellipses[1].x - x_s + 0.0001);

    int count_dist1 = 0;
    int count_dist2 = 0;

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist1++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist1++;
            }

        }
    }




    x_s = sorted_ellipses[1].x;
    y_s = sorted_ellipses[1].y;
    m = (sorted_ellipses[2].y - y_s)/(sorted_ellipses[2].x - x_s + 0.0001);

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist2++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist2++;
            }

        }
    }


    if(count_dist1 < count_dist2)
    {
        Point p = sorted_ellipses[2];
        sorted_ellipses[2] = sorted_ellipses[3];
        sorted_ellipses[3] = p;
        //cout<<"FORMA AMPLIA"<<endl;
    }
    else
    {
        vector<Point2f> sorted_ellipses2;
        sorted_ellipses2.push_back(sorted_ellipses[3]);
        sorted_ellipses2.push_back(sorted_ellipses[0]);
        sorted_ellipses2.push_back(sorted_ellipses[2]);
        sorted_ellipses2.push_back(sorted_ellipses[1]);
        //cout<<"FORMA ALTA"<<endl;
        sorted_ellipses = sorted_ellipses2;

    }

    //============================================FUNCTION, HOW?============================================================
    vector<Point2f> sorted_ellipses_final(20);

    //Adding first row 0 to 4
    vector<Point2f> intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[1], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    sorted_ellipses_final[0] = sorted_ellipses[0];

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+1] = intermediate_vectors[i];
    }
    sorted_ellipses_final[4] = sorted_ellipses[1];




    //Adding first row 5, 10, 15
    intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[2], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[(i+1)*5] = intermediate_vectors[i];
    }
    sorted_ellipses_final[15] = sorted_ellipses[2];


    //Adding first row 16 to 19
    intermediate_vectors = get_intermediate_points(sorted_ellipses[2], sorted_ellipses[3], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[2], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+16] = intermediate_vectors[i];
    }
    sorted_ellipses_final[19] = sorted_ellipses[3];



    //Adding first row 9 , 14
    intermediate_vectors = get_intermediate_points(sorted_ellipses[1], sorted_ellipses[3], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[1], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[9+(i*5)] = intermediate_vectors[i];
    }

    //Adding first row 6 to 8
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[5], sorted_ellipses_final[9], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[5], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+6] = intermediate_vectors[i];
    }

    //Adding first row 11 to 13
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[10], sorted_ellipses_final[14], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[10], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+11] = intermediate_vectors[i];
    }





    //=========================================================================================================================

    //return good_ellipses;//sorted_ellipses_final;
    return sorted_ellipses_final;
}


// ================================= AUXILIO ==================================================================================

vector<Point2f> ellipses_order12(vector<Point2f> good_ellipses)
{
    vector<Point2f> convex_hull;
    vector<Point2f> sorted_ellipses;


    convexHull(good_ellipses, convex_hull, true); // Get corners
    convex_hull.push_back(convex_hull[0]);  // To ensure the square, searching the change to add the last corner
    convex_hull.push_back(convex_hull[1]);

    float x_s = convex_hull[0].x;
    float y_s = convex_hull[0].y;
    float m = (convex_hull[1].y - y_s)/(convex_hull[1].x - x_s + 0.0001); //Pendiente
    float holgura = 5; // In order to say the element is inside the line (recta)
    float y_eq = 0; // Equation with respect y
    float x_eq = 0; // Equation with respect x

    for(uint i=1; i<convex_hull.size(); i++)
    {
        if(abs(convex_hull[i].x - x_s) > abs(convex_hull[i].y - y_s)) // X mayor -> calculate the normal eq in function to y
        {
            y_eq = m*(convex_hull[i].x - x_s) + y_s;

            if(abs(y_eq - convex_hull[i].y) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }
        else
        {
            x_eq = (convex_hull[i].y - y_s)/m + x_s;

            if(abs(x_eq - convex_hull[i].x) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }


    }

    //cout<<sorted_ellipses.size()<<endl;
    x_s = sorted_ellipses[0].x;
    y_s = sorted_ellipses[0].y;
    m = (sorted_ellipses[1].y - y_s)/(sorted_ellipses[1].x - x_s + 0.0001);

    int count_dist1 = 0;
    int count_dist2 = 0;

    // Count the number of points from 0 to 1 position
    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist1++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist1++;
            }

        }
    }




    x_s = sorted_ellipses[1].x;
    y_s = sorted_ellipses[1].y;
    m = (sorted_ellipses[2].y - y_s)/(sorted_ellipses[2].x - x_s + 0.0001);

    // Count the number of points from 1 to 2 position
    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist2++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist2++;
            }

        }
    }

    // Sort the corners
    if(count_dist1 < count_dist2)
    {
        Point p = sorted_ellipses[2];
        sorted_ellipses[2] = sorted_ellipses[3];
        sorted_ellipses[3] = p;
        //cout<<"FORMA AMPLIA"<<endl;
    }
    else
    {
        vector<Point2f> sorted_ellipses2;
        sorted_ellipses2.push_back(sorted_ellipses[3]);
        sorted_ellipses2.push_back(sorted_ellipses[0]);
        sorted_ellipses2.push_back(sorted_ellipses[2]);
        sorted_ellipses2.push_back(sorted_ellipses[1]);
        //cout<<"FORMA ALTA"<<endl;
        sorted_ellipses = sorted_ellipses2;

    }

    //============================================FUNCTION, HOW?============================================================
    vector<Point2f> sorted_ellipses_final(12);

    //Adding first row 0 to 4
    vector<Point2f> intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[1], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    sorted_ellipses_final[0] = sorted_ellipses[0];

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+1] = intermediate_vectors[i];
    }
    sorted_ellipses_final[3] = sorted_ellipses[1];




    //Adding first row 4
    intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[2], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[4] = intermediate_vectors[0];
    }
    sorted_ellipses_final[8] = sorted_ellipses[2];


    //Adding first row 9, 10
    intermediate_vectors = get_intermediate_points(sorted_ellipses[2], sorted_ellipses[3], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[2], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+9] = intermediate_vectors[i];
    }
    sorted_ellipses_final[11] = sorted_ellipses[3];



    //Adding first row 7
    intermediate_vectors = get_intermediate_points(sorted_ellipses[1], sorted_ellipses[3], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[1], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[7] = intermediate_vectors[0];
    }

    //Adding first row 5, 6
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[4], sorted_ellipses_final[7], good_ellipses, holgura);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[4], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+5] = intermediate_vectors[i];
    }



    //=========================================================================================================================

    return sorted_ellipses_final;
}







void gridDetection(cv::Mat &frame     , cv::Mat  &binarized,
                   cv::Mat &morphology, cv::Mat  &ellipses ,
                   cv::Mat &result, cv::RotatedRect &minRec,
                   int &ellipseCount, vector<Point2f> &puntosOriginales){

    int width  = frame.cols;
    int height = frame.rows;
    vector<Vec4i> hierarchy;




/*
    Binarized Image
    ---------------
 */
    cv::Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    adaptiveThreshold(gray, binarized, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,15,3);

/*
    Morphological Transformations
    -----------------------------
 */
    cv::Mat element = getStructuringElement( MORPH_ELLIPSE, Size ( 7, 7 ),Point( 2, 2 ));
    erode ( binarized,  binarized, element );
    dilate( binarized, morphology, element );

/*
    Ellipse detection
    -----------------
 */
    std::vector<std::vector<cv::Point> > contours;
    findContours( morphology, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    std::vector<RotatedRect> minEllipse( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ ){
        if( contours[i].size() > 5 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

/*
    Ellipse purge
    -------------
 */
    vector<Point2f> good_ellipses;
    ellipsePurge(morphology,ellipses,minEllipse,
                    good_ellipses,width, height);
/*
    auto it = std::remove_if(good_ellipses.begin(),
                             good_ellipses.  end(),
                             [&minRec](Point2f &p){
                                return DoesntRectangleContainPoint(minRec,p);
                            });
    good_ellipses.erase(it,good_ellipses.end());*/

/*
    Update ROI
    ----------
 */
   if(good_ellipses.size() > 16){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width  = minRec.size.width +100;
        minRec.size.height = minRec.size.height+100;
    }
    else if(good_ellipses.size() > 10){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width  = minRec.size.width +200;
        minRec.size.height = minRec.size.height+200;
    }else
    {
        minRec = cv::RotatedRect(cv::Point(frame.cols,         0),
                                 cv::Point(         0,         0),
                                 cv::Point(         0,frame.rows));
    }


/*
    Result
    ------
 */
    vector<Point2f> sorted_ellipses = good_ellipses;

    if(good_ellipses.size() == 20)
        sorted_ellipses = ellipses_order20(good_ellipses, ellipses, 0);

    if(good_ellipses.size() == 12)
        sorted_ellipses = ellipses_order12(good_ellipses);


    frame.copyTo(result);
    for (size_t i = 0; i < sorted_ellipses.size(); ++i){
        circle(result, sorted_ellipses[i], 2, Scalar(253,35,255), -1, 8, 0); //-1 full circle
        putText(result,      std::to_string(i+1),
                              sorted_ellipses[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                          0.65, // Scale. 2.0 = 2x bigger
                         cv::Scalar(35,255,75), // BGR Color
                                            1); // Line Thickness (Optional)
    }

    if(sorted_ellipses.size() == 20 || sorted_ellipses.size() == 12)
        puntosOriginales = sorted_ellipses;

    // Save ellipse count
    ellipseCount = int(good_ellipses.size());

}

void gridDetection2(cv::Mat &frame     , cv::Mat  &binarized,
                   cv::Mat &morphology, cv::Mat  &ellipses ,
                   cv::Mat &result, cv::RotatedRect &minRec,
                   int &ellipseCount, vector<Point2f> &puntosOut){

    int width  = frame.cols;
    int height = frame.rows;
    vector<Vec4i> hierarchy;



/*
    Binarized Image
    ---------------
 */
    cv::Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    //blur( gray, gray, Size(5,5) );
    imshow("gray", gray);
    
    

    // Binary Threshold
    //threshold(gray, binarized, 128, 255, THRESH_BINARY);
 //   adaptiveThreshold(gray, binarized, 255, THRESH_BINARY, THRESH_BINARY,11,3);
    //adaptiveThreshold(binarized, binarized, 255, THRESH_BINARY, THRESH_BINARY,11,3);
    
    
    Mat canny;
    //threshold(gray, canny,127,255,THRESH_BINARY+THRESH_OTSU);
    adaptiveThreshold(gray, binarized, 255, THRESH_BINARY, THRESH_BINARY,15,3);
    //Canny( canny, binarized, 3, 100, 3 );

/*
    Morphological Transformations
    -----------------------------
 */
    cv::Mat element = getStructuringElement( MORPH_ELLIPSE, Size ( 7, 7 ),Point( 2, 2 ));
    erode ( binarized,  morphology, element );
    dilate( morphology, morphology, element );
    erode ( morphology,  morphology, element );
    
    //morphology = binarized.clone();
    
/*
    Ellipse detection
    -----------------
 */
    std::vector<std::vector<cv::Point> > contours;
    findContours( morphology, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    std::vector<RotatedRect> minEllipse( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ ){
        if( contours[i].size() > 5 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }
    
    
/*
    Ellipse purge
    -------------
 */
    ellipses = Mat::zeros( morphology.size(), CV_8UC3 );
    vector<Point2f> good_ellipses;
    ellipsePurge2(morphology,ellipses,minEllipse,
                    good_ellipses,width, height);

    


/*
    Result
    ------
 */
    vector<Point2f> sorted_ellipses = good_ellipses;

    if(good_ellipses.size() == 20)
        sorted_ellipses = ellipses_order20(good_ellipses, ellipses, 1);

    if(good_ellipses.size() == 12)
        sorted_ellipses = ellipses_order12(good_ellipses);


    //cout<<"Cant Elipse lista: " <<sorted_ellipses.size() <<endl;
    for (int i = 0; i < sorted_ellipses.size(); ++i)
    {
        circle(ellipses, sorted_ellipses[i], 10,cv::Scalar(35,255,75), -1, 8, 0);
    }
    //cout<<endl;



    
    /*frame.copyTo(result);
    for (size_t i = 0; i < sorted_ellipses.size(); ++i){
        circle(result, sorted_ellipses[i], 2, Scalar(253,35,255), -1, 8, 0); //-1 full circle
        putText(result,      std::to_string(i+1),
                              sorted_ellipses[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                          0.65, // Scale. 2.0 = 2x bigger
                         cv::Scalar(35,255,75), // BGR Color
                                            1); // Line Thickness (Optional)
    }
    imshow("orden",result);*/

    if(sorted_ellipses.size() == 20 || sorted_ellipses.size() == 12)
        puntosOut = sorted_ellipses;

    // Save ellipse count
    ellipseCount = int(puntosOut.size());

}

















// RINGS DETECTION
bool findRingsGrid( Mat &view, Mat &binarized, Mat &morphology, Mat &ellipses, Mat &result, RotatedRect &minRect, Size &boardSize, vector<Point2f> &puntosOriginales)
{
    int ellipseCount = 0;
    gridDetection(view, binarized, morphology, ellipses, result, minRect, ellipseCount, puntosOriginales);

    for(int i=0; i<puntosOriginales.size(); i++)
    {
        if(puntosOriginales[i].x == 0 && puntosOriginales[i].y == 0)
        {
            puntosOriginales.clear();
            return false;
        }
    }
    
    if(puntosOriginales.size() == 12 || puntosOriginales.size() == 20)
        return true;

    return false;

}

bool findRingsGrid2( Mat &view, Mat &binarized, Mat &morphology, Mat &ellipses, Mat &result, RotatedRect &minRect, Size &boardSize, vector<Point2f> &puntos)
{
    int ellipseCount = 0;
    gridDetection2(view, binarized, morphology, ellipses, result, minRect, ellipseCount, puntos);

    

    //cout<<"puntosOut1: "<<puntos.size()<<endl;
    for(int i=0; i<puntos.size(); i++)
    {
        if(puntos[i].x == 0 && puntos[i].y == 0)
        {
            puntos.clear();
            return false;
        }
    }

    //cout<<"puntosOut2: "<<puntos.size()<<endl;
    
    if(puntos.size() == 12 || puntos.size() == 20)
        return true;

    return false;

}

vector<Point2f>getExtremePoints(Mat view, vector<Point2f> puntosOriginales, vector<Point2f> dst_vertices, int offset)
{

    vector<Point2f> src_vertices;
    src_vertices.push_back( Point(puntosOriginales[15].x, puntosOriginales[15].y ) );
    src_vertices.push_back( Point(puntosOriginales[19].x, puntosOriginales[19].y ) );
    src_vertices.push_back( Point(puntosOriginales[0 ].x, puntosOriginales[0 ].y ) );
    src_vertices.push_back( Point(puntosOriginales[4 ].x, puntosOriginales[4 ].y ) );



    Mat H = findHomography(src_vertices, dst_vertices);
    //Matx33f H = getPerspectiveTransform(src_vertices, dst_vertices);

    cv::Mat rotated;
    warpPerspective(view, rotated, H, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);

    vector<Point2f> new_vertices;
    int width = view.cols, height = view.rows;
    new_vertices.push_back( Point(0-offset, 0-offset) );
    new_vertices.push_back( Point(width+offset, 0-offset) );
    new_vertices.push_back( Point(0-offset, height+offset) );
    new_vertices.push_back( Point(width+offset, height+offset) );


    perspectiveTransform( new_vertices, new_vertices, H.inv());


    //imshow("Fronto-Parallel-Corto", rotated);
    //waitKey(0);
    return new_vertices;
}
                                                                                                
