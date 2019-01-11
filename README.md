# CameraCalibration

Computer vision techniques are widely studied in order to develop real time applications. Thus, to develop sophisticated applications one of the crucial stage is the camera calibration. However, this step has several challenges due to factors such as: illumination, brightness, contrast, noise, and low resolution. This repository proposes a processing method in order to detect a calibration pattern in a sequence video. The method was evaluated in three videos and the results show an accurate detection in the majority of the cases.

## Dependencies

Use [OpenCV](https://opencv.org/) 3.4 and [QT](https://www.qt.io/) 5.11

## Compilation and execution all functions (circle, chessboard, rings)

Compile and execute:

```console
g++ -std=c++11 calibracion.cpp -o main `pkg-config --cflags --libs opencv`&& ./main
```

Change the pattern and settings in the file: default.xml

## Compilation and execution basic program

Compile CameraCalibration.pro in Qt 5.11.

## Usage

Execute program.


## Test

* **Grid 3x4** detection (click in image to play video)

[![Watch the video](https://i.imgur.com/ztK9luV.png)](https://youtu.be/w7SZ-9yJCts)


* **Grid 4x5** detection (click in image to play video)

[![Watch the video](https://i.imgur.com/BxgIfVo.png)](https://youtu.be/A9ZRA_AWvHI)


* **Grid 4x5 online detection** (click in image to play video)

[![Watch the video](https://i.imgur.com/s0n7vvA.png)](https://youtu.be/3CqVtkllsrk)

