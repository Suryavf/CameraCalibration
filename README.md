# CameraCalibration

Computer vision techniques are widely studied in order to develop real time applications. Thus, to develop sophisticated applications one of the crucial stage is the camera calibration. However, this step has several challenges due to factors such as: illumination, brightness, contrast, noise, and low resolution. This repository proposes a processing method in order to detect a calibration pattern in a sequence video. The method was evaluated in three videos and the results show an accurate detection in the majority of the cases.

## Dependencies

Use [OpenCV](https://opencv.org/) 3.4

## Compilation

```bash
make
./out
```

## Usage

Open CameraCalibration/main.cpp

```cpp
int main(){

	// Read image
	string name_video = "Preprocessing/PadronAnillos_01.avi";
	find_rings(name_video); // Ellipse Method

    waitKey(0); 
    return 0;
}
```

or

```cpp
int main(){

	// Read image
	string name_video = "Preprocessing/PadronAnillos_01.avi";
	hough_transform_from_video(name_video); // Hough method

    waitKey(0); 
    return 0;
}
```