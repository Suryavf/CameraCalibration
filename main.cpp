#include "Preprocessing/preprocessing.h"

int main(){

	// Read image
	string name_video = "Preprocessing/PadronAnillos_02.avi";
    //hough_transform_from_video(name_video);   // Hough method
	find_rings(name_video);                     // Ellipse Method

    waitKey(0); 
    return 0;
}
