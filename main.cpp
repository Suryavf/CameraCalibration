#include "Preprocessing/preprocessing.h"

int main(){

	// Read image
	string name_video = "Preprocessing/padron2.avi";
	find_rings(name_video);                     // Ellipse Method

    waitKey(0); 
    return 0;
}
