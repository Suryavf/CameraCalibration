#include "Preprocessing/preprocessing.h"

int main(){

	// Read image
	string name_video = "Preprocessing/PadronAnillos_03.avi";
	find_rings(name_video);

    waitKey(0); 
    return 0;
}
