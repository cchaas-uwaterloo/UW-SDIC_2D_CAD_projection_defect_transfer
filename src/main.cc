#include <stdio.h>
#include <cstdint>
#include "imageReader.h"

using namespace cam_cad;


int main () {

    ImageReader imageReader;
    std::vector<point> input_points_camera, input_points_CAD; 
    bool read_success_camera = false, read_success_CAD = false; 

    read_success_camera = imageReader.readPoints("/home/cameron/cam_cad_proj/src/P210_north.json", &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");

    read_success_CAD = imageReader.readPoints("/home/cameron/cam_cad_proj/src/P210_north_crackmap.json", &input_points_CAD);

    if (read_success_camera) printf("CAD data read success\n");

    imageReader.densifyPoints(&input_points_camera, 20);

    imageReader.scalePoints(&input_points_CAD);

    return 0;
}

