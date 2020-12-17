# Beam Camera Image to 2D CAD Drawing Crack Transfer
## Overview 
This module provides a set of tools to transfer dimensional data for defects identified a camera image of a planar structure face to a corresponding orthographic CAD drawing of that structure surface. The camera image of the surface can be taken from any orientation, though an initial estimate of the camera pose relative to the structure surface, or the component transforms of this pose, must be provided. The accuracy of the transfer depenends on the accuracy of the inital camera pose estimate provided. 

### inputs
To transfer crack and other defect data, the structure surface outlines in both the camera image and CAD drawing must be provided to the module as labelled feature data in json format. An example labelled image file is provided in the config subdirectory of this project (example_input.json). This file was generated manually using the Slava labelling tool (https://github.com/Slava/label-tool). Only the outline of the surface corresponding to the CAD drawing should be outlined in the camera image. There is currently no utility in this module for automatically extracting these outlines from images and it must be done manually. An example labelled image is shown below.

![Alt text](/readme_images/labelled_sim_image.png?raw=true "Labelled Image")

Currently, the module has only been tested using locally generated test crack data. In the future, crack and other defects will be identified in the camera image along with the structure outline and provided to the module in the same format. 

The module must also be provided with an initial estimate of the pose of the camera with respect to the structure surface when the image was taken. This can either be provided either as the complete transform between the structure and camera coordinate frames, or as a series of intermediate transform from which the overall structure-camera transform can be obtained. These can be provided in several ways: 
1. In the SolutionConfiguration.json file (euler angle - translation form) this is the default initial pose used by the pose estimation solver when none other is provided 
2. In a dedicated json file (euler angle - translation form) of the same form as the pose_example.json file provided in the config subdirectory of this project. Several separate files describing intermediate transforms can also be read in to describe the camera pose with respect to the surface
3. The structure - camera transform or its intermediate components can be hard coded in the main executable and fed into the pose estimation solver

### outputs 

The module generates an annotated CAD drawing with the transfered defect outlines overlayed over the original drawing. Supported ouput formats are those of the openCV imwrite() function. An example output is shown below. 

![Alt text](/readme_images/sim_CAD_annotated.jpg?raw=true "Annotated CAD")

### how it works
To accurately transfer outlines of defects from a camera image taken at an arbitrary position to an orthographic CAD drawing, the module must determine the position relative to the structure from which an image was taken. To do this, the module builds an optimization problem to try to determine the position of the camera that minimizes the error between the projection of a point cloud representing the structure surface into the camera frame, and the outline of the surface in the image. If the optimization can converge to a solution for the camera pose (this depends highly on the initial pose estimate provided to the solution), the labelled defect pixels in the camera frame are back-projected onto the structure surface plane. The inverted camera pose transform is then applied to the defect points to return them to the CAD drawing frame. The defect points are then converted back into pixel values, now distorted by the back projection and transformation, and overlayed on the CAD drawing to create an annotated drawing showing the dimensionally accurate defect outlines. The complete workflow is detailed below.

![Alt text](/readme_images/2DCAD_workflow.png?raw=true "CAD workflow")

The pose estimation solver workflow:

![Alt text](/readme_images/solver_workflow.png?raw=true "CAD workflow")

## Instructions for use
### building and running 
The module is implemented as a catkin package. To build, the source files must be added to src directory of the catkin workspace and build using the catkin build command. 

Currently only test programs are generated. To run most of these, the following files must be provided and the relevant paths set in the test source files: 
1. camera image labels with structure outline points (json file, see Config/example_input.json for formatting requirements)
2. CAD drawing labels with structure outline points (json file, see Config/example_input.json for formatting requirements)
3. Pose estimation solution parameters (json file, see Config/SolutionParameters.json)
4. Camera model (see Config/Radtan_test.json or Config/ladybug.conf)
5. (optional) transforms describing initial camera pose for estimation (json file, see Config/pose_example.json for formatting requirements *

*If this is not provided and configured, the pose estimation will use the default initial pose estimate in the SolutionParameters file

Once configured, the test executables can be run from the devel directory of the catkin workspace. 

### visualizer
Visualization of the pose estimation is built into the solver and can be enabled or disabled at runtime by setting the "visualize" parameter of the SolutionConfiguration file. If visualization is enabled, the solution must be stepped forward between Ceres solutions by entering 'n' in the console running the program. 

In some test programs other visualization instances are used. Entering 'r' in the console will progress past these. 

To note is that the visualizer runs in its own thread and does not itself block the exectution of calling code. Any pause in the existing code when a visualization is displayed is implemented in the code calling the visualizer. 

## Next steps 
### Further development
For further development of this module the following next steps could be taken: 
1. Create/Find tool to convert pdfs to jpgs/pngs 
- image labelling and writing cannot be done directly on pdfs 
- a tool to convert pdfs to jpg or png format and vice versa is necessary to work with actual CAD drawings
- online tools might work (?)
2. Create interface to read in defect label points
- currently only locally generated defect outlines have been used to test the back project and write functions
- the ImageReader class should be expanded to read in defect label points from the camera outline label json files and convert them to point clouds
- consistent label naming should help to separate outline and defect label points 
3. Make pose estimation more robust for poor initial pose estimates
- pose estimation solutions fail to converge if the initial estimate is poor 
- automatically run solver multiple times with initial poses around the estimate (if evenly distributed, one should be closer to the real pose and might converge)
- run solver with different max ceres iterations (this can affect overall convergence for poor initializations)
4. Change Densify function to add points to input point sets at regular intervals 
- input label point sets are desified by interpolating points to help pose estimation solution converge 
- currently points are added at intervals that are a fraction of the distance between the original points
- this affects the pose estimation because some regions of the structure outline clouds are more dense and are weighted more heavily in the Ceres solver
- points should be added to the original point sets at regular intervals 
5. Automatically find and label structure onlines and defects in camera images and CAD drawings (likely a separate project)

There are also several minor changes to the source code identified with @todo tags that could (probably should) be made. 

### bugs
- visualizer hangs when ended in program until the entire program stops 
- many, many more that I haven't realized

