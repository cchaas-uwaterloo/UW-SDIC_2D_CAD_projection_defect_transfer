# Beam Camera Image to 2D CAD Drawing Crack Transfer
## Overview 
This module provides a set of tools to transfer dimensional data for defects identified a camera image of a planar structure face to a corresponding orthographic CAD drawing of that structure surface. The camera image of the surface can be taken from any orientation, though an initial estimate of the camera pose relative to the structure surface, or the component transforms of this pose, must be provided. The accuracy of the transfer depenends on the accuracy of the inital camera pose estimate provided. 

### inputs
To transfer crack and other defect data, the structure surface outlines in both the camera image and CAD drawing must be provided to the module as labelled feature data in json format. An example labelled image file is provided in the config subdirectory of this project (example_input.json). This file was generated manually using the Slava labelling tool (https://github.com/Slava/label-tool). Only the outline of the surface corresponding to the CAD drawing should be outlined in the camera image. There is currently no utility in this module for automatically extracting these outlines from images and it must be done manually.

Currently, the module has only been tested using locally generated test crack data. In the future, crack and other defects will be identified in the camera image along with the structure outline and provided to the module in the same format. 

The module must also be provided with an initial estimate of the pose of the camera with respect to the structure surface when the image was taken. This can either be provided either as the complete transform between the structure and camera coordinate frames, or as a series of intermediate transform from which the overall structure-camera transform can be obtained. These can be provided in several ways: 
1. In the SolutionConfiguration.json file (euler angle - translation form) this is the default initial pose used by the pose estimation solver when none other is provided 
2. In a dedicated json file (euler angle - translation form) of the same form as the pose_example.json file provided in the config subdirectory of this project. Several separate files describing intermediate transforms can also be read in to describe the camera pose with respect to the surface
3. The structure - camera transform or its intermediate components can be hard coded in the main executable and fed into the pose estimation solver

### outputs 


