 Projection convergence ideas: 
 - generate set of probable initial poses (in matrix or euler angle and translation form) for different cameras given the likely geometry of the pillars 
    - cycle through these probable initials to see if one of them converges below an overall error threshold 
 - relabel images without the bottom of the pillar (in both image and CAD), this is sometimes clipped from the camera image and falsely identifying the bottom can mess with the convergence
 - try to converge image taken from one of the side ladybug cameras (easier intial pose estimation)
 - try to generate 3-d cloud from both orthogonal views of the structure in the cad drawing for projection/reprojection (difficult)
 - get correct ladybug camera id for each image
 - densify CAD points more evenly