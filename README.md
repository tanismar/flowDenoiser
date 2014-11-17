flowDenoiser
============
This module uses performs denoising on the last frame by using information from previous frames. It takes as input any processed image
    to be denoised and the camera image. It works by computing the optical flow between the last the camera frame and the N previous ones, 
    and computing an estimate of the latest processed image frame by warping previous frames with the obtained optical flow. 
    It has two main working modes, non discrete, on which a weighted average of the estimates is computed, and discrete, thought to stabilize 
    labelled image which cant be averaged. In the discrete mode, K-means is performed to get consistent labelling, and then the stabilized image 
    is computed as the pixel-wise statistical mode among the considered previous frames.
    
    
