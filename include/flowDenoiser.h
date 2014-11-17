/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  Tanis Mar
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/** 
\defgroup nearBlobber
 
@ingroup icub_module  
 
Uses disparity information to find the closest object.
 
Author: Tanis

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
Module which uses 3D coordinates and disparity information from to find the closest object. Returns the bounding box of its blob in the image
 
\section lib_sec Libraries 
- YARP libraries. 
- OPENCV library. 

\section parameters_sec Parameters
 
\section portsa_sec Ports Accessed
The module is assumed to work on the output of a disparity computation module, which generally requires iKinGazeCtrl
 
\section portsc_sec Ports Created 

Input ports
- \e /<modName>/disp:i receives a BRG image containing the disparity of the image obtained through stereo vision.
- \e /<modName>/rpc:i can be used to issue commands to the robot.
    Recognized remote commands:
        margin (int) - sets the margin (in pixels) that the ROI keeps around the closest blob.");
        thresh (int) (int)- sets higher and lower limit of disparity in terms of luminosity (0-255) that is considered. In other words, objects with luminosity outside boundaries wont be considered.;
        verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
        help - produces this help.;
        quit - closes the module.;
        

Output ports
- \e /<modName>/target:o streams out the coordinates (tl-br) of the bounding box around the closest blob
- \e /<modName>/img:o streams out a disparity image where the closest object is highlighted in green
- \e /<modName>/imgBin:o streams out a binary image containing in white only the closest detected blob
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 

\section conf_file_sec Configuration Files 
--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context).

  The configuration file passed through the option \e --from
should look like as follows:
 
\code 
name                nearBlobber
robot               icub
verbose             false
margin              20
backgroundThresh    50
frontThresh         200
cannyThresh         20
minBlobSize         400
gaussSize           5
dispThreshRatioLow  10
dispThreshRatioHigh 20
\endcode 

\section tested_os_sec Tested OS
Linux, Windows

\author Tanis Mar

*/

#ifndef __FLOWDENOISER_H__
#define __FLOWDENOISER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <time.h>
#include <string>
#include <iostream>
#include <queue>          

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

class FlowDenoiser : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:

    std::string name;                     // string containing module name
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imgFrameInPort;		// Receives disparity greyscale image --- Handled by the clas itself    
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imgProcInPort;		// Receives disparity greyscale image --- Handled by the clas itself    

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imOutPort;	        // output image Port with info drawn over
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >	imFiltOutPort;	        // output binary image of closest blob
        
    /* Pointer to the Resource Finder */
    yarp::os::ResourceFinder *moduleRF;

    /* Algorithm Variables */
    int buffSize;
    int numK;
    double smoothSize;
    double beta;
    bool verbose;
    bool discrete;
    bool warp;
  
    //cv::Mat frame;
    //cv::Mat imIn;

    std::deque<cv::Mat> bufferFrame;
    std::deque<cv::Mat> bufferFrameGray;
    std::deque<cv::Mat> bufferIm;

    cv::Mat imClusterLabels;
    cv::Mat kmLabels;

    bool bufferFull;

protected:
   
public:
    /**
     * constructor
     * @param moduleName is passed to the thread in order to initialise all the ports correctly
     */
    FlowDenoiser( const std::string &name, yarp::os::ResourceFinder &module );
    ~FlowDenoiser();

    bool setVerbose(std::string verb);
    bool setDiscrete(std::string disc);
    bool setWarp(std::string warpF);
    bool setBuffSize(int newBuffSize);
    bool setSmoothing(int newSmooth);
    bool setK(int newK);
    bool setAddWeight(double newAlpha);

    bool        open();
    void        close();
    void        onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &img );
    void        interrupt();
    
    yarp::os::Semaphore         mutex;          //semaphore for accessing/modifying within the callback
       
};

class FlowDenoiserModule:public yarp::os::RFModule
{
    /* module parameters */
    std::string             moduleName;
    std::string             handlerPortName;    
    yarp::os::RpcServer     rpcInPort;

    /* pointer to a new thread */
    FlowDenoiser            *denoiser;
    bool                    closing;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};


#endif
//empty line to make gcc happy
