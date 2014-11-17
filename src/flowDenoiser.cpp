/*
 * Copyright (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
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

#include "flowDenoiser.h"

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/**********************************************************/
bool FlowDenoiserModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("flowDenoiser"), "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();
    handlerPortName +=  "/rpc:i";

    if (!rpcInPort.open(handlerPortName.c_str()))    {
        fprintf(stdout, "%s : Unable to open input RPC port %s\n", getName().c_str(), handlerPortName.c_str());
        return false;
    }

    attach(rpcInPort);
    closing = false;

    /* create the thread and pass pointers to the module parameters */
    denoiser = new FlowDenoiser( moduleName, rf );

    /* now start the thread to do the work */
    denoiser->open();
    return true ;
}

/**********************************************************/
bool FlowDenoiserModule::interruptModule()
{
    closing = true;
    rpcInPort.interrupt();
    denoiser->interrupt();
    return true;
}

/**********************************************************/
bool FlowDenoiserModule::close()
{
    rpcInPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");   
    denoiser->close();
    fprintf(stdout, "deleting thread\n");
    delete denoiser;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool FlowDenoiserModule::updateModule()
{
    return !closing;
}

/**********************************************************/
bool FlowDenoiserModule::respond(const Bottle &command, Bottle &reply)
{
    /* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

    /* Get command string */
    string receivedCmd = command.get(0).asString().c_str();
    int responseCode;   //Will contain Vocab-encoded response
    if (receivedCmd == "buffSize"){
        bool ok = denoiser->setBuffSize(command.get(1).asInt());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"New buffer size couldnt be set.\n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;
    }else if (receivedCmd == "K"){
        bool ok = denoiser->setK(command.get(1).asInt());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"New number of clusters couldnt be set.\n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "addWeight"){
        bool ok = denoiser->setAddWeight(command.get(1).asDouble());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"New buffer size couldnt be set.\n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;
     
    }else if (receivedCmd == "smooth"){
        bool ok = denoiser->setSmoothing(command.get(1).asDouble());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"New gaussian smoothing couldnt be set.\n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "verbose"){
        bool ok = denoiser->setVerbose(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;
     }else if (receivedCmd == "warp"){
        bool ok = denoiser->setWarp(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Warp can only be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "discrete"){
        bool ok = denoiser->setDiscrete(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Discrete can only be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
        }
        reply.addVocab(responseCode);
        return true;

    }else if (receivedCmd == "help"){
        reply.addVocab(Vocab::encode("many"));
        responseCode = Vocab::encode("ack");
        reply.addString("Available commands are:");
        reply.addString("discrete ON/OFF  - Switches between the discrete output mode (for segmentation and labels) and the continue output mode (for gradual images).");
        reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
        reply.addString("warp ON/OFF - Turns ON/OFF optical flow warp. On off, it just uses the previous frames for flitering.");
        reply.addString("smooth (double) - Sets the size of the box filter for smoothing before optical flow computation. Size 0 just turns off smoothing.");
        reply.addString("buffSize (int) - Sets the size of the buffer of previous images used to compute an estimate based on the optical flow.");
        reply.addString("K (int) - Sets the number of klusters used to stabilize segmentation.");
        reply.addString("addWeight (double (0,1]) - Sets the value of the adding weight beta, which represents the relative importance of the past frame compared to most actual one. It decreases exponentially on each past frame.");
        reply.addString("help - produces this help.");
        reply.addString("quit - closes the module.");
        
        reply.addVocab(responseCode);
        return true;
    } else if (receivedCmd == "quit"){
        responseCode = Vocab::encode("ack");
        reply.addVocab(responseCode);
        closing = true;
        return true;
    }
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;
}

/**********************************************************/
double FlowDenoiserModule::getPeriod()
{
    return 0.1;
}

/**********************************************************/
FlowDenoiser::~FlowDenoiser()
{
    
}

/**********************************************************/
FlowDenoiser::FlowDenoiser( const string &moduleName, ResourceFinder &rf)
{    
    this->name = moduleName;
    this->moduleRF = &rf;
}

/**********************************************************/
bool FlowDenoiser::open()
{
    this->useCallback();

    // Initialize module
    fprintf(stdout,"Initialising Variables\n");
    bufferFull = false;
    //cout << "isStarting? " << bufferFull << endl;

    fprintf(stdout,"Parsing parameters\n");	

    verbose = moduleRF->check("verbose", Value(false)).asBool();
    discrete = moduleRF->check("discrete", Value(false)).asBool();
    warp = moduleRF->check("warp", Value(true)).asBool();
    buffSize = moduleRF->check("buffsize", Value(4)).asInt();
    numK = moduleRF->check("buffsize", Value(12)).asInt();
    beta = moduleRF->check("addWeight", Value(0.3)).asDouble();
    smoothSize = moduleRF->check("smooth", Value(0)).asDouble();

    //create all ports
    /* Inputs ports */
    bool ret=true;
    BufferedPort<ImageOf<PixelRgb>  >::open( ("/"+name+"/imgProc:i").c_str());        // port to receive the processed imaged to warp
    ret = imgFrameInPort.open(("/"+name+"/imgCam:i").c_str());                          // port to receive the ra wimage to copmute the optical flow
    /* Output ports */
    ret = ret && imOutPort.open(("/"+name+"/imgWarpCam:o").c_str());                      // port to receive the precessed image to warp
    ret = ret && imFiltOutPort.open(("/"+name+"/imgWarpProc:o").c_str());                 // port to receive the precessed image to warp

    warp = true;

    return ret;   
}

/**********************************************************/
void FlowDenoiser::close()
{
    fprintf(stdout,"now closing ports...\n");
    
    BufferedPort<ImageOf<PixelRgb>  >::close();

    imgFrameInPort.close();
    imOutPort.close();
    imFiltOutPort.close();
            
    fprintf(stdout,"finished closing input and output ports...\n");
}

/**********************************************************/
void FlowDenoiser::interrupt()
{	
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
   
    BufferedPort<ImageOf<PixelRgb>  >::interrupt();

    imgFrameInPort.interrupt();
    imOutPort.interrupt();
    imFiltOutPort.interrupt();
    
    fprintf(stdout,"finished interrupt ports\n");
}


bool FlowDenoiser::setVerbose(string verb)
{
    if (verb == "ON"){
        verbose = true;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    } else if (verb == "OFF"){
        verbose = false;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    }    
    return false;
}

bool FlowDenoiser::setWarp(string warpF)
{
    if (warpF == "ON"){
        warp = true;
        fprintf(stdout,"Warping is : %s\n", warpF.c_str());
        return true;
    } else if (warpF == "OFF"){
        warp = false;
        fprintf(stdout,"Warping is : %s\n", warpF.c_str());
        return true;
    }    
    return false;
}

bool FlowDenoiser::setDiscrete(string disc)
{
    if (disc == "ON"){
        discrete = true;
        fprintf(stdout,"Discrete Image mode turned : %s\n", disc.c_str());
        //fprintf(stdout,"This functionality is not implemented yet, im sorry\n");
        return true;
    } else if (disc == "OFF"){
        discrete = false;
        fprintf(stdout,"Discrete Image mode turned : %s\n", disc.c_str());
        return true;
    }    
    return false;
}

bool FlowDenoiser::setBuffSize(int newBuffSize)
{
    if (newBuffSize <= 10){        
        fprintf(stdout,"New size : %f\n", newBuffSize);
        this->buffSize = newBuffSize;
        return true;
    } else    { 
        this->buffSize = 10;
        cout << "Desired size is too big. Size set to 10" << endl;
        return true;
    }
}

bool FlowDenoiser::setSmoothing(int newSmooth)
{
    if (newSmooth <= 20){        
        fprintf(stdout,"New gaussian smoothing : %f\n", newSmooth);
        this->smoothSize = newSmooth;
        return true;
    } else    { 
        this->smoothSize = 20;
        cout << "Desired smoothing size is too big. Smoothign gaussian set 20" << endl;
        return true;
    }
}

bool FlowDenoiser::setK(int newK)
{
    if (newK <= 15){        
        fprintf(stdout,"New size : %f\n", newK);
        this->numK = numK;
        return true;
    } else    { 
        this->numK = 15;
        fprintf(stdout,"%f are too many, K set to 15.\n",newK);
        return true;
    }
}

bool FlowDenoiser::setAddWeight(double newWeight)
{
    if (newWeight > 1){        
        cout << "Alpha has to be between 0 and 1. set to 1" << endl;        
        this->beta = 1;
        return true;
    } else    { 
        this->beta = newWeight;
        fprintf(stdout,"New size : %f\n", newWeight);
        return true;
    }
}


/**********************************************************/
void FlowDenoiser::onRead(ImageOf<PixelRgb> &imageIn)
{
    mutex.wait();    

    //cout << "Processed image received "<< endl;

    /* Prepare output image for visualization */
    ImageOf<PixelRgb> &imageOut  = imOutPort.prepare();
    imageOut.resize(imageIn.width(), imageIn.height());	imageOut.zero();    //initalize the image
    Mat imOut((IplImage*) imageOut.getIplImage(),false);

    ImageOf<PixelRgb> &imageFiltOut  = imFiltOutPort.prepare();
    imageFiltOut.resize(imageIn.width(), imageIn.height());	imageFiltOut.zero();    //initalize the image
    Mat imFiltOut((IplImage*) imageFiltOut.getIplImage(),false);
    if(verbose)  {cout << "Output ports prepared "<< endl;}

    Mat frame, frameGray, imIn;
    /* Save the previous filtered and raw images and read the new ones */  
    if (!bufferFull){
        ImageOf<PixelRgb> *tmp = imgFrameInPort.read(false);  // read an image from the cameras

        if(tmp!=NULL)
        {
            Mat framePort = (IplImage*) tmp->getIplImage();
            framePort.copyTo(frame);
            bufferFrame.push_front(frame);
                        
            cvtColor(frame, frameGray, CV_RGB2GRAY);   
            bufferFrameGray.push_front(frameGray);
           
            Mat imInPort = cvarrToMat(imageIn.getIplImage(),false);    // Format the processed input image to Mat
            imInPort.copyTo(imIn);
            bufferIm.push_front(imIn);
            
            if(verbose)  {cout << "Buffer size = " << bufferIm.size() << ". Filling buffer... "<< endl;}
            if (bufferIm.size() >= buffSize){            

                /*Compute K-means to initialize clusters*/                                 
                int origRows = imIn.rows;
                Mat colVec = imIn.reshape(1, imIn.rows*imIn.cols); // change to a Nx3 column vector
                Mat colVecD, centers;
                colVec.convertTo(colVecD, CV_32FC3, 1.0/255.0); // convert to floating point
                double compactness = kmeans(colVecD, numK, kmLabels, 
                    TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 5, 0.001), 1, KMEANS_PP_CENTERS, centers);               
                imClusterLabels = kmLabels.reshape(1, origRows); // single channel image of labels

                bufferFull  = true;
                if(verbose)  {cout << "Buffer full " << endl;}
            }
        }
    } else    {
        if(verbose)  {cout << " Reading next frame"<< endl;}
        /*Read incoming images*/
        ImageOf<PixelRgb> *tmp = imgFrameInPort.read(false);  // read an image
        if(tmp==NULL){
            if(verbose)  {cout << " no camera image, skipping frame"<< endl;}
        }else  {
            Mat framePort  = (IplImage*) tmp->getIplImage();
            framePort.copyTo(frame);            
            
            cvtColor(frame, frameGray, CV_RGB2GRAY);
            Mat imInPort = cvarrToMat(imageIn.getIplImage(),false);    // Create the src image form the received frame	
            imInPort.copyTo(imIn);

            if (smoothSize>0){
                if(verbose)  {cout << "Smoothing image"<< endl;}
                boxFilter(frameGray, frameGray, -1, cv::Size(smoothSize,smoothSize));  
            }
        
            /* Get estimate of actual frame form warping previous frames by means of the optical flow */
            if(verbose)  {cout << "Compute Optical from " << bufferFrameGray.size() << " previous frames." << endl;}
            
            deque<Mat> poolImWarp; poolImWarp.clear();
            Mat denoisedFrame;
            frame.copyTo(denoisedFrame);
            Mat denoisedIm;
            imIn.copyTo(denoisedIm);
            double beta_fr = beta;        // intialize the weight of the last frame wrt the actual one
            for (int f = 0; f < bufferFrameGray.size() ; f++)
            {            

                Mat warpedFrame;
                Mat warpedFiltIm;
                if (warp){
                    /* Compute optical flow*/
                    if(verbose)  {cout << "Computing Optical backflow on frame -" << f+1  << endl;}
                    Mat backFlow;           // backward flow
                    if(!backFlow.empty())
                        backFlow.setTo(Scalar(0));                
                    calcOpticalFlowFarneback(frameGray, bufferFrameGray[f], backFlow, 0.5, 5, 9, 5, 7, 1.5, 0 );    // computes optical flow from actual to previous(f) frame, hence backFlow     

                    /* Compute flow-based warp mapping*/
                    if(verbose)  {cout << "Mapping frame and Im -" << f+1 << " using optical flow" << endl;}
                
                    //cout << "Computing flow Mapping"<< endl;
                    Mat map(backFlow.size(), CV_32FC2);
                    for (int y = 0; y < map.rows; ++y)
                    {
                        for (int x = 0; x < map.cols; ++x)
                        {
                            Point2f f = backFlow.at<Point2f>(y, x);
                            map.at<Point2f>(y, x) = Point2f(x + f.x, y + f.y);
                        }
                    }
            
                    /*Warping images */            
                    remap(bufferFrame[f], warpedFrame, map, Mat(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0));
                    remap(bufferIm[f], warpedFiltIm, map, Mat(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0));

                }else{
                    warpedFiltIm = bufferIm[f];
                    warpedFrame = bufferFrame[f];
                }

                /* Compute a weigthed average of the warped frames*/
                double alpha = ( 1.0 - beta_fr );
                if(verbose)  {cout << "Adding warped processed frame -" << f+1 << " with alpha " << alpha << " and beta " << beta_fr << endl;}
                addWeighted( denoisedFrame, alpha, warpedFrame , beta_fr, 0.0, denoisedFrame);            
            
                //stringstream fss;        fss << f+1;         string fs = fss.str();
                //imshow(("Warped Frame -" + fs), warpedFrame);            

                if (!discrete){
                    /* Compute a weigthed average of the warped images for real value images*/
                    //poolImWarp.push_back(warpedFiltIm);         // Push warp f of non-discrete processed image into buffer

                    /*Perform a weight average of the warped non-discrete image*/                
                    if(verbose)  {cout << "Adding warped processed image -" << f+1 << " with weight " << beta_fr << endl;}
                    addWeighted( denoisedIm, alpha, warpedFiltIm , beta_fr, 0.0, denoisedIm);                
                }else{
                    /* Perform pixel wise voting for labelled images. K means necessary for changing labels*/
                    /*Compute K-means to initialize clusters*/
                    if(verbose)  {cout << "Perform k-means of warped frame -" << f+1 << endl;}
               
                    int origRows = warpedFiltIm.rows;
                    Mat colVec = warpedFiltIm.reshape(1, warpedFiltIm.rows*warpedFiltIm.cols); // change to a Nx3 column vector
                    Mat colVecD, centers;
                    colVec.convertTo(colVecD, CV_32FC3, 1.0/255.0); // convert to floating point
                    double compactness = kmeans(colVecD, numK, kmLabels, 
                            TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 5, 0.001), 1, KMEANS_USE_INITIAL_LABELS);// , centers);
                    imClusterLabels = kmLabels.reshape(1, origRows); // single channel image of labels

                    //Mat clustVis;       // Saves a normalized version of the cluster labesl, for visualization.
                    //clustVis = imClusterLabels* (256/numK);                
                    //clustVis.convertTo(clustVis,CV_8U);
                    //stringstream fss;        fss << f+1;         string fs = fss.str();
                    //imshow(("K-Means on  Frame -" + fs), clustVis);

                    poolImWarp.push_back(imClusterLabels);  // We puch back so that the first pushed im/frames (f= 0,1,..) will keep their order

                    /* Pixel wise mode of the K-means images is computed outside the loop*/
                }
                beta_fr = beta_fr * beta;
            }        
            //waitKey(1.0);

            /* Compute pixel wise mode of to select the most likely blob for each pixel */
            if (discrete){  // Compute Pixel wise mode of images            
                if(verbose)  {cout << "Computing mode of  = " << poolImWarp.size() << "warped, k-meaned images"<< endl;}
                Mat mode = Mat::zeros(imIn.rows, imIn.cols, CV_8UC1);
                for(int i=0;i<mode.rows;i++)
                {
                    for(int j=0;j<mode.cols;j++)
                    {
                        vector<int> count(numK,0);
                        int maxIndex=0, maxCount=0;
                        int index;
                        for(int n=0;n<poolImWarp.size();n++)
                        {                        
                            index = poolImWarp[n].at<uchar>(i,j,0);
                            count[index]++;
                            if(count[index] > maxCount)
                            {
                                maxCount = count[index];
                                maxIndex = index;
                            }                        
                        }
                        mode.at<uchar>(i,j) = maxIndex;
                    }
                }           
                Mat modeNorm;
                // Normalized version of the mode, for visualization.
                modeNorm = mode* (256/numK);  
                cvtColor(modeNorm, denoisedIm, CV_GRAY2RGB);
                //modeNorm.convertTo(modeNorm,CV_8UC1);
                //imshow(("Pooled labelled image "), modeNorm);
                //waitKey(1.0);
            }

            if(verbose)  {cout << "Push back one frame"<< endl;}
            while (bufferIm.size() >= buffSize)
            {
                bufferFrame.pop_back();
                bufferFrameGray.pop_back();
                bufferIm.pop_back();
            }

            bufferFrame.push_front(frame);
            bufferFrameGray.push_front(frameGray);
            bufferIm.push_front(imIn);

            if(verbose)  {cout << "Writing out images"<< endl;}
            denoisedFrame.copyTo(imOut);
            denoisedIm.copyTo(imFiltOut);

            /* write info on output ports */
            imOutPort.write();
            imFiltOutPort.write();
        }
    }

    mutex.post();
}



//empty line to make gcc happy

