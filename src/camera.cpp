#include "camera.h"

#include <chrono>
using namespace std::chrono;



CameraWrapper::~CameraWrapper() {
    pCam_ = nullptr;
}

CameraWrapper::CameraWrapper(CameraPtr pCam, bool color, StereoGigE *gige) : gige_(gige) {

    pCam_ = pCam;

    if (pCam_->IsInitialized()) {
        ROS_WARN_STREAM("Camera already initialized. Deinitializing...");
        pCam_->EndAcquisition();
        pCam_->DeInit();
    }

    lastFrameID_ = -1;
    frameID_ = -1;
    MASTER_ = false;
    COLOR_=color;

    pic_ = new PIcontroller(COLOR_);
}

void CameraWrapper::init() {

    pCam_->Init();
}

void CameraWrapper::deinit() {

    pCam_->DeInit();

}

void CameraWrapper::begin_acquisition() {
    ROS_DEBUG_STREAM("Begin Acquisition...");

    pCam_->BeginAcquisition();
}

void CameraWrapper::end_acquisition() {
    if (pCam_->GetNumImagesInUse())
        ROS_WARN_STREAM("Some images still currently in use! Use image->Release() before deinitializing.");
        
    ROS_DEBUG_STREAM("End Acquisition...");
    pCam_->EndAcquisition();

}

void CameraWrapper::setEnumValue(string setting, string value) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (enum retrieval). Aborting...");

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (entry retrieval). Aborting...");
		
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
		
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    ROS_DEBUG_STREAM(setting << " set to " << value);
    
}

void CameraWrapper::setIntValue(string setting, int val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void CameraWrapper::setFloatValue(string setting, float val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void CameraWrapper::setBoolValue(string setting, bool val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void CameraWrapper::setResolutionPixels(int width, int height) {
    CIntegerPtr ptrHeight=pCam_->GetNodeMap().GetNode("Height");
    CIntegerPtr ptrWidth=pCam_->GetNodeMap().GetNode("Width");
    if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth)){
        ROS_FATAL_STREAM("Unable to set width" << "). Aborting...");
        return ; 
    }
    int64_t widthMax = ptrWidth->GetMax();
    if(widthMax<width)
        width=widthMax;
    ptrWidth->SetValue(width);
    ROS_DEBUG_STREAM("Set Width"<<width);

    if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight)){
        ROS_FATAL_STREAM("Unable to set height" << "). Aborting...");
        return ; 
    }
    int64_t heightMax = ptrHeight->GetMax();
    if(heightMax<height)
        height=heightMax;

    ROS_DEBUG_STREAM("Set Height"<<height);
    ptrHeight->SetValue(height);                                                                                                                                 
}

void CameraWrapper::adcBitDepth(gcstring bitDep) {
    CEnumerationPtr ptrADC = pCam_->GetNodeMap().GetNode("AdcBitDepth");
    if (!IsAvailable(ptrADC) || !IsWritable(ptrADC)){
        ROS_FATAL_STREAM("Unable to set ADC Bit " <<  "). Aborting...");
        return ;
    }

    CEnumEntryPtr ptrADCA = ptrADC->GetEntryByName(bitDep);
    int currDepth=ptrADCA->GetValue();
    ptrADC->SetIntValue(currDepth);
    ROS_DEBUG_STREAM("BPS: "<<ptrADC->GetIntValue());

}

void CameraWrapper::setBufferSize(int numBuf) {
    INodeMap & sNodeMap = pCam_->GetTLStreamNodeMap();
    CIntegerPtr StreamNode = sNodeMap.GetNode("StreamDefaultBufferCount");
    int64_t bufferCount = StreamNode->GetValue();
    if (!IsAvailable(StreamNode) || !IsWritable(StreamNode)){
        ROS_FATAL_STREAM("Unable to set StreamMode " << "). Aborting...");
        return;
    }
    StreamNode->SetValue(numBuf);
    ROS_DEBUG_STREAM("Set Buf "<<numBuf<<endl);
}

void CameraWrapper::setISPEnable() {
    CBooleanPtr ptrISPEn=pCam_->GetNodeMap().GetNode("IspEnable");
    if (!IsAvailable(ptrISPEn) || !IsWritable(ptrISPEn)){
        ROS_FATAL_STREAM("Unable to set ISP Enable (node retrieval; camera " << "). Aborting...");
        return;
    }
    ptrISPEn->SetValue("True");
}

void CameraWrapper::setFREnable() {
    CBooleanPtr ptrAcquisitionFrameRateEnable=pCam_->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(ptrAcquisitionFrameRateEnable) || !IsWritable(ptrAcquisitionFrameRateEnable)){
        ROS_FATAL_STREAM("Unable to set frameRateEnable (node retrieval; camera " << "). Aborting...");
        return;
    }

    ptrAcquisitionFrameRateEnable->SetValue("True");
}

void CameraWrapper::setPixelFormat(gcstring formatPic) {
    CEnumerationPtr ptrPixelFormat = pCam_->GetNodeMap().GetNode(formatPic);
    if ( !IsWritable(ptrPixelFormat)){
        ROS_FATAL_STREAM("Unable to set Pixel Format " <<  "). Aborting...");
        return ;
    }
    CEnumEntryPtr ptrPixelEnt = ptrPixelFormat->GetEntryByName("RGB8Packed");
    if (!IsAvailable(ptrPixelEnt) || !IsReadable(ptrPixelEnt)){
        ROS_FATAL_STREAM("Unable to set RGBPoint"  << "). Aborting...");
        return ;
    }                                                                                                                                        
    int64_t colorNum = ptrPixelEnt->GetValue();
                                                                                                                                                
    ptrPixelFormat->SetIntValue(colorNum);
    ROS_DEBUG_STREAM( "Camera " << " set pixel format");
}

double CameraWrapper::getFloatValueMax(string node_string) {
    INodeMap& nodeMap = pCam_->GetNodeMap();

    CFloatPtr ptrNodeValue = nodeMap.GetNode(node_string.c_str());

    if (IsAvailable(ptrNodeValue)){
      //cout << "\tMax " << ptrNodeValue->GetValue() << "..." << endl;
      return ptrNodeValue->GetMax();
    } else {
        ROS_FATAL_STREAM("Node " << node_string << " not available" << endl);
        return -1;
    }
}

string CameraWrapper::getTLNodeStringValue(string node_string) {
    INodeMap& nodeMap = pCam_->GetTLDeviceNodeMap();
    CStringPtr ptrNodeValue = nodeMap.GetNode(node_string.c_str());
    if (IsReadable(ptrNodeValue)){
        return string(ptrNodeValue->GetValue());
    } else{
        ROS_FATAL_STREAM("Node " << node_string << " not readable" << endl);
        return "";
    }
}

string CameraWrapper::get_id() {
    return getTLNodeStringValue("DeviceSerialNumber");
}

int CameraWrapper::get_frame_id() {
    return frameID_;
}

bool CameraWrapper::verifyBinning(int binningDesired) {
    int actualBinningX =  (pCam_ ->SensorWidth())/(pCam_ ->Width());
    int actualBinningY =  (pCam_ ->SensorHeight())/(pCam_ ->Height());
    if (binningDesired == actualBinningX) return true;
    else return false;
}

void CameraWrapper::calibrationParamsTest(int calibrationWidth, int calibrationHeight) {
    if ( (pCam_ ->SensorWidth()) != calibrationWidth )
        ROS_WARN_STREAM(" Looks like your calibration is not done at full Sensor Resolution for cam_id = "<<get_id()<<" , Sensor_Width = "<<(pCam_ ->SensorWidth()) <<" given cameraInfo params:width = "<<calibrationWidth);
    if ( (pCam_ ->SensorHeight()) != calibrationHeight )
        ROS_WARN_STREAM(" Looks like your calibration is not done at full Sensor Resolution for cam_id = "<<get_id()<<" , Sensor_Height= "<<(pCam_ ->SensorHeight()) <<" given cameraInfo params:height = "<<calibrationHeight);
}

void CameraWrapper::targetGreyValueTest() {
    CFloatPtr ptrExpTest =pCam_->GetNodeMap().GetNode("AutoExposureTargetGreyValue");
    //CFloatPtr ptrExpTest=pCam_->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExpTest) || !IsReadable(ptrExpTest)){
        ROS_FATAL_STREAM("Unable to set exposure " << "). Aborting..." << endl << endl);
        return ;
    }
    ptrExpTest->SetValue(90.0);
    ROS_INFO_STREAM("target grey mode Time: "<<ptrExpTest->GetValue()<<endl);

}

void CameraWrapper::exposureTest() {
    CFloatPtr ptrExpTest=pCam_->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExpTest) || !IsReadable(ptrExpTest)){
        ROS_FATAL_STREAM("Unable to set exposure " << "). Aborting..." << endl << endl);
        return ;
    }
    float expTime=ptrExpTest->GetValue();
    ROS_DEBUG_STREAM("Exposure Time: "<<expTime<<endl);

}

void CameraWrapper::trigger() {

    pCam_->TriggerSoftware();
//    INodeMap & nodeMap = pCam_->GetNodeMap();
//
//    CCommandPtr ptr = nodeMap.GetNode("TriggerSoftware");
//    if (!IsAvailable(ptr) || !IsWritable(ptr))
//        ROS_FATAL_STREAM("Unable to execute trigger. Aborting...");
//    ptr->Execute();

}

void CameraWrapper::configureImageEvent(){
    try {
        pCam_->RegisterEventHandler(*this);
    }
    catch (Spinnaker::Exception& e){
        cout << "Error: " << e.what() << endl;
    }
}

void CameraWrapper::resetImageEvent(){
    try {
        pCam_->UnregisterEventHandler(*this);
        cout << "Image events unregistered..." << endl << endl;
    }
    catch (Spinnaker::Exception& e){
        cout << "Error: " << e.what() << endl;
    }
}

inline float CameraWrapper::convert_to_mat(ImagePtr pImage, Mat &img) {

    ImagePtr convertedImage;
    if (COLOR_)
        convertedImage = pImage->Convert(PixelFormat_BGR8);//, NO_COLOR_PROCESSING); //, NEAREST_NEIGHBOR);
    else
        convertedImage = pImage->Convert(PixelFormat_Mono8);//, NO_COLOR_PROCESSING); //, NEAREST_NEIGHBOR);
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();
    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    float mean=0;
    long cnt=0;
    if (COLOR_) {
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(),
                  convertedImage->GetStride());
        Rect crop(500,600, 1000, img.rows-700);  //first 400 rows + lowest 100
        cv::Scalar m= cv::mean(img(crop));
        //cv::Scalar m= cv::mean(img);

        mean = (m[0]+m[1]+m[2])/3.0;
    }
    else {
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(),
                  convertedImage->GetStride());
        cv::Scalar m = cv::mean(img);
        mean = m[0];
    }
    return mean;
    // return img;

}

void CameraWrapper::OnImageEvent(ImagePtr image) {
//        auto start = high_resolution_clock::now();
//    std::cout<<"camID "<<pCam_->DeviceID.ToString()<<std::endl;
    this->msg_time = int64(image->GetTimeStamp() / 1000);
    MatStamped ms;

    // Check image retrieval status
    if (image->IsIncomplete()){
        cout << "Image incomplete with image status " << image->GetImageStatus() << "..." <<endl;
        return;
    } else {
        if(image->GetTimeStamp()==0) {
            cout << "Image with zero timestamp!"<<endl;
            return;
        }
        ms.stamp = ros::Time(double(image->GetTimeStamp() / 1000000000.0)); // function returns nanoseconds

        if (frameID_ >= 0) {
            lastFrameID_ = frameID_;
            frameID_ = image->GetFrameID();
            ROS_WARN_STREAM_COND(frameID_ > lastFrameID_ + 1, "Frames are being skipped!");
        } else {
            frameID_ = image->GetFrameID();
        }
        ms.frame_id = frameID_;

        float mean;
        mean = convert_to_mat(image, ms.image);
        if(ms.stamp.isZero()) {
            std::cout << image->GetTimeStamp() << std::endl;
            cv::imshow("tmp", ms.image);
            cv::waitKey(0);
        }
        if (pic_active_) {
//TODO: for some reason I can't address elements of ms.image matrix outside of convert_to_mat function, thus the mean is computed in the function and passed to computeShutterAndGain
            pic_->computeShutterAndGain(gige_->exposure_time_, gige_->gain_, mean);
        }

        if (gige_->use_autoexposure_) {
            // set shutter
            const double exposureTimeMax = pCam_->ExposureTime.GetMax();
            const double exposureTimeMin = pCam_->ExposureTime.GetMin();
            if (gige_->exposure_time_ > exposureTimeMax) gige_->exposure_time_ = exposureTimeMax;
            if (gige_->exposure_time_ < exposureTimeMin) gige_->exposure_time_ = exposureTimeMin;
            setFloatValue("ExposureTime", gige_->exposure_time_);

            // set gain
            const double gainMax = pCam_->Gain.GetMax();
            const double gainMin = pCam_->Gain.GetMin();
            if (gige_->gain_ > gainMax) gige_->gain_ = gainMax;
            if (gige_->gain_ < gainMin) gige_->gain_ = gainMin;
            setFloatValue("Gain", gige_->gain_);
        }

        //sanity check
        unsigned int rows, cols, stride;
        rows = image->GetHeight();
        cols = image->GetWidth();
        stride = image->GetStride();
        if (cols != gige_->image_width_ || rows != gige_->image_height_) {
            std::cout << "stride: " << stride << "  step: " << gige_->image_stride_ << std::endl;
            //resize image or exit?
            return;
        }

        {
            std::lock_guard<std::mutex> lck(gige_->mutex_display_);
            this->qMatStampedDisplay_.push(ms);
        }

        {
            std::lock_guard<std::mutex> lck(gige_->mutex_notify_display_);
            gige_->dataReady_display_ = true;
        }
        gige_->condVar_display_.notify_one();
    }

//    auto end = high_resolution_clock::now();
//    auto duration = duration_cast<microseconds>(end - start);
//
//    cout << "ImgEvent: "
//         << duration.count() << " us" << endl;
//
}
