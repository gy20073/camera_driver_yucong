#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <condition_variable>

#include <string>
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// // ROS ..
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/fill_image.h"

// Self defined type.
#include <sync/config.h>
#include "sync/barrier.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

Barrier camera_barrier(0);

long diff[16] = {0};

std::mutex initialize_mutex;


int SLAVE_NUM; /* How many slave camera do we have? */
int check_in_num = 0; /* How many camera do we check in? */
long START_EPOCH;
long END_EPOCH;

/* Enum to select trigger type */
enum triggerType {
  SOFTWARE,
  HARDWARE
};

const triggerType chosenTrigger = HARDWARE;
const string MAIN_CAM_ID("17391317");

// This function configures a number of settings on the camera including offsets 
// X and Y, width, height, and pixel format. These settings must be applied before
// BeginAcquisition() is called; otherwise, they will be read only. Also, it is
// important to note that settings are applied immediately. This means if you plan
// to reduce the width and move the x offset accordingly, you need to apply such
// changes in the appropriate order.
int ConfigureCustomImageSettings(INodeMap & nodeMap)
{
  int result = 0;

  cout << endl << endl << "*** CONFIGURING CUSTOM IMAGE SETTINGS ***" << endl << endl;

  try
  {
    //
    // Apply mono 8 pixel format
    //
    // *** NOTES ***
    // Enumeration nodes are slightly more complicated to set than other
    // nodes. This is because setting an enumeration node requires working
    // with two nodes instead of the usual one. 
    //
    // As such, there are a number of steps to setting an enumeration node: 
    // retrieve the enumeration node from the nodemap, retrieve the desired 
    // entry node from the enumeration node, retrieve the integer value from 
    // the entry node, and set the new value of the enumeration node with
    // the integer value from the entry node.
    //
    // Retrieve the enumeration node from the nodemap
    CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
    {
      // Retrieve the desired entry node from the enumeration node
      CEnumEntryPtr ptrPixelFormatRGB8 = ptrPixelFormat->GetEntryByName("RGB8");
      if (IsAvailable(ptrPixelFormatRGB8) && IsReadable(ptrPixelFormatRGB8))
      {
        // Retrieve the integer value from the entry node
        int64_t pixelFormatRGB8 = ptrPixelFormatRGB8->GetValue();

        // Set integer as new value for enumeration node
        ptrPixelFormat->SetIntValue(pixelFormatRGB8);

        cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
      }
      else
      {
        cout << "Pixel format rgb 8 not available..." << endl;
      }
    }
    else
    {
      cout << "Pixel format not available..." << endl;
    }
    //
    // Set maximum width
    //
    // *** NOTES ***
    // Other nodes, such as those corresponding to image width and height, 
    // might have an increment other than 1. In these cases, it can be
    // important to check that the desired value is a multiple of the
    // increment. However, as these values are being set to the maximum,
    // there is no reason to check against the increment.
    //
    CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
    if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
    {
      int64_t widthToSet = ptrWidth->GetMax();

      ptrWidth->SetValue(widthToSet);

      cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
    }
    else
    {
      cout << "Width not available..." << endl;
    }

    //
    // Set maximum height
    //
    // *** NOTES ***
    // A maximum is retrieved with the method GetMax(). A node's minimum and
    // maximum should always be a multiple of its increment.
    //
    CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
    if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
    {
      int64_t heightToSet = ptrHeight->GetMax();

      ptrHeight->SetValue(heightToSet);

      cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
    }
    else
    {
      cout << "Height not available..." << endl << endl;
    }
  }
  catch (Spinnaker::Exception &e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }

  return result;
}

/* This function configures the camera to use the trigger.
 * 1. trigger mode set off to select trigger source.
 * 2. select trigger source, enable trigger mode,
 * which has the camera capture only a single image upon the 
 * execution of the chosen trigger.
*/
int ConfigureTrigger(INodeMap& nodeMap, INodeMap &nodeMapTLDevice, gcstring deviceSerialNumber) {
  int result = 0;
  cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;
  int isMain = 0;
  if (MAIN_CAM_ID.compare(deviceSerialNumber.c_str()) == 0) {
    isMain = 1;
  }

  try {
    //
    // Ensure trigger mode off
    //
    // *** NOTES ***
    // The trigger must be disabled in order to configure whether the source
    // is software or hardware.
    //
    
    CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
    if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode)) {
      cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
      return -1;
    }
        
    CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
    if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff)) {
      cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
      return -1;
    }
      
    ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
    cout << "Trigger mode disabled..." << endl;
    //
    // Select trigger source
    //
    // *** NOTES ***
    // The trigger source must be set to hardware or software while trigger 
    // mode is off.
    //
    CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
    if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource)) {
      cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
      return -1;
    }

    if (chosenTrigger == SOFTWARE) {
      cout << "We only support hardware !" << endl;
      return -1; 
      // Set trigger mode to software
      // CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
      // if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware)) {
        // cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
        // return -1;
      // }
        
      // ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
        
      // cout << "Trigger source set to software..." << endl;
    } else if (chosenTrigger == HARDWARE) {
      /* We need to configure our setting accourding to master/slave */
      if (isMain) {
        // For main, set trigger mode to hardware, line 0.
        // For slave, set trigger source as line 3.
        // Later on, main should be the last to enter GetNextImage()
        // and send the signal. 
        // In another word, main need to loop & check the number of threads,
        // reset the number, and call entering the GetNextImage.


        // cout << "MAIN USING SOFTWARE..." << endl;
        // CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
        // if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware)) {
        //   cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
        //   return -1;
        // }
        // ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
        // cout << deviceSerialNumber << " :Entering main camera, input set to Line0. " << endl; 
        CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
        if (!IsAvailable(ptrTriggerSourceHardware) || !(IsReadable(ptrTriggerSourceHardware))) {
          cout << "Unable to set trigger mode (enum entry retrieval). Aborting... " << endl;
          return -1;
        }
        ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
        cout << "Trigger source set to hardware..." << endl;
      } else {
        cout << deviceSerialNumber << " :Entering slave camera, input set to Line3. " << endl;
        // cout << "The device serial number is: " << deviceSerialNumber << endl;
        CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line3");
        if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware)) {
          cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
          return -1;
        }
            
        ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
        cout << "Trigger source set to hardware..." << endl;
      }
    }
      //
      // Turn trigger mode on
      //
      // *** LATER ***
      // Once the appropriate trigger source has been set, turn trigger mode 
      // on in order to retrieve images using the trigger.
      //
    
    CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
    if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn)) {
      cout << deviceSerialNumber << " :Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
      return -1;
    }
    ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());
    cout << deviceSerialNumber << " :Trigger mode turned back on..." << endl << endl;
    
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}

int PrintDeviceInfo(INodeMap &nodeMap) {
  int result = 0;
  cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

  try {
    FeatureList_t features;
    CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category)) {
      category->GetFeatures(features);
      FeatureList_t::const_iterator it;
      for (it=features.begin(); it != features.end(); ++it) {
        CNodePtr pfeatureNode = *it;
        cout << pfeatureNode->GetName() << " : ";
        CValuePtr pValue = (CValuePtr)pfeatureNode;
        cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
        cout << endl;
      }
    } else {
      cout << "Device control information not available." << endl;
    }
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}

int GrabNextImageByTrigger(INodeMap &nodeMap, CameraPtr pCam, int isMain) {
  int result = 0;
  try {
    if (chosenTrigger == SOFTWARE) {
      cout << "We only support hardare!" << endl;
      return -1;
    } else if (chosenTrigger == HARDWARE) {
      // Only main using software to carry other guys.
      // if (isMain) {
      //   cout << "Press the enter key to initiate software trigger." << endl;
      //   getchar();

      //   //Execute software trigger
      //   CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
      //   if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand)) {
      //     cout << "Unable to execute trigger, aborting..." << endl;
      //     return -1;
      //   }
      //   ptrSoftwareTriggerCommand->Execute();
      // }
    }
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}

int ResetTrigger(INodeMap &nodeMap) {
  int result = 0;
  try {
    /* Turn trigger mode off. */
    CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
    if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode)) {
      cout << "Unable to disable trigger mode (node retrieval). Non-fatal error..." << endl;
      return -1;
    }
    CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
    if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
    {
      cout << "Unable to disable trigger mode (enum entry retrieval). Non-fatal error..." << endl;
      return -1;
    }

    ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
    cout << "Trigger mode disabled..." << endl << endl;
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
}


/* This function acquire images for one camera. */
int AcquireImages(CameraPtr pCam, INodeMap &nodeMap, INodeMap &nodeMapTLDevice, int cameraNum, gcstring deviceSerialNumber, ros::NodeHandle &nh) {
  int result = 0;
  int isMain = 0;
  if (MAIN_CAM_ID.compare(deviceSerialNumber.c_str()) == 0) {
    isMain = 1;
  }
  cout << endl << "*** IMAGE ACQUISITION FOR CAMERA " << cameraNum << " ***" << endl;
  try {
    // Set acquisition mode to continuous.
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
      cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
      return -1;
    }
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous)) {
      cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl << endl;
      return -1;
    }

    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    cout << "Acquisition mode set to continuous..." << endl;

    /* Creating directory for different camera */
    ostringstream dirpath;
    dirpath << "./Camera_" << cameraNum << "_" << deviceSerialNumber;
    boost::filesystem::path p(dirpath.str().c_str());
    if (boost::filesystem::exists(p)) {
      if (boost::filesystem::is_directory(p)) {
        cout << "dir " << dirpath.str() << " exists" << endl;
      } else if (boost::filesystem::is_regular_file(p)) {
        cout << "file with same name exists. quitting.";
      }
    } else {
      boost::system::error_code ec;
      if (!boost::filesystem::create_directory(p, ec)) {
          cout << ec.message() << endl;
          return -1;
      } else {
        cout << "Directory " << dirpath.str() << " created." << endl;
      }
    }

    /* Creating publisher nodes... */
    std::ostringstream node_name_stream;
    node_name_stream << "image_sender_" << cameraNum;
    std::string node_name = node_name_stream.str();

    int buffer_size = 50; // constant to be modified later...
    cout << node_name.c_str() << endl;
    // ros::Publisher pub = nh.advertise<bfsdriver_1_1::ImageStamp>(node_name.c_str(), buffer_size);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>(node_name.c_str(), buffer_size);



    /* Begin acquiring images */
    pCam->BeginAcquisition();
    // printf("id: %s, Camera %d Acquiring images...\n", deviceSerialNumber.c_str(), cameraNum);

    // Retrieve, convert, and save images
    const int unsigned k_numImages = 1000;
    for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++) {
      try {
        // sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
        

        // IMPORTANT: ensure all the threads enter together.
        camera_barrier.wait(cameraNum);  //
        ImagePtr pResultImage = pCam->GetNextImage();
        camera_barrier.wait(cameraNum); 
        Time::time_point t = Time::now();
        
        unsigned long epoch_1 = std::chrono::time_point_cast<Micro>(t).time_since_epoch().count();
        if (imageCnt == 0) {
          START_EPOCH = epoch_1;
        } 
        if (imageCnt == k_numImages - 1) 
        {
          END_EPOCH = epoch_1;
        }


        if (pResultImage->IsIncomplete()) {
          cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
        } else {
          // Print image information
          
          printf("Camera: %d, grabbed image %d, width:%lu, height:%lu\n", cameraNum, imageCnt, pResultImage->GetWidth(), pResultImage->GetHeight());
          printf("device %s, cam %d start in %ld\n", deviceSerialNumber.c_str(), cameraNum, epoch_1);
          ImagePtr convertedImage = pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR);
          ostringstream filename;

          filename << dirpath.str() << "/Trigger-" << epoch_1 << "-" << cameraNum << ".jpg";
          convertedImage->Save(filename.str().c_str());

          /* Convert to CV mat */
          
          unsigned int XPadding = convertedImage->GetXPadding();
          unsigned int YPadding = convertedImage->GetYPadding();
          unsigned int rowsize = convertedImage->GetWidth();
          unsigned int colsize = convertedImage->GetHeight();
          // See converting tutorial. CV_8UC3 = > rgb8 
          cv::Mat image = cv::Mat(colsize + YPadding, rowsize + XPadding, 
          CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
          sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
          //sensor_msgs::Image target_image = *(cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg());
          //ImageStampPtr msg(new bfsdriver_1_1::ImageStamp);
          // sensor_msgs::fillImage(msg->img, target_image.encoding, target_image.height, target_image.width, target_image.step, reinterpret_cast<void *>(target_image.data.data()));
          // msg->timestamp = epoch_1;
          pub.publish(msg);
          cout << "Published..." << endl;
          ros::spinOnce();
          pResultImage->Release();
        }
      } catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
        result = -1;
      }
    }

  pCam->EndAcquisition();
  cout << "The frame rate is :" << k_numImages / ((END_EPOCH - START_EPOCH) / 1000000.0)  << endl;

  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}


/* This function is forked on all the cameras... */
void camera_worker_task(CameraPtr pCam, int cameraNum, ros::NodeHandle& nh) {
  int result = 0;
  int err = 0;

  try {
    /* Retrieve TL device nodemap and print device info */
    std::unique_lock<std::mutex> init_lock(initialize_mutex, std::defer_lock);
    init_lock.lock();
    INodeMap &nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    result = PrintDeviceInfo(nodeMapTLDevice);

    
    /* Initialize camera */
    pCam->Init();

    /* Retrieve GenICam nodemap */
    INodeMap &nodeMap = pCam->GetNodeMap();

    // Configure custom image settings
    err = ConfigureCustomImageSettings(nodeMap);
    if (err < 0)
    {
      return;
    }

    gcstring deviceSerialNumber("");
    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");

    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial)) {
        deviceSerialNumber = ptrStringSerial->GetValue();
        cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
      }

    /* Configure trigger */
    err = ConfigureTrigger(nodeMap, nodeMapTLDevice, deviceSerialNumber);

    if (err < 0) {
      cout << "Error: " << err << endl;
      return;
    }

    

    init_lock.unlock();
    camera_barrier.wait(cameraNum);

    // pCam->BalanceWhiteAuto.SetValue(Spinnaker::BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
    // CEnumerationPtr balanceWhiteAuto = nodeMap.GetNode("BalanceWhiteAuto");
    // balanceWhiteAuto->SetIntValue(balanceWhiteAuto->GetEntryByName("Off")->GetValue());

    /* Acquire images on each camera */
    result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice, cameraNum, deviceSerialNumber, nh);

    /* Reset the trigger back */
    result = result | ResetTrigger(nodeMap);

    /* De-initialize the camera */
    pCam->DeInit();

  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "trigger_sync_capture");
  ros::NodeHandle nh;
  ros::Rate loop(20); // Larger than framerate. 
  
  // pub = nh.advertise<sensor_msgs::Image>("image_sender", buffer_size);

  int result = 0;

  // Print application build info.
  cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
  
  // Retrieve singleton reference to system object.
  SystemPtr system = System::GetInstance();

  // Retrieve list of cameras from the system.
  CameraList camList = system->GetCameras();
  unsigned int numCameras = camList.GetSize();
  SLAVE_NUM = numCameras - 1;

  // Initialize the barrier.
  camera_barrier.setThreadCount(numCameras);

  cout << "Number of cameras detected: " << numCameras << endl << endl;
  // Finish if there are no cameras
  if (numCameras == 0) {
    // Clear camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();
    cout << "Not enough cameras!" << endl;
    cout << "Done! Press Enter to exit..." << endl;
    getchar();
    return -1;
  }

  // Fork on each camera:
  std::vector<std::thread> threads(numCameras);
  for (unsigned int i = 0; i < numCameras; i++) {
    cout << "Starting camera " << i << " in parallel." << endl;
    threads[i] = std::thread(camera_worker_task, camList.GetByIndex(i), i, std::ref(nh));
  }

  // Join on each camera:
  for (unsigned int i = 0; i < numCameras; i++) {
    threads[i].join();
    cout << "Camera " << i << " task complete..." << endl << endl;
  }

  // Clear camera list.
  camList.Clear();

  cout << "Cleared..." << endl;

  // Release system
  system->ReleaseInstance();
  cout << "Released..." << endl;

  cout << endl << "Done! Press Enter to exit..." << endl;
  getchar();

  return 0;

}
