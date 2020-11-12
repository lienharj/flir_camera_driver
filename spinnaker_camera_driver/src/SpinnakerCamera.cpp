/*
This code was developed by the National Robotics Engineering Center (NREC), part
of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for
public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A"
(Approved for Public Release, Distribution
Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.
Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following
disclaimer in the documentation and/or other materials provided with the
distribution.
Neither the name of the Carnegie Mellon University nor the names of its
contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*-*-C++-*-*/
/**
   @file SpinnakerCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "spinnaker_camera_driver/SpinnakerCamera.h"

#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>

#include <ros/ros.h>

namespace spinnaker_camera_driver
{
SpinnakerCamera::SpinnakerCamera()
  : serial_(0)
  , system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , pCam_(static_cast<int>(NULL))  // Hack to suppress compiler warning.
                                   // Spinnaker has only one contructor which
                                   // takes
                                   // an int
  , camera_(static_cast<int>(NULL))
  , captureRunning_(false)
{
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE("[SpinnakerCamera]: Number of cameras detected: " << num_cameras);
}

SpinnakerCamera::~SpinnakerCamera()
{
  camList_.Clear();
  system_->ReleaseInstance();
}

void SpinnakerCamera::setNewConfiguration(const spinnaker_camera_driver::SpinnakerConfig& config, const uint32_t& level)
{
  // Check if camera is connected
  if (!pCam_)
  {
    SpinnakerCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> scopedLock(mutex_);

  if (level >= LEVEL_RECONFIGURE_STOP)
  {
    ROS_DEBUG("SpinnakerCamera::setNewConfiguration: Reconfigure Stop.");
    bool capture_was_running = captureRunning_;
    start();  // For some reason some params only work after aquisition has be
              // started once.
    stop();
    camera_->setNewConfiguration(config, level);
    if (capture_was_running)
      start();
  }
  else
  {
    camera_->setNewConfiguration(config, level);
  }
}  // end setNewConfiguration

void SpinnakerCamera::setGain(const float& gain)
{
  if (camera_)
    camera_->setGain(gain);
}

int SpinnakerCamera::getHeightMax()
{
  if (camera_)
    return camera_->getHeightMax();
  else
    return 0;
}

int SpinnakerCamera::getWidthMax()
{
  if (camera_)
    return camera_->getWidthMax();
  else
    return 0;
}

Spinnaker::GenApi::CNodePtr SpinnakerCamera::readProperty(const Spinnaker::GenICam::gcstring property_name)
{
  if (camera_)
  {
    return camera_->readProperty(property_name);
  }
  else
  {
    return 0;
  }
}

void SpinnakerCamera::connect()
{
  if (!pCam_)
  {
    // If we have a specific camera to connect to (specified by a serial number)
    if (serial_ != 0)
    {
      const auto serial_string = std::to_string(serial_);

      try
      {
        pCam_ = camList_.GetBySerial(serial_string);
      }
      catch (const Spinnaker::Exception& e)
      {
        throw std::runtime_error("[SpinnakerCamera::connect] Could not find camera with serial "
                                 "number " +
                                 serial_string + ". Is that camera plugged in? Error: " + std::string(e.what()));
      }
    }
    else
    {
      // Connect to any camera (the first)
      try
      {
        pCam_ = camList_.GetByIndex(0);
      }
      catch (const Spinnaker::Exception& e)
      {
        throw std::runtime_error("[SpinnakerCamera::connect] Failed to get first connected camera. "
                                 "Error: " +
                                 std::string(e.what()));
      }
    }
    if (!pCam_ || !pCam_->IsValid())
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to obtain camera reference.");
    }

    try
    {
      // Check Device type and save serial for reconnecting
      Spinnaker::GenApi::INodeMap& genTLNodeMap = pCam_->GetTLDeviceNodeMap();

      if (serial_ == 0)
      {
        Spinnaker::GenApi::CStringPtr serial_ptr =
            static_cast<Spinnaker::GenApi::CStringPtr>(genTLNodeMap.GetNode("DeviceID"));
        if (IsAvailable(serial_ptr) && IsReadable(serial_ptr))
        {
          serial_ = atoi(serial_ptr->GetValue().c_str());
          ROS_INFO("[SpinnakerCamera::connect]: Using Serial: %i", serial_);
        }
        else
        {
          throw std::runtime_error("[SpinnakerCamera::connect]: Unable to determine serial number.");
        }
      }

      Spinnaker::GenApi::CEnumerationPtr device_type_ptr =
          static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceType"));

      if (IsAvailable(device_type_ptr) && IsReadable(device_type_ptr))
      {
        ROS_INFO_STREAM("[SpinnakerCamera::connect]: Detected device type: " << device_type_ptr->ToString());

        if (device_type_ptr->GetCurrentEntry() == device_type_ptr->GetEntryByName("U3V"))
        {
          Spinnaker::GenApi::CEnumerationPtr device_speed_ptr =
              static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceCurrentSpeed"));
          if (IsAvailable(device_speed_ptr) && IsReadable(device_speed_ptr))
          {
            if (device_speed_ptr->GetCurrentEntry() != device_speed_ptr->GetEntryByName("SuperSpeed"))
              ROS_ERROR_STREAM("[SpinnakerCamera::connect]: U3V Device not running at "
                               "Super-Speed. Check Cables! ");
          }
        }
        // TODO(mhosmar): - check if interface is GigE and connect to GigE cam
      }
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to determine device info with "
                               "error: " +
                               std::string(e.what()));
    }

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // detect model and set camera_ accordingly;
      Spinnaker::GenApi::CStringPtr model_name = node_map_->GetNode("DeviceModelName");
      std::string model_name_str(model_name->ToString());

      ROS_INFO("[SpinnakerCamera::connect]: Camera model name: %s", model_name_str.c_str());
      if (model_name_str.find("Blackfly S") != std::string::npos)
      {
        camera_.reset(new Camera(node_map_));
        ROS_INFO("[SpinnakerCamera::connect]: Blackfly S camera detected");
      }
      else if (model_name_str.find("Chameleon3") != std::string::npos)
      {
        camera_.reset(new Cm3(node_map_));
        ROS_INFO("[SpinnakerCamera::connect]: Chameleon3 camera detected");
      }
      else if (model_name_str.find("Grasshopper3") != std::string::npos)
      {
        camera_.reset(new Gh3(node_map_));
        ROS_INFO("[SpinnakerCamera::connect]: Grasshopper3 camera detected");
        set_newest_only_buffer_stream(pCam_);
      }
      else
      {
        camera_.reset(new Camera(node_map_));
        ROS_WARN("SpinnakerCamera::connect: Could not detect camera model name.");
      }

      // Configure chunk data - Enable Metadata
      SpinnakerCamera::ConfigureChunkData(*node_map_);
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to connect to camera. Error: " +
                               std::string(e.what()));
    }
    catch (const std::runtime_error& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to configure chunk data. Error: " +
                               std::string(e.what()));
    }
  }

  // TODO(mhosmar): Get camera info to check if camera is running in color or
  // mono mode
  /*
  CameraInfo cInfo;
  error = cam_.GetCameraInfo(&cInfo);
  SpinnakerCamera::handleError("SpinnakerCamera::connect  Failed to get camera
  info.", error);
  isColor_ = cInfo.isColorCamera;
  */
}

void SpinnakerCamera::disconnect()
{
  std::lock_guard<std::mutex> scopedLock(mutex_);
  captureRunning_ = false;
  try
  {
    // Check if camera is connected
    if (pCam_)
    {
      pCam_->DeInit();
      pCam_ = static_cast<int>(NULL);
      camList_.RemoveBySerial(std::to_string(serial_));
    }
    Spinnaker::CameraList temp_list = system_->GetCameras();
    camList_.Append(temp_list);
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::disconnect] Failed to disconnect camera with "
                             "error: " +
                             std::string(e.what()));
  }
}

void SpinnakerCamera::start()
{
  try
  {
    // Check if camera is connected
    if (pCam_ && !captureRunning_)
    {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::start] Failed to start capture with error: " + std::string(e.what()));
  }
}

void SpinnakerCamera::stop()
{
  if (pCam_ && captureRunning_)
  {
    // Stop capturing images
    try
    {
      captureRunning_ = false;
      pCam_->EndAcquisition();
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::stop] Failed to stop capture with error: " + std::string(e.what()));
    }
  }
}

uint64_t SpinnakerCamera::getFrameCounter(void)
{
  uint64_t ret = image_metadata_.GetFrameID();
  return ret;
}

void SpinnakerCamera::grabImage(sensor_msgs::Image* image, const std::string& frame_id)
{
  std::lock_guard<std::mutex> scopedLock(mutex_);

  // Check if Camera is connected and Running
  if (pCam_ && captureRunning_)
  {
    // Handle "Image Retrieval" Exception
    try
    {
      // Since it takes time to initialize (i.e., enabling trigger) arduino
      // sync. Otherwise it will produce timeout error of this driver. Therefore
      // give more time for grabbing an image by increasing timeout.
      timeout_ = 60000;  // 60secs
      Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage(timeout_);
      //  std::string format(image_ptr->GetPixelFormatName());
      //  std::printf("\033[100m format: %s \n", format.c_str());

      if (image_ptr->IsIncomplete())
      {
        throw std::runtime_error("[SpinnakerCamera::grabImage] Image received from camera " + std::to_string(serial_) +
                                 " is incomplete.");
      }
      else
      {
        // Set Image Time Stamp
        image->header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
        image->header.stamp.nsec = image_ptr->GetTimeStamp();

        // Check the bits per pixel.
        size_t bitsPerPixel = image_ptr->GetBitsPerPixel();

        // --------------------------------------------------
        // Set the image encoding
        std::string imageEncoding = sensor_msgs::image_encodings::MONO8;

        Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
            static_cast<Spinnaker::GenApi::CEnumerationPtr>(node_map_->GetNode("PixelColorFilter"));

        Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();
        Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
        Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
        Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
        Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

        // if(isColor_ && bayer_format != NONE)
        if (color_filter_ptr->GetCurrentEntry() != color_filter_ptr->GetEntryByName("None"))
        {
          if (bitsPerPixel == 16)
          {
            // 16 Bits per Pixel
            if (color_filter_str.compare(bayer_rg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
            }
            else if (color_filter_str.compare(bayer_gr_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
            }
            else if (color_filter_str.compare(bayer_gb_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
            }
            else if (color_filter_str.compare(bayer_bg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
            }
            else
            {
              throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized "
                                       "for 16-bit format.");
            }
          }
          else
          {
            // 8 Bits per Pixel
            if (color_filter_str.compare(bayer_rg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            }
            else if (color_filter_str.compare(bayer_gr_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
            }
            else if (color_filter_str.compare(bayer_gb_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
            }
            else if (color_filter_str.compare(bayer_bg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            }
            else
            {
              throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized "
                                       "for 8-bit format.");
            }
          }
        }
        else  // Mono camera or in pixel binned mode.
        {
          if (bitsPerPixel == 16)
          {
            imageEncoding = sensor_msgs::image_encodings::MONO16;
          }
          else if (bitsPerPixel == 24)
          {
            imageEncoding = sensor_msgs::image_encodings::BGR8;
          }
          else
          {
            imageEncoding = sensor_msgs::image_encodings::MONO8;
          }
        }

        int width = image_ptr->GetWidth();
        int height = image_ptr->GetHeight();
        int stride = image_ptr->GetStride();

        // ROS_INFO_ONCE("\033[93m wxh: (%d, %d), stride: %d \n", width, height,
        // stride);
        fillImage(*image, imageEncoding, height, width, stride, image_ptr->GetData());
        image->header.frame_id = frame_id;

        image_metadata_ = image_ptr->GetChunkData();
      }  // end else
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::grabImage] Failed to retrieve buffer with "
                               "error: " +
                               std::string(e.what()));
    }
  }
  else if (pCam_)
  {
    throw CameraNotRunningException("[SpinnakerCamera::grabImage] Camera is currently not running.  Please "
                                    "start "
                                    "capturing frames first.");
  }
  else
  {
    throw std::runtime_error("[SpinnakerCamera::grabImage] Not connected to the camera.");
  }
}  // end grabImage

void SpinnakerCamera::setTimeout(const double& timeout)
{
  timeout_ = static_cast<uint64_t>(std::round(timeout * 1000));
}
void SpinnakerCamera::setDesiredCamera(const uint32_t& id)
{
  serial_ = id;
}

void SpinnakerCamera::ConfigureChunkData(const Spinnaker::GenApi::INodeMap& nodeMap)
{
  ROS_INFO_STREAM("*** CONFIGURING CHUNK DATA ***");
  try
  {
    // Activate chunk mode
    //
    // *** NOTES ***
    // Once enabled, chunk data will be available at the end of the payload
    // of every image captured until it is disabled. Chunk data can also be
    // retrieved from the nodemap.
    //
    Spinnaker::GenApi::CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkModeActive) || !Spinnaker::GenApi::IsWritable(ptrChunkModeActive))
    {
      throw std::runtime_error("Unable to activate chunk mode. Aborting...");
    }
    ptrChunkModeActive->SetValue(true);
    ROS_INFO_STREAM_ONCE("Chunk mode activated...");

    // Enable all types of chunk data
    //
    // *** NOTES ***
    // Enabling chunk data requires working with nodes: "ChunkSelector"
    // is an enumeration selector node and "ChunkEnable" is a boolean. It
    // requires retrieving the selector node (which is of enumeration node
    // type), selecting the entry of the chunk data to be enabled, retrieving
    // the corresponding boolean, and setting it to true.
    //
    // In this example, all chunk data is enabled, so these steps are
    // performed in a loop. Once this is complete, chunk mode still needs to
    // be activated.
    //
    Spinnaker::GenApi::NodeList_t entries;
    // Retrieve the selector node
    Spinnaker::GenApi::CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelector) || !Spinnaker::GenApi::IsReadable(ptrChunkSelector))
    {
      throw std::runtime_error("Unable to retrieve chunk selector. Aborting...");
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    ROS_INFO_STREAM("Enabling entries...");

    for (unsigned int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) ||
          !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      ROS_INFO_STREAM("\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ");
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable))
      {
        ROS_INFO("Node not available");
      }
      else if (ptrChunkEnable->GetValue())
      {
        ROS_INFO("Enabled");
      }
      else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        ROS_INFO("Enabled");
      }
      else
      {
        ROS_INFO("Node not writable");
      }
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error(e.what());
  }
}

void SpinnakerCamera::set_newest_only_buffer_stream(Spinnaker::CameraPtr cam_ptr)
{
  /**
  // Set stream buffer count mode to Manual
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferCountMode) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferCountMode))
    {
      cam_ptr->TLStream.StreamBufferCountMode.SetValue(Spinnaker::StreamBufferCountMode_Manual);
      ROS_INFO("[SpinnakerCamera] Set Stream Buffer Count Mode to Manual");
    }
    else
    {
      ROS_INFO("[SpinnakerCamera] Unable set Stream Buffer Count Mode to Manual");
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR_STREAM("[SpinnakerCamera] Error: " << e.what());
  }

  // Buffer Options
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferHandlingMode) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferHandlingMode))
    {
      cam_ptr->TLStream.StreamBufferHandlingMode.SetValue(Spinnaker::StreamBufferHandlingMode_NewestOnly);
      ROS_INFO("[SpinnakerCamera] Set only store and receive newest image in buffer");
    }
    else
    {
      ROS_INFO("[SpinnakerCamera] Unable to set to only show newest image");
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR_STREAM("[SpinnakerCamera] Error: " << e.what());
  }
  **/

  /**
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferHandlingMode) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferHandlingMode))
    {
      cam_ptr->TLStream.StreamBufferHandlingMode.SetValue(
          Spinnaker::StreamBufferHandlingModeEnum::StreamBufferHandlingMode_OldestFirst);
      ROS_INFO("[SpinnakerCamera] Set to receive oldest image in buffer");
    }
    else
    {
      ROS_INFO("[SpinnakerCamera] Unable to set to receive oldest image in buffer");
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR_STREAM("[SpinnakerCamera] Error: " << e.what());
  }

  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferCountManual) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferCountManual))
    {
      cam_ptr->TLStream.StreamBufferCountManual.SetValue(3);
      ROS_INFO("[SpinnakerCamera] Set buffer to be 3");
    }
    else
    {
      ROS_INFO("[SpinnakerCamera] Unable to set buffer to be 3");
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR_STREAM("[SpinnakerCamera] Error: " << e.what());
  }
  **/

  // Experiment to determine if it is camera settings that are causing the ripples
  try
  {
    Spinnaker::GenApi::INodeMap& nodeMap = cam_ptr->GetNodeMap();
    Spinnaker::GenApi::CEnumerationPtr ptrVideoMode = nodeMap.GetNode("VideoMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrVideoMode) || !Spinnaker::GenApi::IsWritable(ptrVideoMode))
    {
      std::cout << "Unable to access Video Mode. "
                   "Aborting..."
                << std::endl;
      throw;
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrMode1 = ptrVideoMode->GetEntryByName("Mode1");
    if (!Spinnaker::GenApi::IsAvailable(ptrMode1) || !Spinnaker::GenApi::IsReadable(ptrMode1))
    {
      std::cout << "Video Mode 1 is not available. Aborting... " << std::endl;
      throw;
    }

    ptrVideoMode->SetIntValue(ptrMode1->GetValue());
    std::cout << "Set video mode to 1" << std::endl;

    Spinnaker::GenApi::CEnumerationPtr ptrBinningControl = nodeMap.GetNode("BinningControl");
    if (!Spinnaker::GenApi::IsAvailable(ptrBinningControl) || !Spinnaker::GenApi::IsWritable(ptrBinningControl))
    {
      std::cout << "Unable to access BinningControl. "
                   "Aborting..."
                << std::endl;
      throw;
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrAverage = ptrBinningControl->GetEntryByName("Average");
    if (!Spinnaker::GenApi::IsAvailable(ptrAverage) || !Spinnaker::GenApi::IsReadable(ptrAverage))
    {
      std::cout << "Average binning is not available. Aborting... " << std::endl;
      throw;
    }

    ptrBinningControl->SetIntValue(ptrAverage->GetValue());
    std::cout << "Set to Average binning" << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set no pixel color filter
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->PixelColorFilter) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->PixelColorFilter))
    {
      cam_ptr->PixelColorFilter.SetValue(Spinnaker::PixelColorFilter_None);
      std::cout << "Disabled all pixel color filters" << std::endl;
    }
    else
    {
      std::cout << "Failed to disable pixel color filters" << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set pixel format to be Mono8
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->PixelFormat) && Spinnaker::GenApi::IsWritable(cam_ptr->PixelFormat))
    {
      cam_ptr->PixelFormat.SetValue(Spinnaker::PixelFormat_Mono8);
      std::cout << "Pixel format set to " << cam_ptr->PixelFormat.GetCurrentEntry()->GetSymbolic() << "..."
                << std::endl;
    }
    else
    {
      std::cout << "Pixel format not available..." << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set Offset X
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->OffsetX) && Spinnaker::GenApi::IsWritable(cam_ptr->OffsetX))
    {
      cam_ptr->OffsetX.SetValue(0);
      std::cout << "Offset X set to " << cam_ptr->OffsetX.GetValue() << "..." << std::endl;
    }
    else
    {
      std::cout << "Offset X not available..." << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set Offset Y
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->OffsetY) && Spinnaker::GenApi::IsWritable(cam_ptr->OffsetY))
    {
      cam_ptr->OffsetY.SetValue(0);
      std::cout << "Offset Y set to " << cam_ptr->OffsetY.GetValue() << "..." << std::endl;
    }
    else
    {
      std::cout << "Offset Y not available..." << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set Width (Might not be able to be changed)
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->Width) && Spinnaker::GenApi::IsWritable(cam_ptr->Width) &&
        cam_ptr->Width.GetInc() != 0 && cam_ptr->Width.GetMax() != 0)
    {
      cam_ptr->Width.SetValue(cam_ptr->Width.GetMax());
      std::cout << "Width set to " << cam_ptr->Width.GetValue() << "..." << std::endl;
    }
    else
    {
      std::cout << "Width not available..." << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set Height (Might not be able to be changed)
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->Height) && Spinnaker::GenApi::IsWritable(cam_ptr->Height) &&
        cam_ptr->Height.GetInc() != 0 && cam_ptr->Height.GetMax() != 0)
    {
      cam_ptr->Height.SetValue(cam_ptr->Height.GetMax());
      std::cout << "Height set to " << cam_ptr->Height.GetValue() << "..." << std::endl;
    }
    else
    {
      std::cout << "Height not available..." << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Auto exposure
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->ExposureAuto) && Spinnaker::GenApi::IsWritable(cam_ptr->ExposureAuto))
    {
      cam_ptr->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
      std::cout << "Automatic exposure disabled..." << std::endl;
    }
    else
    {
      std::cout << "Unable to disable automatic exposure." << std::endl << std::endl;
      // throw;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // For Grasshopper, we set a temporatrily low frame rate of 50Hz so we can
  // set the exposure timings with no issues later. Video mode has fps of
  // 240Hz, so you would not be able to set common exposure timings of
  // 8.333ms, 16.667ms
  // Note: This must be done only after auto exposure is disabled since possible
  // exposure times are tied to fps

  try
  {
    if (Spinnaker::GenApi::IsAvailable(cam_ptr->AcquisitionFrameRateEnable) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->AcquisitionFrameRateEnable))
    {
      cam_ptr->AcquisitionFrameRateEnable.SetValue(true);
      std::cout << "Enable acquisition frame rate" << std::endl;
    }
    else
    {
      std::cout << "Failed to set enable acquisition frame rate" << std::endl;
    }

    {
      Spinnaker::GenApi::INodeMap& nodeMap = cam_ptr->GetNodeMap();
      Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionFrameRateAuto = nodeMap.GetNode("AcquisitionFrameRateAuto");
      if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionFrameRateAuto) ||
          !Spinnaker::GenApi::IsWritable(ptrAcquisitionFrameRateAuto))
      {
        std::cout << "Unable to access AcquisitionFrameRateAuto. " << std::endl;
      }

      Spinnaker::GenApi::CEnumEntryPtr ptrOff = ptrAcquisitionFrameRateAuto->GetEntryByName("Off");
      if (!Spinnaker::GenApi::IsAvailable(ptrOff) || !Spinnaker::GenApi::IsReadable(ptrOff))
      {
        std::cout << "Off for ptrOff is not available. Aborting... " << std::endl;
      }

      ptrAcquisitionFrameRateAuto->SetIntValue(ptrOff->GetValue());
      std::cout << "Set AcquisitionFrameRateAuto mode to Off" << std::endl;
    }

    if (Spinnaker::GenApi::IsAvailable(cam_ptr->AcquisitionFrameRate) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->AcquisitionFrameRate))
    {
      cam_ptr->AcquisitionFrameRate.SetValue(50);
      std::cout << "Set acquisition frame rate to 50" << std::endl;
    }
    else
    {
      std::cout << "Failed to set acquisition frame rate to 50" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Gamma
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->GammaEnable) && Spinnaker::GenApi::IsWritable(cam_ptr->GammaEnable))
    {
      cam_ptr->GammaEnable.SetValue(false);
      std::cout << "Set GammaEnable to " << false << std::endl;
    }
    else
    {
      std::cout << "Unable to disable Gamma" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set gamma value to 1.0
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->Gamma) && Spinnaker::GenApi::IsWritable(cam_ptr->Gamma))
    {
      cam_ptr->Gamma.SetValue(1.0f);
      std::cout << "Set Gamma to " << 1.0f << std::endl;
    }
    else
    {
      std::cout << "Unable to disable Gamma" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Autogain
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->GainAuto) && Spinnaker::GenApi::IsWritable(cam_ptr->GainAuto))
    {
      cam_ptr->GainAuto.SetValue(Spinnaker::GainAuto_Off);
      std::cout << "Set GainAuto to " << false << std::endl;
    }
    else
    {
      std::cout << "Unable to disable Auto-gain" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set stream buffer count mode to Manual
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferCountMode) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferCountMode))
    {
      cam_ptr->TLStream.StreamBufferCountMode.SetValue(Spinnaker::StreamBufferCountMode_Manual);
      std::cout << "Set Stream Buffer Count Mode to Manual" << std::endl;
    }
    else
    {
      std::cout << "Unable set Stream Buffer Count Mode to Manual" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Buffer Options
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->TLStream.StreamBufferHandlingMode) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->TLStream.StreamBufferHandlingMode))
    {
      cam_ptr->TLStream.StreamBufferHandlingMode.SetValue(Spinnaker::StreamBufferHandlingMode_NewestOnly);
      std::cout << "Set only show newest image" << std::endl;
    }
    else
    {
      std::cout << "Unable to set to only show newest image" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Sharpness Correction
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->SharpeningEnable) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->SharpeningEnable))
    {
      cam_ptr->SharpeningEnable.SetValue(false);
      std::cout << "Disable Sharpness Correction" << std::endl;
    }
    else
    {
      std::cout << "Unable to disable sharpness correction" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Sharpness Auto
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->SharpeningAuto) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->SharpeningAuto))
    {
      cam_ptr->SharpeningAuto.SetValue(false);
      std::cout << "Disable Auto-Sharpen" << std::endl;
    }
    else
    {
      std::cout << "Unable to disable Auto-Sharpen" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Disable Saturation Auto
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->SaturationEnable) &&
        Spinnaker::GenApi::IsWritable(cam_ptr->SaturationEnable))
    {
      cam_ptr->SaturationEnable.SetValue(false);
      std::cout << "Disable Auto-Saturation" << std::endl;
    }
    else
    {
      std::cout << "Unable to disable Auto-Saturation" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set Black Level to Zero
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->BlackLevel) && Spinnaker::GenApi::IsWritable(cam_ptr->BlackLevel))
    {
      cam_ptr->BlackLevel.SetValue(0.0f);
      std::cout << "Set black level to " << cam_ptr->BlackLevel.GetValue() << std::endl;
    }
    else
    {
      std::cout << "Unable to set black value to zero" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->ExposureTime) && Spinnaker::GenApi::IsWritable(cam_ptr->ExposureTime))
    {
      // Note exposure time is in micro-seconds
      // In settings, shutter is in ms so we need to perform a conversion
      cam_ptr->ExposureTime.SetValue(8333.333f);
      std::cout << "Set exposure time to [micro s]: " << cam_ptr->ExposureTime.GetValue() << std::endl;
    }
    else
    {
      std::cout << "Could not set exposure time" << std::endl;
    }

    if (Spinnaker::GenApi::IsReadable(cam_ptr->Gain) && Spinnaker::GenApi::IsWritable(cam_ptr->Gain))
    {
      cam_ptr->Gain.SetValue(0.0f);
      std::cout << "Set gain to: " << cam_ptr->Gain.GetValue() << std::endl;
    }
    else
    {
      std::cout << "Could not set gain" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;

    // Make sure trigger mode is disabled before we configure it
    try
    {
      if (cam_ptr->TriggerMode.GetAccessMode() != Spinnaker::GenApi::RW)
      {
        std::cout << "Unable to disable trigger mode. Aborting..." << std::endl;
        throw;
      }
      cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
      std::cout << "Trigger mode disabled temporarily..." << std::endl;
    }
    catch (Spinnaker::Exception& e)
    {
      std::cout << "Error: " << e.what() << std::endl;
    }

    // Configure for hardware trigger
    try
    {
      // Set the trigger source to hardware (using 'Line0')
      if (cam_ptr->TriggerSource == NULL || cam_ptr->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW)
      {
        std::cout << "Unable to set trigger mode (node retrieval). Aborting..." << std::endl;
        throw;
      }
      cam_ptr->TriggerSource.SetValue(Spinnaker::TriggerSource_Line0);
      std::cout << "Trigger source set to hardware (line 0)..." << std::endl;
    }
    catch (Spinnaker::Exception& e)
    {
      std::cout << "Error: " << e.what() << std::endl;
    }
  }

  // Set trigger selector
  try
  {
    if (cam_ptr->TriggerSelector == NULL || cam_ptr->TriggerSelector.GetAccessMode() != Spinnaker::GenApi::RW)
    {
      std::cout << "Unable to set trigger selector. Aborting..." << std::endl;
      throw;
    }
    cam_ptr->TriggerSelector.SetValue(Spinnaker::TriggerSelectorEnums::TriggerSelector_FrameStart);
    std::cout << "Set Trigger Selector to FrameStart" << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set trigger activation
  try
  {
    if (cam_ptr->TriggerActivation == NULL || cam_ptr->TriggerActivation.GetAccessMode() != Spinnaker::GenApi::RW)
    {
      std::cout << "Unable to set trigger activation. Aborting..." << std::endl;
      throw;
    }
    cam_ptr->TriggerActivation.SetValue(Spinnaker::TriggerActivationEnums::TriggerActivation_RisingEdge);
    std::cout << "Set Trigger Activation to Rising Edge" << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Turn trigger mode back on
  try
  {
    if (cam_ptr->TriggerSource == NULL || cam_ptr->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW)
    {
      std::cout << "Unable to set trigger mode (node retrieval). Aborting..." << std::endl;
      throw;
    }
    cam_ptr->TriggerMode.SetValue(Spinnaker::TriggerMode_On);
    std::cout << "Trigger mode turned back on..." << std::endl << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set acquisition mode to continuous
  try
  {
    if (cam_ptr->AcquisitionMode == NULL || cam_ptr->AcquisitionMode.GetAccessMode() != Spinnaker::GenApi::RW)
    {
      std::cout << "Unable to set acquisition mode to continuous. Aborting..." << std::endl;
      throw;
    }
    cam_ptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
    std::cout << "Acquisition mode set to continuous..." << std::endl;
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }
}

}  // namespace spinnaker_camera_driver
