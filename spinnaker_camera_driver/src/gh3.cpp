/**
Software License Agreement (BSD)
\file      gh3.cpp
\authors   Michael Hosmar <mhosmar@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "spinnaker_camera_driver/gh3.h"

#include <string>

namespace spinnaker_camera_driver
{
Gh3::Gh3(Spinnaker::GenApi::INodeMap* node_map) : Camera(node_map)
{
}

Gh3::~Gh3()
{
}

void Gh3::setFrameRate(const float frame_rate)
{
  // This enables the "AcquisitionFrameRateEnabled"
  //======================================
  setProperty(node_map_, "AcquisitionFrameRateEnabled", true);  // different from Bfly S

  // This sets the "AcquisitionFrameRateAuto" to "Off"
  //======================================
  setProperty(node_map_, "AcquisitionFrameRateAuto", static_cast<std::string>("Off"));  // different from Bfly S

  // This sets the "AcquisitionFrameRate" to X FPS
  // ========================================

  Spinnaker::GenApi::CFloatPtr ptrAcquisitionFrameRate = node_map_->GetNode("AcquisitionFrameRate");
  ROS_DEBUG_STREAM("Minimum Frame Rate: \t " << ptrAcquisitionFrameRate->GetMin());
  ROS_DEBUG_STREAM("Maximum Frame rate: \t " << ptrAcquisitionFrameRate->GetMax());

  // Finally Set the Frame Rate
  setProperty(node_map_, "AcquisitionFrameRate", frame_rate);

  ROS_DEBUG_STREAM("Current Frame rate: \t " << ptrAcquisitionFrameRate->GetValue());
}

void Gh3::setNewConfiguration(const SpinnakerConfig& config, const uint32_t& level)
{
  try
  {
    if (level >= LEVEL_RECONFIGURE_STOP)
    {
      // Original code
      setImageControlFormats(config);
    }

    setFrameRate(static_cast<float>(config.acquisition_frame_rate));
    setProperty(node_map_, "AcquisitionFrameRateEnabled",
                config.acquisition_frame_rate_enable);  // Set enable after frame rate encase its false

    // Set Trigger and Strobe Settings
    // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to configure whether the source is
    // software or hardware.

    ROS_INFO("[gh3.cpp:setNewConfiguration] Temporarily turning off trigger mode so as to configure trigger options");
    setProperty(node_map_, "TriggerMode", std::string("Off"));
    setProperty(node_map_, "TriggerSource", config.trigger_source);
    setProperty(node_map_, "TriggerSelector", config.trigger_selector);
    setProperty(node_map_, "TriggerActivation", config.trigger_activation_mode);
    ROS_INFO("[gh3.cpp:setNewConfiguration] Now turning trigger mode back on if configured to be so");
    setProperty(node_map_, "TriggerMode", config.enable_trigger);
    // Only after trigger mode has been turned on then we can set trigger overlap option
    setProperty(node_map_, "TriggerOverlap", config.trigger_overlap_mode);

    // setProperty(node_map_, "LineSelector", config.line_selector);
    // setProperty(node_map_, "LineMode", config.line_mode);
    // setProperty(node_map_, "LineSource", config.line_source); // Not available in GH3

    // Set auto exposure
    setProperty(node_map_, "ExposureMode", config.exposure_mode);
    setProperty(node_map_, "ExposureAuto", config.exposure_auto);

    // Set sharpness
    if (IsAvailable(node_map_->GetNode("SharpeningEnable")))
    {
      setProperty(node_map_, "SharpeningEnable", config.sharpening_enable);
      if (config.sharpening_enable)
      {
        setProperty(node_map_, "SharpeningAuto", config.auto_sharpness);
        setProperty(node_map_, "Sharpening", static_cast<float>(config.sharpness));
        setProperty(node_map_, "SharpeningThreshold", static_cast<float>(config.sharpening_threshold));
      }
    }

    // Set saturation
    if (IsAvailable(node_map_->GetNode("SaturationEnable")))
    {
      setProperty(node_map_, "SaturationEnable", config.saturation_enable);
      if (config.saturation_enable)
      {
        setProperty(node_map_, "Saturation", static_cast<float>(config.saturation));
      }
    }

    // Set shutter time/speed
    if (config.exposure_auto.compare(std::string("Off")) == 0)
    {
      setProperty(node_map_, "ExposureTime", static_cast<float>(config.exposure_time));
    }
    else
    {
      setProperty(node_map_, "AutoExposureTimeUpperLimit",
                  static_cast<float>(config.auto_exposure_time_upper_limit));  // Different than BFly S
    }

    // Set gain
    // setProperty(node_map_, "GainSelector", config.gain_selector); //Not Writeable for GH3
    setProperty(node_map_, "GainAuto", config.auto_gain);
    if (config.auto_gain.compare(std::string("Off")) == 0)
    {
      setProperty(node_map_, "Gain", static_cast<float>(config.gain));
    }

    // Set brightness
    setProperty(node_map_, "BlackLevel", static_cast<float>(config.brightness));

    // Set gamma
    if (config.gamma_enable)
    {
      setProperty(node_map_, "GammaEnabled", config.gamma_enable);  // GH3 includes -ed
      setProperty(node_map_, "Gamma", static_cast<float>(config.gamma));
    }

    // Set white balance
    if (IsAvailable(node_map_->GetNode("BalanceWhiteAuto")))
    {
      setProperty(node_map_, "BalanceWhiteAuto", config.auto_white_balance);
      if (config.auto_white_balance.compare(std::string("Off")) == 0)
      {
        setProperty(node_map_, "BalanceRatioSelector", "Blue");
        setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_blue_ratio));
        setProperty(node_map_, "BalanceRatioSelector", "Red");
        setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_red_ratio));
      }
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[Gh3::setNewConfiguration] Failed to set configuration: " + std::string(e.what()));
  }
}

// Image Size and Pixel Format
void Gh3::setImageControlFormats(const spinnaker_camera_driver::SpinnakerConfig& config)
{
  // Set Binning and Decimation
  // setProperty(node_map_, "BinningHorizontal", config.image_format_x_binning);
  setProperty(node_map_, "BinningVertical", config.image_format_y_binning);
  // setProperty(node_map_, "DecimationHorizontal", config.image_format_x_decimation);
  // setProperty(node_map_, "DecimationVertical", config.image_format_y_decimation);

  // Grab the Max values after decimation
  Spinnaker::GenApi::CIntegerPtr height_max_ptr = node_map_->GetNode("HeightMax");
  if (!IsAvailable(height_max_ptr) || !IsReadable(height_max_ptr))
  {
    throw std::runtime_error("[Gh3::setImageControlFormats] Unable to read HeightMax");
  }
  height_max_ = height_max_ptr->GetValue();
  Spinnaker::GenApi::CIntegerPtr width_max_ptr = node_map_->GetNode("WidthMax");
  if (!IsAvailable(width_max_ptr) || !IsReadable(width_max_ptr))
  {
    throw std::runtime_error("[Gh3::setImageControlFormats] Unable to read WidthMax");
  }
  width_max_ = width_max_ptr->GetValue();

  // Offset first encase expanding ROI
  // Apply offset X
  setProperty(node_map_, "OffsetX", 0);
  // Apply offset Y
  setProperty(node_map_, "OffsetY", 0);

  // Set Width/Height
  if (config.image_format_roi_width <= 0 || config.image_format_roi_width > width_max_)
    setProperty(node_map_, "Width", width_max_);
  else
    setProperty(node_map_, "Width", config.image_format_roi_width);
  if (config.image_format_roi_height <= 0 || config.image_format_roi_height > height_max_)
    setProperty(node_map_, "Height", height_max_);
  else
    setProperty(node_map_, "Height", config.image_format_roi_height);

  // Apply offset X
  setProperty(node_map_, "OffsetX", config.image_format_x_offset);
  // Apply offset Y
  setProperty(node_map_, "OffsetY", config.image_format_y_offset);

  // Set Pixel Format
  setProperty(node_map_, "PixelFormat", config.image_format_color_coding);
}

void Gh3::setStructuredLightConfiguration(Spinnaker::CameraPtr cam_ptr)
{
  try
  {
    // Set to Video Mode
    Spinnaker::GenApi::INodeMap& nodeMap = cam_ptr->GetNodeMap();
    Spinnaker::GenApi::CEnumerationPtr ptrVideoMode = nodeMap.GetNode("VideoMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrVideoMode) || !Spinnaker::GenApi::IsWritable(ptrVideoMode))
    {
      std::cout << "Unable to access Video Mode. "
                   "Aborting..."
                << std::endl;
      throw;
    }

    // Set to Mode 1 (2x2 binning mode)
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

    // Set Average Binning
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

  // Set Width
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

  // Set Height
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

  // Set exposure time
  try
  {
    if (Spinnaker::GenApi::IsReadable(cam_ptr->ExposureTime) && Spinnaker::GenApi::IsWritable(cam_ptr->ExposureTime))
    {
      // Note exposure time is in micro-seconds
      // cam_ptr->ExposureTime.SetValue(8333.333f);
      cam_ptr->ExposureTime.SetValue(16667.666666667f);
      std::cout << "Set exposure time to [micro s]: " << cam_ptr->ExposureTime.GetValue() << std::endl;
    }
    else
    {
      std::cout << "Could not set exposure time" << std::endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  // Set exposure time
  try
  {
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
  }

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