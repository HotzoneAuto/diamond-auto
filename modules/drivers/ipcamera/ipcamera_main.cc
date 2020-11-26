#include "modules/drivers/ipcamera/ipcamera_main.h"

namespace apollo {
namespace drivers {
namespace ipcamera {
// 设置相机采图模式（连续采图、触发采图）
// Set camera acquisition mode (continuous acquisition, triggered acquisition)
int32_t Ipcamera::setGrabMode(ICameraPtr& cameraSptr, bool bContious) {
  int32_t bRet;
  IAcquisitionControlPtr sptrAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    return -1;
  }

  CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
  bRet = enumNode.setValueBySymbol("FrameStart");
  if (false == bRet) {
    printf("set TriggerSelector fail.\n");
    return -1;
  }

  if (true == bContious) {
    enumNode = sptrAcquisitionControl->triggerMode();
    bRet = enumNode.setValueBySymbol("Off");
    if (false == bRet) {
      printf("set triggerMode fail.\n");
      return -1;
    }
  } else {
    enumNode = sptrAcquisitionControl->triggerMode();
    bRet = enumNode.setValueBySymbol("On");
    if (false == bRet) {
      printf("set triggerMode fail.\n");
      return -1;
    }

    // 设置触发源为软触发（硬触发为Line1）
    // Set trigger source as soft trigger (hard trigger as Line1)
    enumNode = sptrAcquisitionControl->triggerSource();
    bRet = enumNode.setValueBySymbol("Software");
    if (false == bRet) {
      printf("set triggerSource fail.\n");
      return -1;
    }
  }
  return 0;
}

// 获取相机采图模式
// Get camera acquisition mode
int32_t Ipcamera::getGrabMode(ICameraPtr& cameraSptr, bool& bContious) {
  int32_t bRet;
  IAcquisitionControlPtr sptrAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    return -1;
  }

  CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
  bRet = enumNode.setValueBySymbol("FrameStart");
  if (false == bRet) {
    printf("set TriggerSelector fail.\n");
    return -1;
  }

  CString strValue;
  enumNode = sptrAcquisitionControl->triggerMode();
  bRet = enumNode.getValueSymbol(strValue);
  if (false == bRet) {
    printf("get triggerMode fail.\n");
    return -1;
  }

  if (strValue == "Off") {
    bContious = true;
  } else if (strValue == "On") {
    bContious = false;
  } else {
    printf("get triggerMode fail.\n");
    return -1;
  }
  return 0;
}

// 软件触发
// software trigger
int32_t Ipcamera::triggerSoftware(ICameraPtr& cameraSptr) {
  int32_t bRet;
  IAcquisitionControlPtr sptrAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    printf("AcquisitionControl fail.\n");
    return -1;
  }

  CCmdNode cmdNode = sptrAcquisitionControl->triggerSoftware();
  bRet = cmdNode.execute();
  if (false == bRet) {
    printf("triggerSoftware execute fail.\n");
    return -1;
  }
  return 0;
}

// 设置传感器采样率（采集分辨率）
// Set sensor sampling rate (acquisition resolution)
int32_t Ipcamera::setResolution(ICameraPtr& cameraSptr, int nWidth,
                                int nHeight) {
  int32_t bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  CIntNode intNode = sptrImageFormatControl->width();
  bRet = intNode.setValue(nWidth);
  if (false == bRet) {
    printf("set width fail.\n");
    return -1;
  }

  intNode = sptrImageFormatControl->height();
  bRet = intNode.setValue(nHeight);
  if (false == bRet) {
    printf("set height fail.\n");
    return -1;
  }
  return 0;
}

// 获取传感器采样率
// Get sensor sample rate
int32_t Ipcamera::getResolution(ICameraPtr& cameraSptr, int64_t& nWidth,
                                int64_t& nHeight) {
  int32_t bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  CIntNode intNode = sptrImageFormatControl->width();
  bRet = intNode.getValue(nWidth);
  if (false == bRet) {
    printf("get width fail.\n");
    return -1;
  }

  intNode = sptrImageFormatControl->height();
  bRet = intNode.getValue(nHeight);
  if (false == bRet) {
    printf("get height fail.\n");
    return -1;
  }
  return 0;
}

// 设置binning (Off X Y XY)
// set binning (Off X Y XY)
int32_t Ipcamera::setBinning(ICameraPtr& cameraSptr) {
  CEnumNodePtr ptrParam(new CEnumNode(cameraSptr, "Binning"));
  if (ptrParam) {
    if (false == ptrParam->isReadable()) {
      printf("binning not support.\n");
      return -1;
    }

    if (false == ptrParam->setValueBySymbol("XY")) {
      printf("set Binning XY fail.\n");
      return -1;
    }

    if (false == ptrParam->setValueBySymbol("Off")) {
      printf("set Binning Off fail.\n");
      return -1;
    }
  }
  return 0;
}

// 获取传感器最大分辩率
// get the maximum resolution of the sensor
int32_t Ipcamera::getMaxResolution(ICameraPtr& cameraSptr, int64_t& nWidthMax,
                                   int64_t& nHeightMax) {
  CIntNodePtr ptrParamSensorWidth(new CIntNode(cameraSptr, "SensorWidth"));
  if (ptrParamSensorWidth) {
    if (false == ptrParamSensorWidth->getValue(nWidthMax)) {
      printf("get WidthMax fail.\n");
      return -1;
    }
  }

  CIntNodePtr ptrParamSensorHeight(new CIntNode(cameraSptr, "SensorHeight"));
  if (ptrParamSensorHeight) {
    if (false == ptrParamSensorHeight->getValue(nWidthMax)) {
      printf("get WidthMax fail.\n");
      return -1;
    }
  }

  return 0;
}

// 设置图像ROI
// set image ROI
int32_t Ipcamera::setROI(ICameraPtr& cameraSptr, int64_t nX, int64_t nY,
                         int64_t nWidth, int64_t nHeight) {
  bool bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  // 设置宽
  // set width
  CIntNode intNode = sptrImageFormatControl->width();
  bRet = intNode.setValue(nWidth);
  if (!bRet) {
    printf("set width fail.\n");
    return -1;
  }

  // 设置长
  // set height
  intNode = sptrImageFormatControl->height();
  bRet = intNode.setValue(nHeight);
  if (!bRet) {
    printf("set height fail.\n");
    return -1;
  }

  // 设置X偏移
  // set OffsetX
  intNode = sptrImageFormatControl->offsetX();
  bRet = intNode.setValue(nX);
  if (!bRet) {
    printf("set offsetX fail.\n");
    return -1;
  }

  // 设置Y偏移
  // set OffsetY
  intNode = sptrImageFormatControl->offsetY();
  bRet = intNode.setValue(nY);
  if (!bRet) {
    printf("set offsetY fail.\n");
    return -1;
  }

  return 0;
}

// 获取图像ROI
// get image ROI
int32_t Ipcamera::getROI(ICameraPtr& cameraSptr, int64_t& nX, int64_t& nY,
                         int64_t& nWidth, int64_t& nHeight) {
  bool bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  // 设置宽
  // set width
  CIntNode intNode = sptrImageFormatControl->width();
  bRet = intNode.getValue(nWidth);
  if (!bRet) {
    printf("get width fail.\n");
  }

  // 设置长
  // set height
  intNode = sptrImageFormatControl->height();
  bRet = intNode.getValue(nHeight);
  if (!bRet) {
    printf("get height fail.\n");
  }

  // 设置X偏移
  // set OffsetX
  intNode = sptrImageFormatControl->offsetX();
  bRet = intNode.getValue(nX);
  if (!bRet) {
    printf("get offsetX fail.\n");
  }

  // 设置Y偏移
  // set OffsetY
  intNode = sptrImageFormatControl->offsetY();
  bRet = intNode.getValue(nY);
  if (!bRet) {
    printf("get offsetY fail.\n");
  }
  return 0;
}

// 获取采图图像宽度
// Get the width of the image
int32_t Ipcamera::getWidth(ICameraPtr& cameraSptr, int64_t& nWidth) {
  bool bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  CIntNode intNode = sptrImageFormatControl->width();
  bRet = intNode.getValue(nWidth);
  if (!bRet) {
    printf("get width fail.\n");
  }
  return 0;
}

// 获取采图图像高度
// Get the height of the image
int32_t Ipcamera::getHeight(ICameraPtr& cameraSptr, int64_t& nHeight) {
  bool bRet;
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);
  if (NULL == sptrImageFormatControl) {
    return -1;
  }

  CIntNode intNode = sptrImageFormatControl->height();
  bRet = intNode.getValue(nHeight);
  if (!bRet) {
    printf("get height fail.\n");
    return -1;
  }
  return 0;
}

// // 设置曝光值(曝光、自动曝光/手动曝光)
// // Set exposure value (exposure, auto exposure / manual exposure)
// int32_t Ipcamera::setExposureTime(ICameraPtr& cameraSptr, double
// dExposureTime,
//                                bool bAutoExposure = false) {
//   bool bRet;
//   IAcquisitionControlPtr sptrAcquisitionControl =
//       CSystem::getInstance().createAcquisitionControl(cameraSptr);
//   if (NULL == sptrAcquisitionControl) {
//     return -1;
//   }

//   if (bAutoExposure) {
//     CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
//     bRet = enumNode.setValueBySymbol("Continuous");
//     if (false == bRet) {
//       printf("set exposureAuto fail.\n");
//       return -1;
//     }
//   } else {
//     CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
//     bRet = enumNode.setValueBySymbol("Off");
//     if (false == bRet) {
//       printf("set exposureAuto fail.\n");
//       return -1;
//     }

//     CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
//     bRet = doubleNode.setValue(dExposureTime);
//     if (false == bRet) {
//       printf("set exposureTime fail.\n");
//       return -1;
//     }
//   }
//   return 0;
// }

// 获取曝光时间
// get exposureTime
int32_t Ipcamera::getExposureTime(ICameraPtr& cameraSptr,
                                  double& dExposureTime) {
  bool bRet;
  IAcquisitionControlPtr sptrAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
  bRet = doubleNode.getValue(dExposureTime);
  if (false == bRet) {
    printf("get exposureTime fail.\n");
    return -1;
  }
  return 0;
}

// 获取曝光范围
// Get exposure range
int32_t Ipcamera::getExposureTimeMinMaxValue(ICameraPtr& cameraSptr,
                                             double& dMinValue,
                                             double& dMaxValue) {
  bool bRet;
  IAcquisitionControlPtr sptrAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
  bRet = doubleNode.getMinVal(dMinValue);
  if (false == bRet) {
    printf("get exposureTime minValue fail.\n");
    return -1;
  }

  bRet = doubleNode.getMaxVal(dMaxValue);
  if (false == bRet) {
    printf("get exposureTime maxValue fail.\n");
    return -1;
  }
  return 0;
}

// 设置增益值
// set gain
int32_t Ipcamera::setGainRaw(ICameraPtr& cameraSptr, double dGainRaw) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
  bRet = doubleNode.setValue(dGainRaw);
  if (false == bRet) {
    printf("set gainRaw fail.\n");
    return -1;
  }
  return 0;
}

// 获取增益值
// get gain value
int32_t Ipcamera::getGainRaw(ICameraPtr& cameraSptr, double& dGainRaw) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
  bRet = doubleNode.getValue(dGainRaw);
  if (false == bRet) {
    printf("get gainRaw fail.\n");
    return -1;
  }
  return 0;
}

// 获取增益值范围
// Get gain range
int32_t Ipcamera::getGainRawMinMaxValue(ICameraPtr& cameraSptr,
                                        double& dMinValue, double& dMaxValue) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
  bRet = doubleNode.getMinVal(dMinValue);
  if (false == bRet) {
    printf("get gainRaw minValue fail.\n");
    return -1;
  }

  bRet = doubleNode.getMaxVal(dMaxValue);
  if (false == bRet) {
    printf("get gainRaw maxValue fail.\n");
    return -1;
  }
  return 0;
}

// 设置伽马值
// Set gamma
int32_t Ipcamera::setGamma(ICameraPtr& cameraSptr, double dGamma) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gamma();
  bRet = doubleNode.setValue(dGamma);
  if (false == bRet) {
    printf("set gamma fail.\n");
    return -1;
  }
  return 0;
}

// 获取伽马值
// Get gamma
int32_t Ipcamera::getGamma(ICameraPtr& cameraSptr, double& dGamma) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gamma();
  bRet = doubleNode.getValue(dGamma);
  if (false == bRet) {
    printf("get gamma fail.\n");
    return -1;
  }
  return 0;
}

// 获取伽马值范围
// Get gamma range
int32_t Ipcamera::getGammaMinMaxValue(ICameraPtr& cameraSptr, double& dMinValue,
                                      double& dMaxValue) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->gamma();
  bRet = doubleNode.getMinVal(dMinValue);
  if (false == bRet) {
    printf("get gamma minValue fail.\n");
    return -1;
  }

  bRet = doubleNode.getMaxVal(dMaxValue);
  if (false == bRet) {
    printf("get gamma maxValue fail.\n");
    return -1;
  }
  return 0;
}

// 设置白平衡值（有三个白平衡值）
// Set the white balance value ( three white balance values)
int32_t Ipcamera::setBalanceRatio(ICameraPtr& cameraSptr,
                                  double dRedBalanceRatio,
                                  double dGreenBalanceRatio,
                                  double dBlueBalanceRatio) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  // 关闭自动白平衡
  // Turn off auto white balance
  CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
  if (false == enumNode.isReadable()) {
    printf("balanceRatio not support.\n");
    return -1;
  }

  bRet = enumNode.setValueBySymbol("Off");
  if (false == bRet) {
    printf("set balanceWhiteAuto Off fail.\n");
    return -1;
  }

  enumNode = sptrAnalogControl->balanceRatioSelector();
  bRet = enumNode.setValueBySymbol("Red");
  if (false == bRet) {
    printf("set red balanceRatioSelector fail.\n");
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.setValue(dRedBalanceRatio);
  if (false == bRet) {
    printf("set red balanceRatio fail.\n");
    return -1;
  }

  enumNode = sptrAnalogControl->balanceRatioSelector();
  bRet = enumNode.setValueBySymbol("Green");
  if (false == bRet) {
    printf("set green balanceRatioSelector fail.\n");
    return -1;
  }

  doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.setValue(dGreenBalanceRatio);
  if (false == bRet) {
    printf("set green balanceRatio fail.\n");
    return -1;
  }

  enumNode = sptrAnalogControl->balanceRatioSelector();
  bRet = enumNode.setValueBySymbol("Blue");
  if (false == bRet) {
    printf("set blue balanceRatioSelector fail.\n");
    return -1;
  }

  doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.setValue(dBlueBalanceRatio);
  if (false == bRet) {
    printf("set blue balanceRatio fail.\n");
    return -1;
  }
  return 0;
}

// 获取白平衡值（有三个白平衡值)
// Get white balance value (three white balance values)
int32_t Ipcamera::getBalanceRatio(ICameraPtr& cameraSptr,
                                  double& dRedBalanceRatio,
                                  double& dGreenBalanceRatio,
                                  double& dBlueBalanceRatio) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CEnumNode enumNode = sptrAnalogControl->balanceRatioSelector();
  if (false == enumNode.isReadable()) {
    printf("balanceRatio not support.\n");
    return -1;
  }

  bRet = enumNode.setValueBySymbol("Red");
  if (false == bRet) {
    printf("set red balanceRatioSelector fail.\n");
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.getValue(dRedBalanceRatio);
  if (false == bRet) {
    printf("get red balanceRatio fail.\n");
    return -1;
  }

  enumNode = sptrAnalogControl->balanceRatioSelector();
  bRet = enumNode.setValueBySymbol("Green");
  if (false == bRet) {
    printf("set green balanceRatioSelector fail.\n");
    return -1;
  }

  doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.getValue(dGreenBalanceRatio);
  if (false == bRet) {
    printf("get green balanceRatio fail.\n");
    return -1;
  }

  enumNode = sptrAnalogControl->balanceRatioSelector();
  bRet = enumNode.setValueBySymbol("Blue");
  if (false == bRet) {
    printf("set blue balanceRatioSelector fail.\n");
    return -1;
  }

  doubleNode = sptrAnalogControl->balanceRatio();
  bRet = doubleNode.getValue(dBlueBalanceRatio);
  if (false == bRet) {
    printf("get blue balanceRatio fail.\n");
    return -1;
  }
  return 0;
}

// 获取白平衡值范围
// Get white balance value range
int32_t Ipcamera::getBalanceRatioMinMaxValue(ICameraPtr& cameraSptr,
                                             double& dMinValue,
                                             double& dMaxValue) {
  bool bRet;
  IAnalogControlPtr sptrAnalogControl =
      CSystem::getInstance().createAnalogControl(cameraSptr);
  if (NULL == sptrAnalogControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
  if (false == doubleNode.isReadable()) {
    printf("balanceRatio not support.\n");
    return -1;
  }

  bRet = doubleNode.getMinVal(dMinValue);
  if (false == bRet) {
    printf("get balanceRatio min value fail.\n");
    return -1;
  }

  bRet = doubleNode.getMaxVal(dMaxValue);
  if (false == bRet) {
    printf("get balanceRatio max value fail.\n");
    return -1;
  }

  return 0;
}

// 设置采图速度（秒帧数）
// Set the acquisition speed (seconds\frames)
int32_t Ipcamera::setAcquisitionFrameRate(ICameraPtr& cameraSptr,
                                          double dFrameRate) {
  bool bRet;
  IAcquisitionControlPtr sptAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptAcquisitionControl) {
    return -1;
  }

  CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
  bRet = booleanNode.setValue(true);
  if (false == bRet) {
    printf("set acquisitionFrameRateEnable fail.\n");
    return -1;
  }

  CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
  bRet = doubleNode.setValue(dFrameRate);
  if (false == bRet) {
    printf("set acquisitionFrameRate fail.\n");
    return -1;
  }

  return 0;
}

// 获取采图速度（秒帧数）
// Get the acquisition speed (seconds and frames)
int32_t Ipcamera::getAcquisitionFrameRate(ICameraPtr& cameraSptr,
                                          double& dFrameRate) {
  bool bRet;
  IAcquisitionControlPtr sptAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptAcquisitionControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
  bRet = doubleNode.getValue(dFrameRate);
  if (false == bRet) {
    printf("get acquisitionFrameRate fail.\n");
    return -1;
  }

  return 0;
}

// 保存参数
// Save parameters
int32_t Ipcamera::userSetSave(ICameraPtr& cameraSptr) {
  bool bRet;
  IUserSetControlPtr sptUserSetControl =
      CSystem::getInstance().createUserSetControl(cameraSptr);
  if (NULL == sptUserSetControl) {
    return -1;
  }

  bRet = sptUserSetControl->saveUserSet(IUserSetControl::userSet1);
  if (false == bRet) {
    printf("saveUserSet fail.\n");
    return -1;
  }

  return 0;
}

// 加载参数
// Load parameters
int32_t Ipcamera::loadUserSet(ICameraPtr& cameraSptr) {
  bool bRet;
  IUserSetControlPtr sptUserSetControl =
      CSystem::getInstance().createUserSetControl(cameraSptr);
  if (NULL == sptUserSetControl) {
    return -1;
  }

  bRet = sptUserSetControl->setCurrentUserSet(IUserSetControl::userSet1);
  if (false == bRet) {
    printf("saveUserSet fail.\n");
    return -1;
  }

  return 0;
}

// 设置外触发延时时间
// set external trigger delay time
int32_t Ipcamera::setTriggerDelay(ICameraPtr& cameraSptr, double dDelayTime) {
  bool bRet;
  IAcquisitionControlPtr sptAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptAcquisitionControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
  bRet = doubleNode.setValue(dDelayTime);
  if (false == bRet) {
    printf("set triggerDelay fail.\n");
    return -1;
  }

  return 0;
}

// 获取外触发延时时间
// get external trigger delay time
int32_t Ipcamera::getTriggerDelay(ICameraPtr& cameraSptr, double& dDelayTime) {
  bool bRet;
  IAcquisitionControlPtr sptAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptAcquisitionControl) {
    return -1;
  }

  CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
  bRet = doubleNode.getValue(dDelayTime);
  if (false == bRet) {
    printf("set triggerDelay fail.\n");
    return -1;
  }

  return 0;
}

// 设置外触发模式（上升沿触发、下降沿触发）
// Set external trigger mode (rising edge trigger, falling edge trigger)
// static int32_t setLineTriggerMode(ICameraPtr& cameraSptr, bool bRisingEdge)
// {
//     bool bRet;
//     IAcquisitionControlPtr sptAcquisitionControl =
//     CSystem::getInstance().createAcquisitionControl(cameraSptr); if (NULL ==
//     sptAcquisitionControl)
//     {
//         return -1;
//     }

//     CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
//     if (false == enumNode.setValueBySymbol("FrameStart"))
//     {
//         printf("set triggerSelector fail.\n");
//         return -1;
//     }

//     enumNode = sptAcquisitionControl->triggerMode();
//     if (false == enumNode.setValueBySymbol("On"))
//     {
//         printf("set triggerMode fail.\n");
//         return -1;
//     }

//     enumNode = sptAcquisitionControl->triggerSource();
//     if (false == enumNode.setValueBySymbol("Line1"))
//     {
//         printf("set triggerSource fail.\n");
//         return -1;
//     }

//     enumNode = sptAcquisitionControl->triggerActivation();
//     if (true == bRisingEdge)
//     {
//         bRet = enumNode.setValueBySymbol("RisingEdge");
//     }
//     else
//     {
//         bRet = enumNode.setValueBySymbol("FallingEdge");
//     }

//     return 0;
// }

// 获取外触发模式（上升沿触发、下降沿触发）
// Get external trigger mode (rising edge trigger, falling edge trigger)
int32_t Ipcamera::getLineTriggerMode(ICameraPtr& cameraSptr,
                                     bool& bRisingEdge) {
  bool bRet;
  IAcquisitionControlPtr sptAcquisitionControl =
      CSystem::getInstance().createAcquisitionControl(cameraSptr);
  if (NULL == sptAcquisitionControl) {
    return -1;
  }

  CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
  if (false == enumNode.setValueBySymbol("FrameStart")) {
    printf("set triggerSelector fail.\n");
    return -1;
  }

  CString strValue;
  enumNode = sptAcquisitionControl->triggerActivation();
  if (true == bRisingEdge) {
    bRet = enumNode.getValueSymbol(strValue);
  } else {
    bRet = enumNode.getValueSymbol(strValue);
  }

  if (false == bRet) {
    printf("get triggerActivation fail.\n");
    return -1;
  }

  if (strValue == "RisingEdge") {
    bRisingEdge = true;
  } else if (strValue == "FallingEdge") {
    bRisingEdge = false;
  } else {
    printf("get triggerActivation fail.\n");
    return -1;
  }

  return 0;
}

// 设置外触发信号滤波时间
// Set filtering time of external trigger signal
int32_t Ipcamera::setLineDebouncerTimeAbs(ICameraPtr& cameraSptr,
                                          double dLineDebouncerTimeAbs) {
  IDigitalIOControlPtr sptDigitalIOControl =
      CSystem::getInstance().createDigitalIOControl(cameraSptr);
  if (NULL == sptDigitalIOControl) {
    return -1;
  }

  CEnumNode enumNode = sptDigitalIOControl->lineSelector();
  if (false == enumNode.setValueBySymbol("Line1")) {
    printf("set lineSelector fail.\n");
    return -1;
  }

  CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
  if (false == doubleNode.setValue(dLineDebouncerTimeAbs)) {
    printf("set lineDebouncerTimeAbs fail.\n");
    return -1;
  }

  return 0;
}

// 获取外触发信号滤波时间
// Acquisition of filtering time of external trigger signal
int32_t Ipcamera::getLineDebouncerTimeAbs(ICameraPtr& cameraSptr,
                                          double& dLineDebouncerTimeAbs) {
  IDigitalIOControlPtr sptDigitalIOControl =
      CSystem::getInstance().createDigitalIOControl(cameraSptr);
  if (NULL == sptDigitalIOControl) {
    return -1;
  }

  CEnumNode enumNode = sptDigitalIOControl->lineSelector();
  if (false == enumNode.setValueBySymbol("Line1")) {
    printf("set lineSelector fail.\n");
    return -1;
  }

  CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
  if (false == doubleNode.getValue(dLineDebouncerTimeAbs)) {
    printf("get lineDebouncerTimeAbs fail.\n");
    return -1;
  }

  return 0;
}

// 设置外触发脉冲宽度（不支持）  | Set external trigger width (not supported)
// 获取外触发脉冲宽度（不支持）  | Get external trigger width (not supported)
// 设置输出信号线（控制光源用）（面阵相机是Line0） | Set the output signal line
// (for controlling the light source) (the area array camera is line0)
// 获取输出信号线（面阵相机是Line0） | get the output signal line (the area
// array camera is line0) 设置外部光源曝光时间（设置输出值为TRUE的时间） | Set
// the exposure time of the external light source (set the time when the output
// value is true)
int32_t Ipcamera::setOutputTime(ICameraPtr& cameraSptr, int nTimeMS) {
  IDigitalIOControlPtr sptDigitalIOControl =
      CSystem::getInstance().createDigitalIOControl(cameraSptr);
  if (NULL == sptDigitalIOControl) {
    return -1;
  }

  CEnumNode paramLineSource(cameraSptr, "LineSource");
  if (false == paramLineSource.setValueBySymbol("UserOutput1")) {
    printf("set LineSource fail.");
    return -1;
  }

  // 将输出信号拉高然后拉低
  // Pull the output signal up and down
  CBoolNode booleanNode = sptDigitalIOControl->userOutputValue();
  if (false == booleanNode.setValue(true)) {
    printf("set userOutputValue fail.\n");
    return -1;
  }

  CThread::sleep(nTimeMS);

  if (false == booleanNode.setValue(false)) {
    printf("set userOutputValue fail.\n");
    return -1;
  }

  return 0;
}

//  获取外部光源曝光时间（输出信号的时间由软件侧控制） | get the exposure time
//  of external light source (the time of output signal is controlled by the
//  software side) 设置X轴翻转  | Set X-axis flip
int32_t Ipcamera::setReverseX(ICameraPtr& cameraSptr, bool flag) {
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);

  CBoolNode boolNodeReverseX = sptrImageFormatControl->reverseX();
  if (!boolNodeReverseX.setValue(flag)) {
    printf("set reverseX fail.\n");
    return -1;
  }

  return 0;
}

// 设置Y轴翻转
// Set X-axis flip
int32_t Ipcamera::setReverseY(ICameraPtr& cameraSptr, bool flag) {
  IImageFormatControlPtr sptrImageFormatControl =
      CSystem::getInstance().createImageFormatControl(cameraSptr);

  CBoolNode boolNodeReverseY = sptrImageFormatControl->reverseY();
  if (!boolNodeReverseY.setValue(flag)) {
    printf("set reverseY fail.\n");
    return -1;
  }

  return 0;
}

// 当相机与网卡处于不同网段时，自动设置相机IP与网卡处于同一网段
// （与相机连接之前调用） When the camera and the network card are in different
// network segments, automatically set the camera IP and the network card in the
// same network segment (before calling the camera).
int32_t Ipcamera::autoSetCameraIP(ICameraPtr& cameraSptr) {
  IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
  if (NULL == gigeCameraPtr) {
    return -1;
  }

  // 获取Gige相机相关信息
  // Get GigE camera information
  CString ip = gigeCameraPtr->getIpAddress();
  CString subnetMask = gigeCameraPtr->getSubnetMask();
  CString gateway = gigeCameraPtr->getGateway();
  CString macAddress = gigeCameraPtr->getMacAddress();
  printf("ip address is %s.\r\n", ip.c_str());
  printf("subnetMask is %s.\r\n", subnetMask.c_str());
  printf("gateway is %s.\r\n", gateway.c_str());
  printf("macAddress is %s.\r\n", macAddress.c_str());
  printf("\n");

  unsigned long devIpValue =
      ntohl(inet_addr(gigeCameraPtr->getIpAddress().c_str()));
  unsigned long devSubMaskValue =
      ntohl(inet_addr(gigeCameraPtr->getSubnetMask().c_str()));

  // 获取对应接口的网卡信息
  // Get the network card information of the corresponding interface
  IGigEInterfacePtr gigeInterfaceSPtr = IGigEInterface::getInstance(cameraSptr);
  if (NULL == gigeInterfaceSPtr) {
    return -1;
  }

  CString interfaceIp = gigeInterfaceSPtr->getIpAddress();
  CString interfaceSubnetMask = gigeInterfaceSPtr->getSubnetMask();
  CString interfaceGateway = gigeInterfaceSPtr->getGateway();
  CString interfaceMacAddress = gigeInterfaceSPtr->getMacAddress();
  printf("ip address of interface  is %s.\r\n", interfaceIp.c_str());
  printf("subnetMask of interface is %s.\r\n", interfaceSubnetMask.c_str());
  printf("gateway of interface is %s.\r\n", interfaceGateway.c_str());
  printf("macAddress of interface is %s.\r\n", interfaceMacAddress.c_str());
  printf("\n");

  unsigned long InterfaceIpValue =
      ntohl(inet_addr(gigeInterfaceSPtr->getIpAddress().c_str()));
  unsigned long InterfaceSubMaskValue =
      ntohl(inet_addr(gigeInterfaceSPtr->getSubnetMask().c_str()));

  if ((devIpValue & devSubMaskValue) !=
      (InterfaceIpValue & InterfaceSubMaskValue)) {
    // 设备与网卡不在同一网段，强制设置设备与网卡在同一网段
    // The device and network card are not in the same network segment. It is
    // mandatory to set the device and network card in the same network segment
    unsigned char newIPStr[20] = {0};

    while (1) {
      unsigned long newIpValue = rand() % 254 + 1;  // 1~254
      if (newIpValue != (InterfaceIpValue & 0xff)) {
        newIpValue = (InterfaceIpValue & 0xffffff00) + newIpValue;
        struct in_addr stInAddr;
        stInAddr.s_addr = htonl(newIpValue);
        memcpy(newIPStr, inet_ntoa(stInAddr), strlen(inet_ntoa(stInAddr)));
        break;
      }
    }

    if (!gigeCameraPtr->forceIpAddress(
            (const char*)newIPStr, gigeInterfaceSPtr->getSubnetMask().c_str(),
            gigeInterfaceSPtr->getGateway().c_str())) {
      printf("Set device ip failed.\n");
      return -1;
    }
  }

  return 0;
}

// 设置相机IP （与相机连接之前调用）
// Set up camera IP (before calling with camera)
int32_t Ipcamera::setCameraIp(ICameraPtr& cameraSptr, char* ipAddress,
                              char* subnetMask, char* gateway) {
  IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
  if (NULL == gigeCameraPtr) {
    return -1;
  }

  if (!gigeCameraPtr->forceIpAddress(ipAddress, subnetMask, gateway)) {
    printf("Set device ip failed.\n");
    return -1;
  }

  return 0;
}

// 设置相机静态IP （与相机连接之后调用）
// Set camera static IP (after calling with camera)
int32_t Ipcamera::setCameraPersistentIP(ICameraPtr& cameraSptr) {
  IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
  if (NULL == gigeCameraPtr) {
    printf("gigeCameraPtr is null.\n");
    return -1;
  }

  ITransportLayerControlPtr transportLayerControlPtr =
      CSystem::getInstance().createTransportLayerControl(cameraSptr);

  if (NULL == transportLayerControlPtr) {
    printf("transportLayerControlPtr is null.\n");
    return -1;
  }

  transportLayerControlPtr->gevCurrentIPConfigurationPersistentIP().setValue(
      true);
  transportLayerControlPtr->gevPersistentDefaultGateway().setValue(
      gigeCameraPtr->getGateway().c_str());
  transportLayerControlPtr->gevPersistentIPAddress().setValue(
      gigeCameraPtr->getIpAddress().c_str());
  transportLayerControlPtr->gevPersistentSubnetMask().setValue(
      gigeCameraPtr->getSubnetMask().c_str());

  return 0;
}

// 修改曝光时间 （与相机连接之后调用）
// Modify exposure time (after calling connect camera)
void Ipcamera::modifyCamralExposureTime(CSystem& systemObj,
                                        ICameraPtr& cameraSptr) {
  IAcquisitionControlPtr sptrAcquisitionControl =
      systemObj.createAcquisitionControl(cameraSptr);
  if (NULL == sptrAcquisitionControl) {
    return;
  }

  double exposureTimeValue = 0.0;
  CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();

  exposureTime.getValue(exposureTimeValue);
  printf("before change ,exposureTime is %f. thread ID :%d\n",
         exposureTimeValue, CThread::getCurrentThreadID());

  exposureTime.setValue(exposureTimeValue + 2);
  exposureTime.getValue(exposureTimeValue);
  printf("after change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue,
         CThread::getCurrentThreadID());
}

void Ipcamera::LogPrinterFunc(const char* log) { return; }

// ********************** 这部分处理与SDK操作相机无关，用于显示设备列表
// begin*****************************
// ***********BEGIN: These functions are not related to API call and used to
// display device info***********
void Ipcamera::displayDeviceInfo(TVector<ICameraPtr>& vCameraPtrList) {
  ICameraPtr cameraSptr;
  // 打印Title行
  // Print title line
  printf(
      "\nIdx Type Vendor     Model      S/N             DeviceUserID    IP "
      "Address    \n");
  printf(
      "------------------------------------------------------------------------"
      "------\n");
  for (int cameraIndex = 0; cameraIndex < vCameraPtrList.size();
       cameraIndex++) {
    cameraSptr = vCameraPtrList[cameraIndex];
    // Idx 设备列表的相机索引 最大表示字数：3
    // Camera index in device list, display in 3 characters
    printf("%-3d", cameraIndex + 1);

    // Type 相机的设备类型（GigE，U3V，CL，PCIe
    // Camera type (eg:GigE，U3V，CL，PCIe)
    switch (cameraSptr->getType()) {
      case ICamera::typeGige:
        printf(" GigE");
        break;
      case ICamera::typeU3v:
        printf(" U3V ");
        break;
      case ICamera::typeCL:
        printf(" CL  ");
        break;
      case ICamera::typePCIe:
        printf(" PCIe");
        break;
      default:
        printf("     ");
        break;
    }

    // VendorName 制造商信息 最大表示字数：10
    // Camera vendor name, display in 10 characters
    const char* vendorName = cameraSptr->getVendorName();
    char vendorNameCat[11];
    if (strlen(vendorName) > 10) {
      strncpy(vendorNameCat, vendorName, 7);
      vendorNameCat[7] = '\0';
      strcat(vendorNameCat, "...");
      printf(" %-10.10s", vendorNameCat);
    } else {
      printf(" %-10.10s", vendorName);
    }

    // ModeName 相机的型号信息 最大表示字数：10
    // Camera model name, display in 10 characters
    printf(" %-10.10s", cameraSptr->getModelName());

    // Serial Number 相机的序列号 最大表示字数：15
    // Camera serial number, display in 15 characters
    printf(" %-15.15s", cameraSptr->getSerialNumber());

    // deviceUserID 自定义用户ID 最大表示字数：15
    // Camera user id, display in 15 characters
    const char* deviceUserID = cameraSptr->getName();
    char deviceUserIDCat[16] = {0};
    if (strlen(deviceUserID) > 15) {
      strncpy(deviceUserIDCat, deviceUserID, 12);
      deviceUserIDCat[12] = '\0';
      strcat(deviceUserIDCat, "...");
      printf(" %-15.15s", deviceUserIDCat);
    } else {
      // 防止console显示乱码,UTF8转换成ANSI进行显示
      // Prevent console from displaying garbled code and convert utf8 to ANSI
      // for display
      memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
      printf(" %-15.15s", deviceUserIDCat);
    }

    // IPAddress GigE相机时获取IP地址
    // IP address of GigE camera
    IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
    if (NULL != gigeCameraPtr.get()) {
      CString ip = gigeCameraPtr->getIpAddress();
      printf(" %s", ip.c_str());
    }
    printf("\n");
  }
}

char* Ipcamera::trim(char* pStr) {
  char* pDst = pStr;
  char* pTemStr = NULL;
  // int ret = -1;

  if (pDst != NULL) {
    pTemStr = pDst + strlen(pStr) - 1;
    // 除去字符串首部空格
    // remove the first space of the string
    while (*pDst == ' ') {
      pDst++;
    }
    // 除去字符串尾部空格
    // remove trailing space from string
    while ((pTemStr > pDst) && (*pTemStr == ' ')) {
      *pTemStr-- = '\0';
    }
  }
  return pDst;
}

// 函数功能：判断pInpuStr字符串每个字符是否都为数字。
// function: judge whether each character of pinpustr string is a number.
// 该函数与SDK接口操作相机无关
// this function is independent of the SDK interface operation camera
int Ipcamera::isInputValid(char* pInpuStr) {
  char numChar;
  char* pStr = pInpuStr;
  while (*pStr != '\0') {
    numChar = *pStr;
    if ((numChar > '9') || (numChar < '0')) {
      return -1;
    }
    pStr++;
  }
  return 0;
}

// 函数功能：从displayDeviceInfo显示的相机列表选择需要操作的相机对象。
// function: select the camera object to be operated from the list of cameras
// displayed in displaydeviceinfo 该函数与SDK接口操作相机无关 this function is
// independent of the SDK interface operation camera
int Ipcamera::selectDevice(int cameraCnt) {
  char inputStr[256] = {0};
  char* pTrimStr;
  char* find = NULL;
  int inputIndex = -1;
  int ret = -1;
  // 提示用户选择
  // inform user to select
  printf("\nPlease input the camera index: ");
  while (1) {
    // 获取输入内容字符串
    // get input string*/
    memset(inputStr, 0, sizeof(inputStr));
    fgets(inputStr, sizeof(inputStr), stdin);

    // 清空输入缓存
    // clear flush
    fflush(stdin);

    // fgets比gets多吃一个换行符号，取出换行符号
    // fgets eats one more line feed symbol than gets, and takes out the line
    // feed symbol
    find = strchr(inputStr, '\n');
    if (find) {
      *find = '\0';
    }

    // 除去字符串首尾空格
    // remove starting and trailing spaces from string
    pTrimStr = trim(inputStr);
    // 判断输入字符串是否为数字
    // judge whether the input string is a number
    ret = isInputValid(pTrimStr);
    if (ret == 0) {
      // 输入的字符串转换成为数字
      // the input string is converted to a number
      inputIndex = atoi(pTrimStr);
      // 判断用户选择合法性 */
      // judge the validity of user selection
      inputIndex -=
          1;  //显示的Index是从1开始 english: input index starts from 1
      if ((inputIndex >= 0) && (inputIndex < cameraCnt)) {
        break;
      }
    }

    printf("Input invalid! Please input the camera index: ");
  }
  return inputIndex;
}

}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo