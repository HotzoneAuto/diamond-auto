#ifndef MODULES_DRIVERS_IPCAMERA_IPCAMERA_MAIN_H_
#define MODULES_DRIVERS_IPCAMERA_IPCAMERA_MAIN_H_

#include "arpa/inet.h"

#include "GenICam/Camera.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "GenICam/StreamSource.h"
#include "GenICam/System.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"

#include "GenICam/Frame.h"
#include "Infra/Thread.h"
#include "Media/ImageConvert.h"
#include "Media/RecordVideo.h"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;

namespace apollo {
namespace drivers {
namespace ipcamera {
class Ipcamera {
 public:
  // 设置相机采图模式（连续采图、触发采图）
  // Set camera acquisition mode (continuous acquisition, triggered acquisition)
  static int32_t setGrabMode(ICameraPtr& cameraSptr, bool bContious);
  // 获取相机采图模式
  // Get camera acquisition mode
  static int32_t getGrabMode(ICameraPtr& cameraSptr, bool& bContious);
  // 软件触发
  // software trigger
  static int32_t triggerSoftware(ICameraPtr& cameraSptr);
  // 设置传感器采样率（采集分辨率）
  // Set sensor sampling rate (acquisition resolution)
  static int32_t setResolution(ICameraPtr& cameraSptr, int nWidth, int nHeight);
  // 获取传感器采样率
  // Get sensor sample rate
  static int32_t getResolution(ICameraPtr& cameraSptr, int64_t& nWidth,
                               int64_t& nHeight);
  // 设置binning (Off X Y XY)
  // set binning (Off X Y XY)
  static int32_t setBinning(ICameraPtr& cameraSptr);
  // 获取传感器最大分辩率
  // get the maximum resolution of the sensor
  static int32_t getMaxResolution(ICameraPtr& cameraSptr, int64_t& nWidthMax,
                                  int64_t& nHeightMax);
  static int32_t setROI(ICameraPtr& cameraSptr, int64_t nX, int64_t nY,
                        int64_t nWidth, int64_t nHeight);
  // 获取图像ROI
  // get image ROI
  static int32_t getROI(ICameraPtr& cameraSptr, int64_t& nX, int64_t& nY,
                        int64_t& nWidth, int64_t& nHeight);
  // 获取采图图像宽度
  // Get the width of the image
  static int32_t getWidth(ICameraPtr& cameraSptr, int64_t& nWidth);
  // 获取采图图像高度
  // Get the height of the image
  static int32_t getHeight(ICameraPtr& cameraSptr, int64_t& nHeight);
  // 设置曝光值(曝光、自动曝光/手动曝光)
  // Set exposure value (exposure, auto exposure / manual exposure)
  // static int32_t setExposureTime(ICameraPtr& cameraSptr,
  //                                   double dExposureTime,
  //                                   bool bAutoExposure = false);
  // 获取曝光时间
  // get exposureTime
  static int32_t getExposureTime(ICameraPtr& cameraSptr, double& dExposureTime);
  // 获取曝光范围
  // Get exposure range
  static int32_t getExposureTimeMinMaxValue(ICameraPtr& cameraSptr,
                                            double& dMinValue,
                                            double& dMaxValue);
  // 设置增益值
  // set gain
  static int32_t setGainRaw(ICameraPtr& cameraSptr, double dGainRaw);
  // 获取增益值
  // get gain value
  static int32_t getGainRaw(ICameraPtr& cameraSptr, double& dGainRaw);
  // 获取增益值范围
  // Get gain range
  static int32_t getGainRawMinMaxValue(ICameraPtr& cameraSptr,
                                       double& dMinValue, double& dMaxValue);

  // 设置伽马值
  // Set gamma
  static int32_t setGamma(ICameraPtr& cameraSptr, double dGamma);
  // 获取伽马值
  // Get gamma
  static int32_t getGamma(ICameraPtr& cameraSptr, double& dGamma);
  // 获取伽马值范围
  // Get gamma range
  static int32_t getGammaMinMaxValue(ICameraPtr& cameraSptr, double& dMinValue,
                                     double& dMaxValue);
  // 设置白平衡值（有三个白平衡值）
  // Set the white balance value ( three white balance values)
  static int32_t setBalanceRatio(ICameraPtr& cameraSptr,
                                 double dRedBalanceRatio,
                                 double dGreenBalanceRatio,
                                 double dBlueBalanceRatio);
  // 获取白平衡值（有三个白平衡值)
  // Get white balance value (three white balance values)
  static int32_t getBalanceRatio(ICameraPtr& cameraSptr,
                                 double& dRedBalanceRatio,
                                 double& dGreenBalanceRatio,
                                 double& dBlueBalanceRatio);
  // 获取白平衡值范围
  // Get white balance value range
  static int32_t getBalanceRatioMinMaxValue(ICameraPtr& cameraSptr,
                                            double& dMinValue,
                                            double& dMaxValue);
  // 设置采图速度（秒帧数）
  // Set the acquisition speed (seconds\frames)
  static int32_t setAcquisitionFrameRate(ICameraPtr& cameraSptr,
                                         double dFrameRate);
  // 获取采图速度（秒帧数）
  // Get the acquisition speed (seconds and frames)
  static int32_t getAcquisitionFrameRate(ICameraPtr& cameraSptr,
                                         double& dFrameRate);
  // 保存参数
  // Save parameters
  static int32_t userSetSave(ICameraPtr& cameraSptr);
  // 加载参数
  // Load parameters
  static int32_t loadUserSet(ICameraPtr& cameraSptr);
  // 设置外触发延时时间
  // set external trigger delay time
  static int32_t setTriggerDelay(ICameraPtr& cameraSptr, double dDelayTime);
  // 获取外触发延时时间
  // get external trigger delay time
  static int32_t getTriggerDelay(ICameraPtr& cameraSptr, double& dDelayTime);
  // 获取外触发模式（上升沿触发、下降沿触发）
  // Get external trigger mode (rising edge trigger, falling edge trigger)
  static int32_t getLineTriggerMode(ICameraPtr& cameraSptr, bool& bRisingEdge);
  // 设置外触发信号滤波时间
  // Set filtering time of external trigger signal
  static int32_t setLineDebouncerTimeAbs(ICameraPtr& cameraSptr,
                                         double dLineDebouncerTimeAbs);
  // 获取外触发信号滤波时间
  // Acquisition of filtering time of external trigger signal
  static int32_t getLineDebouncerTimeAbs(ICameraPtr& cameraSptr,
                                         double& dLineDebouncerTimeAbs);
  // 设置外触发脉冲宽度（不支持）  | Set external trigger width (not
  // supported)
  // 获取外触发脉冲宽度（不支持）  | Get external trigger width (not
  // supported) 设置输出信号线（控制光源用）（面阵相机是Line0） | Set the
  // output signal line (for controlling the light source) (the area array
  // camera is line0) 获取输出信号线（面阵相机是Line0） | get the output
  // signal line (the area array camera is line0)
  // 设置外部光源曝光时间（设置输出值为TRUE的时间） | Set the exposure time
  // of the external light source (set the time when the output value is
  // true)
  static int32_t setOutputTime(ICameraPtr& cameraSptr, int nTimeMS);
  //  获取外部光源曝光时间（输出信号的时间由软件侧控制） | get the exposure
  //  time of external light source (the time of output signal is controlled
  //  by the software side) 设置X轴翻转  | Set X-axis flip
  static int32_t setReverseX(ICameraPtr& cameraSptr, bool flag);
  // 设置Y轴翻转
  // Set X-axis flip
  static int32_t setReverseY(ICameraPtr& cameraSptr, bool flag);
  // 当相机与网卡处于不同网段时，自动设置相机IP与网卡处于同一网段
  // （与相机连接之前调用） When the camera and the network card are in
  // different network segments, automatically set the camera IP and the
  // network card in the same network segment (before calling the camera).
  static int32_t autoSetCameraIP(ICameraPtr& cameraSptr);
  // 设置相机IP （与相机连接之前调用）
  // Set up camera IP (before calling with camera)
  static int32_t setCameraIp(ICameraPtr& cameraSptr, char* ipAddress,
                             char* subnetMask, char* gateway);
  // 设置相机静态IP （与相机连接之后调用）
  // Set camera static IP (after calling with camera)
  static int32_t setCameraPersistentIP(ICameraPtr& cameraSptr);
  // 修改曝光时间 （与相机连接之后调用）
  // Modify exposure time (after calling connect camera)
  static void modifyCamralExposureTime(CSystem& systemObj,
                                       ICameraPtr& cameraSptr);
  void LogPrinterFunc(const char* log);
  // ********************** 这部分处理与SDK操作相机无关，用于显示设备列表
  // begin*****************************
  // ***********BEGIN: These functions are not related to API call and used
  // to display device info***********
  static void displayDeviceInfo(TVector<ICameraPtr>& vCameraPtrList);
  static char* trim(char* pStr);
  // 函数功能：判断pInpuStr字符串每个字符是否都为数字。
  // function: judge whether each character of pinpustr string is a number.
  // 该函数与SDK接口操作相机无关
  // this function is independent of the SDK interface operation camera
  static int isInputValid(char* pInpuStr);
  // 函数功能：从displayDeviceInfo显示的相机列表选择需要操作的相机对象。
  // function: select the camera object to be operated from the list of
  // cameras displayed in displaydeviceinfo 该函数与SDK接口操作相机无关 this
  // function is independent of the SDK interface operation camera
  static int selectDevice(int cameraCnt);
};
}  // namespace ipcamera
}  // namespace drivers
}  // namespace apollo
#endif  // MODULES_DRIVERS_IPCAMERA_IPCAMERA_MAIN_H_