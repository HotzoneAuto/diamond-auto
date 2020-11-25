//****** 本Demo 简单演示SDK 发现相机，连接相机，取图，断开相机的使用********//
//****** This demo simply demonstrates the use of SDK to discover cameras,
//connect cameras, get frame, and disconnect cameras ********//

#include "ipcamera_main.h"

// ********************** 这部分处理与SDK操作相机无关，用于显示设备列表
// end*****************************
// *** This part of the processing is independent of the SDK operation camera
// and is used to display the device list**
int main() {
  // g++  -I./include -c -Wall -g -m64 streamRetrieve.cpp -o streamRetrieve.o
  // g++  -I./include -c -Wall -g -m64 main.cpp -o main.o
  // g++  -I./include -Wall -g -m64  streamRetrieve.o  main.o -L./../../lib
  // -lMVSDK -lRecordVideo -lImageConvert -o sample

  // g++  -I./include -c -Wall -g -m64 main.cpp -c main.o
  // g++  -I./include -c -Wall -g -m64 maintest.cpp -o maintest.o
  // g++  -I./include -Wall -g -m64 streamRetrieve.o maintest.o -L./../../lib
  // -lMVSDK -lRecordVideo -lImageConvert -o samples

  // g++  -I/opt/apollo/pkgs/ipcamera/include -c -Wall -g -m64 maintest.cpp -o
  // maintest.o g++  -I/opt/apollo/pkgs/ipcamera/include -Wall -g -m64
  // maintest.o -L/opt/apollo/pkgs/ipcamera/lib/ -lMVSDK -lRecordVideo
  // -lImageConvert -o samples
  ICameraPtr cameraSptr;

  // 发现设备
  // discovery device
  CSystem &systemObj = CSystem::getInstance();
  TVector<ICameraPtr> vCameraPtrList;
  bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
  if (!isDiscoverySuccess) {
    printf("discovery device fail.\n");
    return 0;
  }

  if (vCameraPtrList.size() == 0) {
    printf("no devices.\n");
    return 0;
  }
  // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
  // print camera info (index,Type,vendor name, model,serial
  // number,DeviceUserID,IP Address)
  displayDeviceInfo(vCameraPtrList);
  int cameraIndex = selectDevice(vCameraPtrList.size());
  cameraSptr = vCameraPtrList[cameraIndex];

  // GigE相机时，连接前设置相机Ip与网卡处于同一网段上
  // When a GigE camera is connected, set the camera IP to be on the same
  // network segment as the network card
  if (ICamera::typeGige == cameraSptr->getType()) {
    if (autoSetCameraIP(cameraSptr) != 0) {
      printf("set camera Ip failed.\n");
    }
  }

  // 连接相机
  // connect camera
  if (!cameraSptr->connect()) {
    printf("connect cameral failed.\n");
    return 0;
  }

  // 设置相机为连续取流模式
  // set camera to continue grab mode
  setGrabMode(cameraSptr, true);
  // 创建流对象
  // create acquisitioncontrol object
  IStreamSourcePtr streamPtr = systemObj.createStreamSource(cameraSptr);
  if (NULL == streamPtr) {
    printf("create stream obj  fail.\r\n");
    return 0;
  }

  // 开始取图
  // start grabbing
  bool isStartGrabbingSuccess = streamPtr->startGrabbing();
  if (!isStartGrabbingSuccess) {
    printf("StartGrabbing  fail.\n");
  }

  // 建取流线程
  // create get frame thread
  // Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new
  // StreamRetrieve(streamPtr)); if (NULL == streamThreadSptr)
  // {
  //     printf("create thread obj failed.\n");
  //     return 0;
  // }

  //  // 线程开始取图
  //  // start get frame thread
  // streamThreadSptr->start();

  //  // 取图2秒
  //  // get frame 2 seconds
  // CThread::sleep(2000);

  // // 停止拉流线程
  // // Stop streaming thread
  // streamThreadSptr->stop();

  // 停止相机拉流
  // stop camera grabbing
  streamPtr->stopGrabbing();

  // 修改相机曝光时间
  // Modify camera exposure time
  modifyCamralExposureTime(systemObj, cameraSptr);

  // 断开设备
  // disconnect device
  if (!cameraSptr->disConnect()) {
    printf("disConnect camera failed\n");
    return 0;
  }

  printf("disConnect successfully thread ID :%d\n",
         CThread::getCurrentThreadID());
  return 1;
}
