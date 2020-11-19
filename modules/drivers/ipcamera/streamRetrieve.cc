#include "modules/drivers/ipcamera/StreamRetrieve.h"

// #include "opencv2/highgui.hpp"
// using namespace cv;
StreamRetrieve::StreamRetrieve(IStreamSourcePtr& streamSptr)
	: CThread("streamRetrieve")
	, m_isLoop(false)
	, m_streamSptr(streamSptr)
{
	
}

bool StreamRetrieve::start()
{
	m_isLoop = true;
	return createThread();
}

bool StreamRetrieve::stop()
{
	m_isLoop = false;
	m_streamSptr.reset();
	return destroyThread();
}

void StreamRetrieve::threadProc()
{
	
	while (m_isLoop)
	{
		// 此frame对象必须为临时变量，对象的生命周期与驱动帧缓存相关。
		// 此对象生命周期结束意味着:驱动可以回收此帧缓存并用于存放后续获取到的帧。
		// 如没有及时释放，获取到的帧与设置的帧缓存相同时，将无法获取到帧（因为驱动已没有可用的帧缓存）。
		// This frame object must be a temporary variable, and its lifecycle is related to the drive frame cache.
		// The end of this object's life cycle means that the driver can reclaim this frame cache and use it to store subsequent acquired frames.
		// If it is not released in time, the acquired frame will be the same as the set frame cache, and the frame will not be acquired (because the driver has no available frame cache).	
		CFrame frame;

		// 获取一帧
	    // Get one frame
		if (!m_streamSptr)
		{
			printf("m_streamPtr is NULL.\n");
			return;
		}
		bool isSuccess = m_streamSptr->getFrame(frame, 300);
		if (!isSuccess)
		{
			printf("getFrame  fail.\n");
			continue;
		}
		// int nBGRBufferSize = frame.getImageWidth() * frame.getImageHeight() * 3;
		// 	uint8_t *pBGRbuffer = new uint8_t[nBGRBufferSize];
		// cv::Mat image = cv::Mat(frame.getImageHeight(),
		// 	frame.getImageWidth(),
		// 	CV_8UC3,
		// 	(uint8_t*)pBGRbuffer);
		// std::cout <<"====="<< (uint8_t*)pBGRbuffer << "\n";
		// imshow("output",image);
		// cv::waitKey(1);
		// 判断帧的有效性
		// Judge the validity of frame
		bool isValid = frame.valid();
		if (!isValid)
		{
			printf("frame is invalid!\n");
			continue;
		}

		// 打印FrameID
		// print frame ID
		// printf("get frame %lld successfully thread ID :%d\n", frame.getBlockId(), CThread::getCurrentThreadID());
	}

}


