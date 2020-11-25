#include "GenICam/Frame.h"
#include "GenICam/StreamSource.h"
#include "Infra/Thread.h"
#include "Media/ImageConvert.h"
#include "Media/RecordVideo.h"

// #include "modules/drivers/proto/ipcamera.pb.h"
// #include "modules/drivers/proto/sensor_image.pb.h
// #include "modules/drivers/ipcamera/proto/ipcamera_conf.pb.h"

// using apollo::drivers::IPCAMERA;
// using apollo::cyber::Writer;
// using apollo::drivers::Image;

using namespace Dahua::GenICam;
using namespace Dahua::Infra;

class StreamRetrieve : public CThread {
 public:
  StreamRetrieve(IStreamSourcePtr& streamSptr);
  bool start();
  bool stop();

 private:
  void threadProc();
  bool m_isLoop;
  IStreamSourcePtr m_streamSptr;
};
