#include "modules/drivers/ipcamera/include/Infra/Thread.h"
#include "modules/drivers/ipcamera/include/GenICam/StreamSource.h"
#include "modules/drivers/ipcamera/include/GenICam/Frame.h"
#include "modules/drivers/ipcamera/include/Media/RecordVideo.h"
#include "modules/drivers/ipcamera/include/Media/ImageConvert.h"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;


class StreamRetrieve : public CThread
{
public:
	StreamRetrieve(IStreamSourcePtr& streamSptr);
	bool start();
	bool stop();

private:
	void threadProc();
	bool m_isLoop;
	IStreamSourcePtr m_streamSptr;
};
