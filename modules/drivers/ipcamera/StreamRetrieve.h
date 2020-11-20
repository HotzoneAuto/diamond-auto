#include "Infra/Thread.h"
#include "GenICam/StreamSource.h"
#include "GenICam/Frame.h"
#include "Media/RecordVideo.h"
#include "Media/ImageConvert.h"

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
