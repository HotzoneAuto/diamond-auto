#include <string>

#include "modules/sensors/wr_ls/wr_ls_common_tcp.h"
#include "modules/sensors/wr_ls/wr_ls1207de_parser.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wr_ls1207de");
	ros::NodeHandle nhPrivHome("~");

	/*Check whether hostname is provided*/
	bool isTcpConnection = false;
	std::string strHostName;
	std::string strPort;
	if(nhPrivHome.getParam("hostname", strHostName))
	{
		isTcpConnection = true;
		nhPrivHome.param<std::string>("port", strPort, "2112");
	}

	/*Get configured time limit*/
	int iTimeLimit = 5;
	nhPrivHome.param("timelimit", iTimeLimit, 5);

	bool isDataSubscribed = false;
	nhPrivHome.param("subscribe_datagram", isDataSubscribed, false);

	int iDeviceNumber = 0;
	nhPrivHome.param("device_number", iDeviceNumber, 0);

	/*Create and initialize parser*/
	wr_ls::CWrLs1207DEParser *pParser = new wr_ls::CWrLs1207DEParser();
	double param;

	if(nhPrivHome.param("range_min", param))
	{
		pParser->SetRangeMin(0.01);
	}
	if(nhPrivHome.param("range_max", param))
	{
		pParser->SetRangeMax(100.00);
	}
	if(nhPrivHome.param("time_increment", param))
	{
		pParser->SetTimeIncrement(param);
	}

	/*Setup TCP connection and attempt to connect/reconnect*/
	wr_ls::CWrLsCommon *pWrLs = NULL;
	int result = wr_ls::ExitError;
	while(ros::ok())
	{
		if(pWrLs != NULL)
		{
			delete pWrLs;
		}

		pWrLs = new wr_ls::CWrLsCommonTcp(strHostName, strPort, iTimeLimit, pParser);
		result = pWrLs->Init();

		/*Device has been initliazed successfully*/
		while(ros::ok() && (result == wr_ls::ExitSuccess))
		{
			ros::spinOnce();
			result = pWrLs->LoopOnce();
		}

		if(result == wr_ls::ExitFatal)
		{
			return result;
		}
	}

	if(pWrLs != NULL)
	{
		delete pWrLs;
	}

	if(pParser != NULL)
	{
		delete pParser;
	}

	return result;
}
