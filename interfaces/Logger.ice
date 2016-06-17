#ifndef LOGGER_ICE
#define LOGGER_ICE

/** \mainpage RoboComp Interfaces: Logger.ice
 *
 * \section intro_sec Introduction
 * Interface for rclogger  
 *                                                                   
 */
module RoboCompLogger
{
	struct LogMessage
	{
		string sender;
		string method;
		string file;
		int line;
		string timeStamp;
		string message;
		string type;
		string fullpath;
	};

	interface Logger
	{
		void sendMessage(LogMessage m);
	};
};

#endif
