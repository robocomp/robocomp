#ifndef LOGGER_ICE
#define LOGGER_ICE

/** \mainpage RoboComp Interfaces: Logger.ice
 *
 * \section intro_sec Introduction
* Interface for LoggerComp.  
*                                                                   
* Component text output logging.
*
*    PORT 10070 <br>   
*/

/** \namespace RoboCompLogger
  *@brief Name space logger interface.
  */

module RoboCompLogger
{
   /** \struct LogMessage
  *@brief struct of the log message.
  */
  struct LogMessage
  {
    string sender;
    string method;
    string file;
    int line;
    string timeStamp;
    string message;
    ///info, error, debug,
    string type;	
    string fullpath;
  };

	/** \interface Logger
  *@brief interface Logger
  */ 
  interface Logger
  {
    void sendMessage(LogMessage m);
  
  };
};

#endif
