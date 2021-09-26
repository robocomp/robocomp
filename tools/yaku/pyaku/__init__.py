import logging
import sys
from logging.handlers import TimedRotatingFileHandler

_outfmt = ' %(asctime)s.%(msecs)03d %(module)s (%(levelname)s): %(funcName)20s() %(message)s '
_outfmt_syslog = ' %(name)-11s (%(levelname)-7s): %(message)s '
_datefmt = "%Y-%m-%dT%H:%M:%S"
formatter = logging.Formatter(_outfmt, _datefmt)

logging.basicConfig(
    level=logging.DEBUG,
    filename='/dev/null',  # disabled
    format=_outfmt,
    datefmt=_datefmt
)



def get_console_handler():
   console_handler = logging.StreamHandler(sys.stdout)
   console_handler.setFormatter(formatter)
   return console_handler

def get_file_handler(logfile=None):
    if logfile is None:
        logfile = "./logfile.log"
    _handler_file = TimedRotatingFileHandler(logfile, when='midnight')
    _handler_file.setLevel(logging.DEBUG)
    _handler_file.setFormatter(formatter)
    return _handler_file

def get_logger(logger_name):
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.INFO) # better to have too much log than not enough
    logger.addHandler(get_console_handler())
    # logger.addHandler(get_file_handler())
    # with this pattern, it's rarely necessary to propagate the error up to parent
    logger.propagate = False
    return logger

logger = get_logger(__name__)
logger.log(100, 'LOG INIT')