#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#

#
# CODE BEGINS
#
import logging


# class Singleton(object):
#     _instance = None

class RCManagerLogger(object):
    """ This make this class a Singleton. To get the instance you just need to call the constructor anytime you need it."""
    _instance = None
    def __new__(class_, *args, **kwargs):
        if not isinstance(class_._instance, class_):
            class_._instance = object.__new__(class_, *args, **kwargs)
            class_._instance._initialized = False
        return class_._instance

    def __init__(self):
        if (self._initialized): return
        self._initialized = True
        super(RCManagerLogger, self).__init__()
        # create main logger
        self._logger = logging.getLogger()
        self.logging_level = logging.DEBUG
        self._logger.setLevel(self.logging_level)
        # create file handler which logs even debug messages
        self.file_handler = logging.FileHandler('rcmanager.log')
        self.file_handler.setLevel(self.logging_level)
        # create console handler with a higher log level
        self.console_handler = logging.StreamHandler()
        self.console_handler.setLevel(self.logging_level)
        # create formatter and add it to the handlers
        self.current_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.file_handler.setFormatter(self.current_format)
        self.console_handler.setFormatter(self.current_format)
        # add the handlers to the self._logger
        self._logger.addHandler(self.file_handler)
        self._logger.addHandler(self.console_handler)
        self._logger.info('Created main logger')
        self.text_edit_handler = None


    def get_logger(self, logger_name=''):
        return logging.getLogger(logger_name)

    def set_log_file(self, filepath):
        root_logger = logging.getLogger()
        root_logger.handlers[self.file_handler].stream.close()
        root_logger.removeHandler(self.file_handler)
        file_handler = logging.FileHandler(filepath)
        file_handler.setLevel(self.logging_level)
        file_handler.setFormatter(self.current_format)
        root_logger.addHandler(file_handler)


    def set_text_edit_handler(self, qtextedit):
        class QTextEditLogger(logging.Handler):
            def __init__(self, text_widget):
                super(QTextEditLogger, self).__init__()
                self.text_widget = text_widget

            def emit(self, record):
                self.append_line(self.format(record))  # implementation of append_line omitted

            def append_line(self, msg):
                self.text_widget.append(msg)
        self.text_edit_handler = QTextEditLogger(qtextedit)
        self.text_edit_handler.setFormatter(self.current_format)
        self._logger.addHandler(self.text_edit_handler)
