import difflib
import sys
import inspect


class RobocompDslException(Exception):
	"""A base class for robocompdsl's exceptions."""


class InterfaceNotFound(RobocompDslException):
	"""Accessing an Interface which is not found.
		:param interfaceName: missing Interface
		:param validNames: valid interfaces
	"""
	def __init__(self, interface_name, valid_names=None):
		self.interface_name = interface_name
		self.valid_names = valid_names
		self.message = interface_name
		if valid_names:
			similar_list = difflib.get_close_matches(self.interface_name, self.valid_names, cutoff=0.4)
			if len(similar_list) > 0:
				self.message = self.message + ". Did you mean " + str(similar_list[0])
		super(RobocompDslException, self).__init__(self.message)


class ParseException(RobocompDslException):

	def __init__(self, msg, line, column):
		info = line + '\n' + " "*(column-1) + "^"
		self.message = msg + '\n' + info
		super(ParseException, self).__init__(self.message)
