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
	def __init__(self, interfaceName, validNames=None):
		self.interfaceName = interfaceName
		self.validNames = validNames
		self.message = f"EXCEPTION: InterfaceNotFound: {self.interfaceName}."
		if validNames:
			similar_list = difflib.get_close_matches(self.interfaceName, self.validNames, cutoff=0.4)
			if len(similar_list) > 0:
				self.message += f" Did you mean {str(similar_list)}?"
		super(InterfaceNotFound, self).__init__(self.message)


class ParseException(RobocompDslException):

	def __init__(self, msg, line, column):
		info = line + '\n' + " "*(column-1) + "^"
		self.message = msg + '\n' + info
		super(ParseException, self).__init__(self.message)
