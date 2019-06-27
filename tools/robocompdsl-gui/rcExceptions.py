import difflib, sys, inspect


class RobocompDslException(BaseException):
	"""A base class for robocompdsl's exceptions."""
	def __init__(self, msg):
		self.message = msg
		super(RobocompDslException, self).__init__(self.message)


class InterfaceNotFound(RobocompDslException):
	"""Accessing an Interface which is not found.
		:param interfaceName: missing Interface
		:param validNames: valid interfaces
	"""
	def __init__(self, interfaceName, validNames=None):
		self.interfaceName = interfaceName
		self.validNames = validNames
		self.message = interfaceName
		if validNames:
			similar_list = difflib.get_close_matches(self.interfaceName, self.validNames, cutoff=0.4)
			if len(similar_list) > 0:
				self.message = self.message + ". Did you mean " + str(similar_list[0])
		super(InterfaceNotFound, self).__init__(self.message)


class ParseException(RobocompDslException):

	def __init__(self, msg, line, column):
		info = line + '\n' + " "*(column-1) + "^"
		self.line = line
		self.message = msg + '\n' + info
		super(ParseException, self).__init__(self.message)