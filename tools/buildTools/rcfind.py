#!/usr/bin/env python3
import os
import sys
from workspace import Workspace
class rccd:
	def __init__(self):
        	self.ws = Workspace()

	def save_filtered_component(self, searched_component):
		options = self.ws.find_component(searched_component)
		if (options):
			print(options)
		else:
			pass

	def list_comp(self):
		components=self.ws.list_components_names()
		for component in components:
			print(component)

if __name__ == '__main__':
	rccd = rccd()
	args = sys.argv[1]
	if args == "list":
		rccd.list_comp()
	else:
		try:
			rccd.save_filtered_component(args)
		except KeyboardInterrupt:
			save_output("")





