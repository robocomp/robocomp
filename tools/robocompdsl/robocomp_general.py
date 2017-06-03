#!/usr/bin/env python
import json
import os
# TODO: CONVERT THIS CLASS TO A ROBOCOMP CONFIG CLASS 

class config_robocomp:
	def __init__(self, path_config):
		if path_config is not None:
			self.read_config(path_config)
			self.config_robocompdsl = True
		else:
			self.config_robocompdsl = False


	def read_config(self, path_config="/opt/robocomp/share/robocompdsl/robocompdsl_config.json"):
		file_name, file_extension = os.path.splitext(path_config)
		if file_extension == ".json":
			try:
				with open(path_config) as data_config:
					self.config = json.load(data_config)
			except IOError:
				print "ERROR: File",path_config,"not found"
				sys.exit(-1)
		else:
			# add your config file format
			print "ERROR: format",file_extension,"not supported"
			pass


	def check_config():
		return self.config_robocompdsl is True