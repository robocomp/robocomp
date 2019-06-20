#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import os
import subprocess
import sys
from shutil import rmtree

from termcolor import cprint, colored


#Class to print colored error message on argparse
class MyParser(argparse.ArgumentParser):
    def error(self, message):
        cprint('error: %s' % message, 'red')
        self.print_help()
        sys.exit(2)


class ComponentGenerationChecker:

	def __init__(self):
		self.results = {}
		self.valid = 0
		self.generated = 0
		self.compiled = 0
		self.comp_failed = 0
		self.gen_failed = 0
		self.dry_run = False


	def generate_code(self, cdsl_file, log_file, dry_run=True):
		if dry_run:
			print('robocompdsl %file > /dev/null 2>&1' % cdsl_file)
			print('robocompdsl %cdsl_file . > %log_file 2>&1' % (cdsl_file, log_file))
			return True
		else:
			# completedProc = subprocess.Popen('robocompdsl %s > /dev/null 2>&1'%cdsl_file)
			# completedProc = subprocess.Popen('robocompdsl %s . > %s 2>&1' % (cdsl_file, log_file))
			command_output = subprocess.Popen(
				"python " + os.path.expanduser("~/robocomp/tools/robocompdsl/robocompdsl.py %s" % cdsl_file),
				stdout=subprocess.PIPE,
				stderr=subprocess.STDOUT, shell=True)
			stdout, stderr = command_output.communicate()
			# print(stdout)
			# print(stderr)

			with open("generation_output.log", "wb") as log:
				command_output = subprocess.Popen(
					"python " + os.path.expanduser("~/robocomp/tools/robocompdsl/robocompdsl.py") + " %s ." % cdsl_file,
					stdout=subprocess.PIPE,
					stderr=log,
					shell=True)
				stdout, stderr = command_output.communicate()
				# print(stdout)
				# print(stderr)
				return command_output.returncode
			return -1


	def cmake_component(self):
		with open("cmake_output.log", "wb") as log:
			command_output = subprocess.Popen("cmake .",
											  stdout=subprocess.PIPE,
											  stderr=log,
											  shell=True)
			stdout, stderr = command_output.communicate()
			# print(stdout)
			# print(stderr)
			return command_output.returncode


	def make_component(self, dry_run=True):
		command = "make"
		if dry_run:
			command += " --dry-run"
		with open("make_output.log", "wb") as log:
			command_output = subprocess.Popen(command,
											  stdout=subprocess.PIPE,
											  stderr=log,
											  shell=True)
			stdout, stderr = command_output.communicate()
			if dry_run:
				print(stdout)
			# print(stderr)
			return command_output.returncode


	def remove_genetared_files(self,dir, dry_run=True):
		component_files = os.listdir(dir)
		# Remove not useful files
		for file in component_files:
			if not any(file.endswith(extension) for extension in (".smdsl", ".cdsl", ".log")):
				if dry_run:
					print("rm -r %s" % file)
				else:
					if os.path.isfile(file):
						os.remove(file)
					else:
						rmtree(file)



	def check_components_generation(self, robocompdsl_dir, dry_run, dirty, generate_only=False, filter="", clean_only=False):
		self.dry_run = dry_run
		previous_dir = os.getcwd()
		os.chdir(os.path.expanduser(robocompdsl_dir))
		list_dir = os.listdir(".")
		for item in list_dir:
			if os.path.isdir(item) and filter in item:
				dir = item
				cprint("Entering dir %s" % dir, 'magenta')
				os.chdir(dir)
				self.remove_genetared_files(".", self.dry_run)
				if clean_only:
					cprint("\tCleaned", 'green')
					os.chdir("..")
					continue
				cdsl_file = None
				for file in os.listdir("."):
					if file.endswith(".cdsl"):
						cdsl_file = file
				if cdsl_file:
					self.valid += 1
					self.results[dir] = {}
					if self.generate_code(cdsl_file, "generation_output.log", False) == 0:
						self.results[dir]['generation'] = True
						self.generated += 1
						if generate_only:
							self.results[dir]['compilation'] = False
							print("%s not compiled (-g option)" % dir)
						else:
							print("Executing cmake for %s ... WAIT!"%(dir))
							self.cmake_component()
							print("Executing make for %s ... WAIT!"%(dir))
							make_result = self.make_component(self.dry_run)

							if make_result == 0 or (make_result==2 and self.dry_run):
								self.results[dir]['compilation'] = True
								self.compiled += 1
								cprint("%s compilation OK" % dir, 'green')
							else:
								self.results[dir]['compilation'] = False
								self.comp_failed += 1
								cprint("%s compilation FAILED" % dir, 'red')
					else:
						cprint("$dir $cdsl_file generation FAILED", 'red')
						self.gen_failed += 1
						self.results[dir]['generation']=False
				if not dirty:
					self.remove_genetared_files(".", self.dry_run)
				print("")
				os.chdir("..")
		os.chdir(previous_dir)

		# Command final output
		if not clean_only:

			print("%d components found with cdsl" % self.valid)
			print("%d components generated OK (%d failed)" % (self.generated, self.gen_failed))
			print("%d components compiled OK (%d failed)" % (self.compiled, self.comp_failed))

			for dir, result in self.results.items():
				cname = colored(dir, 'magenta')

				# Printing results for generation
				if result['generation']:
					gen_result = colored("TRUE", 'green')
				else:
					gen_result = colored("FALSE", 'red')

				# Printing results for compilation
				if result['compilation']:
					comp_result = colored("TRUE", 'green')
				else:
					comp_result = colored("FALSE", 'red')

				if generate_only:
					print("\t%s have been generated? %s" % (cname, gen_result))
				else:
					print("\t%s have been generated? %s compile? %s" % (cname, gen_result, comp_result))








if __name__ == '__main__':
	parser = MyParser(description=colored('This application generate components from cdsl files to check component generation/compilation\n', 'magenta'),
	                  formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument("-g", "--generate-only", action="store_true",
	                    help="Only the generation with robocompdsl is checked. No cmake or make is tested")
	parser.add_argument("-d", "--dirty",
	                    help="No cleaning is done after execution. All source files and temp will be left on their dirs.",
	                    action="store_true")
	parser.add_argument("-c", "--clean",
	                    help="Just clean all source files and temp will be left on their dirs. .cdsl, .smdsl and .logs are kept on their dirs.",
	                    action="store_true")
	parser.add_argument("-n", "--dry-run", help="Executing dry run. No remove and make will be executed in dry run mode.", action="store_true")
	parser.add_argument("-i", "--installation", type=str, help="Installation directory where robocompdsl.py can be found.", default="~/robocomp/tools/robocompdsl/component_generation_test")

	parser.add_argument("-f", "--filter", type=str, help="Execute the check only for directories containing this string.", default="")
	args = parser.parse_args()
	#
	# if args.output_path is None:
	# 	if args.input_file.endswith(".cdsl"):
	# 		generateDummyCDSL(args.input_file)
	# 		generateDummySMDSL("statemachine.smdsl")
	# 		sys.exit(0)
	# 	else:
	# 		print args.output_path, args.input_file
	# 		print parser.error("No output path with non .cdsl file")
	# 		sys.exit(-1)
	#
	# inputFile = args.input_file
	# outputPath = args.output_path
	#
	# sys.path.append('/opt/robocomp/python')
	#
	# new_existing_files = {}
	#
	# if inputFile.endswith(".cdsl"):

	checker = ComponentGenerationChecker()
	checker.check_components_generation(args.installation, args.dry_run, args.dirty, args.generate_only, args.filter, args.clean)