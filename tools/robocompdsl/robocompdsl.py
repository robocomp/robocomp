#!/usr/bin/env python


# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#

import sys, os, subprocess

def generateHeaders(idslFile, outputPath, comp): #idslFile es el fichero idsl importando en el cdsl outputPath es outputPath+/src/
	imported = []
	idsl = IDSLParsing.fromFileIDSL(idslFile)
	if not os.path.exists(outputPath):
		creaDirectorio(outputPath)

	def generarH(idslFile, imported):
		idsl = IDSLParsing.fromFileIDSL(idslFile)
		for imp in idsl['module']['contents']:
			if imp['type'] in ['struct','sequence']:
				for f in [ "SERVANT.MSG"]:
					ofile = outputPath+"/"+imp['name'] + "." + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')'
					# Call cog
					run = "cog.py -z -d" + " -D structName=" + imp['name'] +" -D theIDSL="+idslFile+ " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
					commandCPP = "/opt/ros/jade/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py " +ofile+ " -Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg -I" + idsl['module']['name'] + ":" + outputPath
					commandPY  = "/opt/ros/jade/share/gencpp/cmake/../../../lib/genpy/genmsg_py.py " +ofile+ " -Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg -I" + idsl['module']['name'] + ":" + outputPath
					for impo in imported:
						if not impo == idsl['module']['name']:
							commandCPP = commandCPP + " -I" + impo + ":" + outputPath
							commandPY  = commandPY + " -I" + impo + ":" + outputPath
					if not os.path.exists(outputPath):
						creaDirectorio(outputPath)
					commandCPP = commandCPP + " -p "+ idsl['module']['name'] + " -o "+ outputPath+"/"+idsl['module']['name']+ " -e /opt/ros/jade/share/gencpp/cmake/.."
					commandPY = commandPY + " -p "+ idsl['module']['name'] + " -o "+ outputPath+"/"+idsl['module']['name']
					if comp['language'].lower() == 'cpp':
						os.system(commandCPP)
					else:
						os.system(commandPY)
		for imp in idsl['module']['contents']:
			if imp['type'] == 'interface':
				for ima in component['implements']+component['requires']:
					im = ima
					if type(im) != type(''):
						im = im[0]
					if not communicationIsIce(ima) and im == imp['name']:
						for method in imp['methods']:
							if 'params' in method:
								if len(method['params']) == 2:
									for f in [ "SERVANT.SRV"]:
										ofile = outputPath+"/"+method['name'] + "." + f.split('.')[-1].lower()
										print 'Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')'
										# Call cog
										run = "cog.py -z -d" + " -D methodName=" + method['name'] +" -D theIDSL="+idslFile+ " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
										run = run.split(' ')
										ret = Cog().main(run)
										if ret != 0:
											print 'ERROR'
											sys.exit(-1)
										replaceTagsInFile(ofile)
										commandCPP = "/opt/ros/jade/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py " +ofile+ " -Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg -Istd_srvs:/opt/ros/jade/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + ":" + outputPath
										commandPY  = "/opt/ros/jade/share/gencpp/cmake/../../../lib/genpy/gensrv_py.py " +ofile+ " -Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg -Istd_srvs:/opt/ros/jade/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + ":" + outputPath
										for impo in imported:
											if not impo == idsl['module']['name']:
												commandCPP = commandCPP + " -I" + impo + ":" + outputPath
												commandPY  = commandPY + " -I" + impo + ":" + outputPath
										if not os.path.exists(outputPath):
											creaDirectorio(outputPath)
										commandCPP = commandCPP + " -p "+ idsl['module']['name'] + " -o "+ outputPath+"/"+idsl['module']['name']+ " -e /opt/ros/jade/share/gencpp/cmake/.."
										commandPY = commandPY + " -p "+ idsl['module']['name'] + " -o "+ outputPath+"/"+idsl['module']['name']
										if comp['language'].lower() == 'cpp':
											os.system(commandCPP)
										else:
											os.system(commandPY)
								else:
									print "error: service with too many params. Form is: void method(type inVar, out type outVar);"
									#sys.exit(-1)
							else:
								print "error: service without params. Form is: void method(type inVar, out type outVar);"
								#sys.exit(-1)
		return idsl['module']['name']

	for importIDSL in idsl['imports']:
		imported.append(generarH("/opt/robocomp/interfaces/IDSLs/"+importIDSL, []))

	generarH(idslFile, imported)
	os.system("rm "+outputPath+"/*.msg")
	os.system("rm "+outputPath+"/*.srv")
#
# Misc functions
#
def replaceTagsInFile(path):
	i = open(path, 'r')
	text = i.read()
	reps = []
	reps.append(["\n<@@<" ,""])
	reps.append([">@@>\n" ,""])
	reps.append(["<TABHERE>", '\t'])
	reps.append(["<S1>", ' '])
	reps.append(["<S2>", '  '])
	reps.append(["<S4>", '    '])
	for r in reps:
		text = text.replace(r[0], r[1])
	i.close()
	w = open(path, 'w')
	w.write(text)
	w.close()

def generateDummyCDSL(path):
	if os.path.exists(path):
		print "File", path, "already exists.\nExiting..."
	else:
		print "Generating dummy CDSL file:", path
		string = """import "/robocomp/interfaces/IDSLs/import1.idsl";
import "/robocomp/interfaces/IDSLs/import2.idsl";

Component <CHANGETHECOMPONENTNAME>
{
	Communications
	{
		implements interfaceName;
		requires otherName;
		subscribesTo topicToSubscribeTo;
		publishes topicToPublish;
	};
	language Cpp;
	gui Qt(QWidget);
};\n\n"""
		name = path.split('/')[-1].split('.')[0]
		string = string.replace('<CHANGETHECOMPONENTNAME>', name)
		open(path, "w").write(string)

correct = True
if len(sys.argv) < 3:
	if len(sys.argv) == 2 and sys.argv[1].endswith(".cdsl"):
		generateDummyCDSL(sys.argv[1])
		sys.exit(0)
	else: correct = False
if not correct:
	print 'Usage:'
	print '\ta) to generate code from a CDSL file:     '+sys.argv[0].split('/')[-1]+'   INPUT_FILE.CDSL   OUTPUT_DIRECTORY'
	print '\tb) to generate a new CDSL file:           '+sys.argv[0].split('/')[-1]+'   NEW_COMPONENT_DESCRIPTOR.CDSL'
	sys.exit(-1)

inputFile  = sys.argv[1]
outputPath = sys.argv[2]

sys.path.append('/opt/robocomp/python')

from cogapp import Cog

if sys.argv[1].endswith(".cdsl"):
	from parseCDSL import *
	from parseIDSL import *
	component = CDSLParsing.fromFile(inputFile)

	#########################################
	# Directory structure and other checks  #
	#########################################
	# Function to create directories
	def creaDirectorio(directory):
		try:
			print 'Creating', directory,
			os.mkdir(directory)
			print ''
		except:
			if os.path.isdir(directory):
				print '(already existed)'
				pass
			else:
				print '\nCOULDN\'T CREATE', directory
				sys.exit(-1)

	imports = ''.join( [ imp.split('/')[-1]+'#' for imp in component['imports'] ] )

	if component['language'].lower() == 'cpp':
		#
		# Check output directory
		#
		if not os.path.exists(outputPath):
			creaDirectorio(outputPath)
		# Create directories within the output directory
		try:
			creaDirectorio(outputPath+"/bin")
			creaDirectorio(outputPath+"/etc")
			creaDirectorio(outputPath+"/src")
		except:
			print 'There was a problem creating a directory'
			sys.exit(1)
			pass
		#
		# Generate regular files
		#
		files = [ 'CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.cpp', 'src/CMakeLists.txt', 'src/CMakeListsSpecific.txt', 'src/commonbehaviorI.h', 'src/commonbehaviorI.cpp', 'src/genericmonitor.h', 'src/genericmonitor.cpp', 'src/config.h', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'src/genericworker.h', 'src/genericworker.cpp', 'src/specificworker.h', 'src/specificworker.cpp', 'src/mainUI.ui' ]

		specificFiles = [ 'src/specificworker.h', 'src/specificworker.cpp', 'src/CMakeListsSpecific.txt', 'src/mainUI.ui', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'README.md', 'etc/config' ]
		for f in files:
			ofile = outputPath + '/' + f
			if f in specificFiles and os.path.exists(ofile):
				print 'Not overwriting specific file "'+ ofile +'", saving it to '+ofile+'.new'
				ofile += '.new'
			ifile = "/opt/robocomp/share/robocompdsl/templateCPP/" + f
			if f != 'src/mainUI.ui' or component['gui'] != 'none':
				print 'Generating', ofile, 'from', ifile
				run = "cog.py -z -d -D theCDSL="+inputFile + " -D theIDSLs="+imports + " -o " + ofile + " " + ifile
				run = run.split(' ')
				ret = Cog().main(run)
				if ret != 0:
					print 'ERROR'
					sys.exit(-1)
				replaceTagsInFile(ofile)
		#
		# Generate interface-dependent files
		#
		for ima in component['implements']:
			im = ima
			if type(im) != type(''):
				im = im[0]
			if communicationIsIce(ima):
				for f in [ "SERVANT.H", "SERVANT.CPP"]:
					ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', im + ')'
					# Call cog
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports + " -D theInterface="+im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
					
		for imp in component['subscribesTo']:
			im = imp
			if type(im) != type(''):
				im = im[0]
			if communicationIsIce(imp):
				for f in [ "SERVANT.H", "SERVANT.CPP"]:
					ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', im + ')'
					# Call cog
					theInterfaceStr = im
					if type(theInterfaceStr) == type([]):
						theInterfaceStr = str(';'.join(im))
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports + " -D theInterface="+theInterfaceStr + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
					#print run
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
	elif component['language'].lower() == 'python':
		#
		# Check output directory
		#
		if not os.path.exists(outputPath):
			creaDirectorio(outputPath)
		# Create directories within the output directory
		try:
			creaDirectorio(outputPath+"/etc")
			creaDirectorio(outputPath+"/src")
		except:
			print 'There was a problem creating a directory'
			sys.exit(1)
			pass
		#
		# Generate regular files
		#
		files = [ 'CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.py', 'src/genericworker.py', 'src/specificworker.py', 'src/mainUI.ui' ]
		specificFiles = [ 'src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config' ]
		for f in files:
			if f == 'src/main.py':
				ofile = outputPath + '/src/' + component['name'] + '.py'
			else:
				ofile = outputPath + '/' + f
			if f in specificFiles and os.path.exists(ofile):
				print 'Not overwriting specific file "'+ ofile +'", saving it to '+ofile+'.new'
				ofile += '.new'
			ifile = "/opt/robocomp/share/robocompdsl/templatePython/" + f
			print 'Generating', ofile, 'from', ifile
			run = "cog.py -z -d -D theCDSL="+inputFile + " -D theIDSLs="+imports + " -o " + ofile + " " + ifile
			run = run.split(' ')
			ret = Cog().main(run)
			if ret != 0:
				print 'ERROR'
				sys.exit(-1)
			replaceTagsInFile(ofile)
			if f == 'src/main.py': os.chmod(ofile, os.stat(ofile).st_mode | 0111 )
		#
		# Generate interface-dependent files
		#
		for imp in component['implements']+component['subscribesTo']:
			if type(imp) != type(''):
				im = imp[0]
			if communicationIsIce(imp):
				for f in [ "SERVANT.PY"]:
					ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', im + ')'
					# Call cog
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports + " -D theInterface="+im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templatePython/" + f
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
	else:
		print 'Unsupported language', component['language']
	if component['usingROS']:
		for imp in component['imports']:
			generateHeaders("/opt/"+imp, outputPath+"/src", component)
elif sys.argv[1].endswith(".idsl"):
	from parseIDSL import *
	imported = []
	idsl = IDSLParsing.fromFileIDSL(inputFile)
	if not os.path.exists(outputPath):
		creaDirectorio(outputPath)

	def generarMSG(inputFile, imported):
		idsl = IDSLParsing.fromFileIDSL(inputFile)
		for imp in idsl['module']['contents']:
			if imp['type'] in ['struct','sequence']:
				for f in [ "SERVANT.MSG"]:
					ofile =imp['name'] + "." + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', inputFile.split('.')[0].lower() + ')'
					# Call cog
					run = "cog.py -z -d" + " -D structName=" + imp['name'] +" -D theIDSL="+inputFile+ " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
		for imp in idsl['module']['contents']:
			if imp['type'] == 'interface':
				for method in imp['methods']:
					if 'params' in method:
						if len(method['params']) == 2:
							for f in [ "SERVANT.SRV"]:
								ofile =method['name'] + "." + f.split('.')[-1].lower()
								print 'Generating', ofile, ' (servant for', inputFile.split('.')[0].lower() + ')'
								# Call cog
								run = "cog.py -z -d" + " -D methodName=" + method['name'] +" -D theIDSL="+inputFile+ " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
								run = run.split(' ')
								ret = Cog().main(run)
								if ret != 0:
									print 'ERROR'
									sys.exit(-1)
								replaceTagsInFile(ofile)
		return idsl['module']['name']

	for importIDSL in idsl['imports']:
		imported.append(generarMSG(importIDSL, []))
	generarMSG(inputFile, imported)