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
		os.system("rm -f "+outputPath + "/" + idsl['module']['name'] + "ROS/msg/__init__.py")
		os.system("rm -f "+outputPath + "/" + idsl['module']['name'] + "ROS/srv/__init__.py")
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
					commandCPP = "/opt/ros/kinetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py " +ofile+ " -Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg -I" + idsl['module']['name'] + "ROS:" + outputPath
					commandPY  = "/opt/ros/kinetic/share/gencpp/cmake/../../../lib/genpy/genmsg_py.py " +ofile+ " -Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg -I" + idsl['module']['name'] + "ROS:" + outputPath
					for impo in imported:
						if not impo == idsl['module']['name']+"ROS":
							commandCPP = commandCPP + " -I" + impo + ":" + outputPath
							commandPY  = commandPY + " -I" + impo + ":" + outputPath
					if not os.path.exists(outputPath):
						creaDirectorio(outputPath)
					commandCPP = commandCPP + " -p "+ idsl['module']['name'] + "ROS -o " + outputPath + "/" + idsl['module']['name'] + "ROS -e /opt/ros/kinetic/share/gencpp/cmake/.."
					commandPY = commandPY + " -p "+ idsl['module']['name'] + "ROS -o " + outputPath + "/" + idsl['module']['name'] +"ROS/msg"
					if comp['language'].lower() == 'cpp':
						os.system(commandCPP)
					else:
						os.system(commandPY)
					try:
						fileInit = open(outputPath + "/" + idsl['module']['name'] + "ROS/msg/__init__.py", 'a')
						fileInit.write("from ._"+imp['name']+" import *\n")
						fileInit.close()
					except:
						pass
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
										commandCPP = "/opt/ros/kinetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py " +ofile+ " -Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg -Istd_srvs:/opt/ros/kinetic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + outputPath
										commandPY  = "/opt/ros/kinetic/share/gencpp/cmake/../../../lib/genpy/gensrv_py.py " +ofile+ " -Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg -Istd_srvs:/opt/ros/kinetic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + outputPath
										for impo in imported:
											if not impo == idsl['module']['name']+"ROS":
												commandCPP = commandCPP + " -I" + impo + ":" + outputPath
												commandPY  = commandPY + " -I" + impo + ":" + outputPath
										if not os.path.exists(outputPath):
											creaDirectorio(outputPath)
										commandCPP = commandCPP + " -p "+ idsl['module']['name'] + "ROS -o "+ outputPath+"/"+idsl['module']['name'] + "ROS -e /opt/ros/kinetic/share/gencpp/cmake/.."
										commandPY = commandPY + " -p "+ idsl['module']['name'] + "ROS -o "+ outputPath+"/"+idsl['module']['name'] +"ROS/srv"
										if comp['language'].lower() == 'cpp':
											os.system(commandCPP)
										else:
											os.system(commandPY)
										try:
											fileInit = open(outputPath + "/" + idsl['module']['name'] + "ROS/srv/__init__.py", 'a')
											fileInit.write("from ._"+method['name']+" import *\n")
											fileInit.close()
										except:
											pass
								else:
									print "error: ROS service with incorrect number of parameters. ROS only supports remote procedure calls of the form: void method(type inVar, out type outVar);"
									for param in enumerate(method['params']):
										print param[0], '-->', param[1]
									sys.exit(-1)
							else:
								print "error: service without params. Form is: void method(type inVar, out type outVar);"
								sys.exit(-1)
		os.system("touch "+outputPath + "/" + idsl['module']['name'] + "ROS/__init__.py")
		return idsl['module']['name']+"ROS"
	try:
		for importIDSL in idsl['imports']:
			imported.append(generarH("/opt/robocomp/interfaces/IDSLs/"+importIDSL, []))
	except:
		pass

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

if sys.argv[1].endswith(".cdsl"):
	from parseCDSL import *
	from parseIDSL import *
	import sys

	# Get -I parameters, replacing ~ by $HOME
	includeDirectories = [ os.path.expanduser(x[2:]) for x in sys.argv if x.startswith('-I') ]

	component = CDSLParsing.fromFile(inputFile, includeDirectories=includeDirectories)
	imports = ''.join( [ imp+'#' for imp in component['imports'] ] )

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
				run = "cog.py -z -d -D theCDSL="+inputFile + " -D theIDSLs="+imports + ' -D theIDSLPaths='+ '#'.join(includeDirectories) + " -o " + ofile + " " + ifile
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
					print 'Generating ', ofile, ' (servant for', im + ')'
					# Call cog
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports + ' -D theIDSLPaths='+ '#'.join(includeDirectories) + " -D theInterface="+im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
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
					print 'Generating ', ofile, ' (servant for', im + ')'
					# Call cog
					theInterfaceStr = im
					if type(theInterfaceStr) == type([]):
						theInterfaceStr = str(';'.join(im))
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports  + ' -D theIDSLPaths='+ '#'.join(includeDirectories) + " -D theInterface="+theInterfaceStr + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
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
			run = "cog.py -z -d -D theCDSL="+inputFile + " -D theIDSLs="+imports + ' -D theIDSLPaths='+ '#'.join(includeDirectories) + " -o " + ofile + " " + ifile
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
			else:
				im = imp
			if communicationIsIce(imp):
				for f in [ "SERVANT.PY"]:
					ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
					print 'Generating', ofile, ' (servant for', im + ')'
					# Call cog
					run = "cog.py -z -d -D theCDSL="+inputFile  + " -D theIDSLs="+imports  + ' -D theIDSLPaths='+ '#'.join(includeDirectories) + " -D theInterface="+im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templatePython/" + f
					run = run.split(' ')
					ret = Cog().main(run)
					if ret != 0:
						print 'ERROR'
						sys.exit(-1)
					replaceTagsInFile(ofile)
	else:
		print 'Unsupported language', component['language']

	if component['usingROS'] == True:
		for imp in component['imports']:
			generateHeaders("/opt/"+imp, outputPath+"/src", component)

elif sys.argv[1].endswith(".idsl"):
	from parseIDSL import *
	inputFile  = sys.argv[1]
	outputFile = sys.argv[2]
	#idsl = IDSLParsing.fromFileIDSL(inputFile)
	print 'Generating ICE file ', outputFile
	# Call cog
	run = "cog.py -z -d" + " -D theIDSL="+inputFile + ' -D theIDSLPaths='+ '#'.join(includeDirectories) +" -o " + outputFile + " /opt/robocomp/share/robocompdsl/TEMPLATE.ICE"
	run = run.split(' ')
	ret = Cog().main(run)
	if ret != 0:
		print 'ERROR'
		sys.exit(-1)
	replaceTagsInFile(outputFile)
