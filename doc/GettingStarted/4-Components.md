#Components

In RoboComp whenever you want to build/simulate for a specific problem statement you build a component. Think component more like a folder which contains all the necessary code for a particular operation which can be saved for future use with other projects.

You can find the sample components which robocomp offers in 

	cd robocomp/components

You can build components using the eclipse based robocompDSLEditor [Deprecated] or the robocompdsl command line tool [Preferred]. In the course of the tutorial you will learn component generating using robocompdsl.

Currently robocomp's component based programming can be done in two languages c++ or python and the code will be generated accordingly when specified while creating the cdsl file.

When you generate a component you will find the following folders

1. bin - Contains the binary file of the component
2. CMakeFiles - will contain all the build logs/errors etc generated when building the component
3. etc - contains the config files for Ice
4. src - contains the files of interfaces which was imported in cdsl files. You program the component according to your requirements here.
