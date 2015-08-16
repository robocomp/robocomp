#Component Creation using robocompdsl

#Command Line tool
robocompdsl is a command line tool and it is more easy to work with and build components than by using robocompDSLEditor. To start with first let us create a new directory for our component say firstcomp

	mkdir firstcomp
	cd firstcomp

now we must create a new cdsl file to do that execute

	robocompdsl first.cdsl

Now inside the firstcomp folder a dummy cdsl file will be generated which will have the following code

	import "/robocomp/interfaces/IDSLs/import1.idsl";
	import "/robocomp/interfaces/IDSLs/import2.idsl";

	Component test
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
	};

This is the skeleton code for any cdsl file. As an example let us use the idsl differential robot and create a component. Replace the above code with the following example and save the cdsl file

	import "/robocomp/interfaces/IDSLs/DifferentialRobot.idsl";
	Component first
	{
		Communications
		{
			requires DifferentialRobot;

		};
		language Cpp;
	};

Now that we have coded the cdsl file we now generate the code for the same. To do so, execute

	robocompdsl first.cdsl build

Here `build` is the directory where the code will be generated. After generating the code to build the component execute the following

	cd build
	cmake .
	make

You have now succesfully created and built a component using robocompdsl.
