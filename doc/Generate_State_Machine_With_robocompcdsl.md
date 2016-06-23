#Generate Componet with State Machine

Usualy, when we create a component, this componet used a State Machine. So we must create a state machine from scratch. With robocomp it is now much easier. 

These are the steps to create a state machine with robocomp:

##### Definition State Machine

Run the following line:

	robocompcdsl mycomponent.cdsl

When you run this line two files are created component.cdsl and statemachine.smdsl. In stachine.smdsl we have to define our state machine, following the next struct:

	name_machine{
	    [states name_state *[, name_state];]
	    [initial_state name_state;]
	    [end_state name_state;]
	    [transition{
		name_state => name_state *[, name_state];
		*[name_state => name_state *[, name_state];]
	    };]
	};

	[:parent_state [parallel]{
	    states name_state *[, name_state];
	    [initial_state name_state;]
	    [end_state name_state;]
	    [transition{
		name_state => name_state *[, name_state];
		*[name_state => name_state *[, name_state];]
	    };]
	};]

For example:

![State Machine](img/StateMachine.jpg)

This is the code of the state machine:

	Machine_Statecpp{
	    states State2, State3, State4, State5;
	    initial_state State1;			// It may not be contained in the states list. initial_state is required
	    end_state State6;				// It may not be contained in the states list. initial_state and end_state can't be equal
	    transition{
		State1 => State1, State2;
		State2 => State3, State5, State6;
		State3 => State3, State4;
		State4 => State5;
		State5 => State6;
	    };
	};

	:State1 parallel{				// If it is parallel, it can't have initial_state and end state
	    states State11, State12;
	    transition{
		State11 => State11;
		State12 => State12;
	    };
	};

	:State12{
	    initial_state State121;			// If it isn't parallel, initial_state is required
	    end_state State122;
	    transition{
		State121 => State121,State122;
	    };
	};

	:State3 parallel{
	    states State31, State32, State33;
	    transition{
		State31 => State31;
		State32 => State32;
	    };
	};

	:State5{
	    states State52;
	    initial_state State51;
	    transition{
		State51 => State12;
		State12 => State51;
	    };
	};

Robocomp need the following line to implement the state machine. Online This will be contained in mycomponet.cdsl:

	statemachine statemachine.smdsl;

For example:

	import "/robocomp/interfaces/IDSLs/import1.idsl";
	import "/robocomp/interfaces/IDSLs/import2.idsl";

	Component mycomponet
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
		statemachine statemachine.smdsl;
	};

After executing the following line:

	robocompdsl mycomponet.cdsl .

The code is generated. Each state has a method, this method is modified which it is  executed when it enters this state.

To move from one state to another, we will emit the signal with the following structure "statesrctostatedst".

###In C++

For example:

	emit State1toState2;

###In Python

For example:

	self.State1toState2.emit

