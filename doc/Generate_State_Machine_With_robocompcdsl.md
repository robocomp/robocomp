#Generate Componet with State Machine

###In C++

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

![State Machine](/img/StateMachine.jpg)

This is the code of the state machine:

	Machine_testcpp{
	    states test2, test3, test4, test5;
	    initial_state test1;
	    end_state test6;
	    transition{
		test1 => test1, test2;
		test2 => test3, test5, test6;
		test3 => test3, test4;
		test4 => test5;
		test5 => test6;
	    };
	};

	:test1 parallel{
	    states test1sub1, test1sub2;
	    transition{
		test1sub1 => test1sub1;
		test1sub2 => test1sub2;
	    };
	};

	:test1sub2{
	    initial_state test1sub21;
	    end_state test1sub22;
	    transition{
		test1sub21 => test1sub21,test1sub22;
	    };
	};

	:test3 parallel{
	    states test3sub1, test3sub2, test3sub3;
	    transition{
		test3sub1 => test3sub1;
		test3sub2 => test3sub2;
	    };
	};

	:test5{
	    states test5sub2;
	    initial_state test5sub1;
	    transition{
		test5sub1 => test1sub2;
		test1sub2 => test5sub1;
	    };
	};


