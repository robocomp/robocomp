# RoboComp Integration with CoppeliaSim Using PyRep

In many situations, a more robust simulator than _RCIS_ is required to leverege the power of _RoboComp_ in more physically complex scenes.
Consequently, _RoboComp_ needs to be integrated to more complex simulators and physics engines. The current choice for _RoboComp_ is [CoppeliaSim](https://www.coppeliarobotics.com/), previously known as __V-REP__. 

The standard remote _python_ API of __CoppeliaSim__ is very slow, so we opt for [PyRep](https://github.com/stepjam/PyRep) which includes a fast and flexible API to connect with __CoppeliaSim__.

In this tutorial, we walk you through the setup procedure of __CoppeliaSim__ and __PyRep__, along with some examples to try with _RoboComp_.

## CoppeliaSim Installation

-   First, download __CoppeliaSim__ EDU version (compatible with your system) from [download page](https://www.coppeliarobotics.com/downloads.html).

-   Extract it into your prefered directory.

-   Add the following to your `~/.bashrc` file (changing the path) :
```bash
export COPPELIASIM_ROOT=PATH/TO/COPPELIASIM/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```

-   Source your `~/.bashrc` file :
```bash
source ~/.bashrc
```

## PyRep Installation

-   First, clone _master_ branch of __PyRep__ repo :
```bash
git clone https://github.com/stepjam/PyRep.git
```

-   Install __PyRep__ dependencies and library :
```bash
cd PyRep
pip3 install -r requirements.txt
pip3 install .
```

## Testing your Installation

-   After completing the installation, you can make sure it's working correctly, by trying some of the examples in `examples` folder in __PyRep__ repo.

-   Also, you can refer to [Usage](https://github.com/stepjam/PyRep#usage) section in main `README.md`, for easy and fast code snippets to get started with.

## Testing RoboComp Bridges

-   Now that everything is ready, you can start trying the currently-available _RoboComp_ bridges.

-   The main _RoboComp_ bridges that currently exists :
    -   [omniPyrep](https://github.com/robocomp/dsr-graph/tree/development/robots_pyrep/omniPyrep).
    -   [pioneerPyrep](https://github.com/robocomp/dsr-graph/tree/development/robots_pyrep/pioneerPyrep).
    -   [viriatoPyrep](https://github.com/robocomp/dsr-graph/tree/development/robots_pyrep/viriatoPyrep).
    -   [viriatoGraspingPyrep](https://github.com/robocomp/grasping/tree/master/components/viriatoGraspingPyrep) : testbench for _6D object pose estimation_ and _grasping_.

-   Make sure to follow the instructions for each one.

## Current Known Issues

-   __This application failed to start because no Qt platform plugin could be initialized :__
    -   This problem can appear when trying to start the component, due to compatibility issues with _Qt_ version in _OpenCV_ and _VREP_.
    -   This problem is solved by installing `opencv-python-headless` :
        ```bash
        pip install opencv-python-headless
        ```

-   __"NotImplementedError: Must be overridden" exception in pyrep/objects/object.py, when running viriatoPyrep :__
    -   Comment out the following lines in `/home/xxxyour-userxxx/.local/lib/python3.6/site-packages/pyrep/objects/object.py` :
        ```python
        assert_type = self._get_requested_type()
        actual = ObjectType(sim.simGetObjectType(self._handle))
        if actual != assert_type:
            raise WrongObjectTypeError(
                'You requested object of type %s, but the actual type was '
                '%s' % (assert_type.name, actual.name))
        ```
