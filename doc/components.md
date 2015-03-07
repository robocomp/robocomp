#Software components: a brief introduction

What is Component Oriented Programming?

Two major problems encountered when creating software are scalability and reusability. These problems are especially acute when it comes to software being used in robotics and is due to software use is exceptional in this field. Despite the importance of reusability, it is generally lose sight of this and just creating monolithic and unwieldy software.

In the field of robotics is very common for researchers to implement all the algorithms with a rigid design a task-oriented and / or a specific robot. If so, when it completes the implementation stage of the software developed is ultimately impossible to use. Often so tied to a specific platform or task that is more practical to start from zero (due to dependencies and side effects resulting from its stiffness).

The component-oriented programming emerged as a solution to such problems. It is an approach that does not necessarily have to do with concurrency and distributed computing, but with how software is organized. The object-oriented programming represented a major advance on structured programming, however, when the number of classes and their interdependencies increases, too difficult to understand the overall system. It is therefore beneficial to have a greater degree of encapsulation, which combines several related classes under a single interface, and for understanding the system with less detailed. The component-oriented programming, which was proposed to solve such problems, many see it as the next step after object-oriented programming.

From a design point of view can be seen as a great class to offer public methods. The only difference from this point of view is that the complexity introduced by the classes of dependent component (or class) that are not the domain of the problem disappears because the interface hides the component. A component can be arbitrarily complex, but a step back, all you see is the interface offered. This is what defines it as a component.

Therefore, if each component performs a series of tasks or answer a series of orders, who need to handle that communication. 

To use component-oriented programming has split the software design into pieces that provide an interface. In return we get more reusability, using the same components in different contexts, thus significantly reducing time, cost and effort of developing new applications, while increasing flexibility, reusability and reliability of them. It will be easier to isolate and find bugs, getting eliminate the need to consider hundreds of classes to understand the developed software.
