# CDSL
CDSL is Component Definition Language used to create and define component in Robocomp. With CDSL you specify how the component will communicate with the world. 

A basic cdsl file can be generated as given below. Go to a folder and type the following commands to make a directory and generate a component :
https://robocomp.github.io/web/gsoc/2017/love/post5

So after this we can compile the component using the following command :
robocompdsl myComponent.cdsl .

So this generates two new folders :

1. etc
2. src

The src folder contains a folder and two files:
1. UI (folder)
2. component.js
3. makefile

So now our basic component is ready. To make the component we sue the makefile. Just type the following commands:
cd src
make
