RoboComp Component & Ros Node chatter
============

Here we will create a RoboComp Component which can communicate with a Ros Node using Ros Middleware.
The Ros Node which we will use can be found [here](https://github.com/robocomp/robocomp/tree/highlyunstable/doc/src/beginner_tutorials).

Steps:

1. Analyze the Ros Node.
2. Generate an IDSL.
3. Generate our RoboComp Component.
4. Run it.

### Analyze the Ros Node

We need to know what messages or services they are using. To find out, we can take a look at the package documentation and the msg and srv directory.
Here is an *image.msg* example which is used in our Ros Node:

    string name
    uint32 size
    int32[] image

In the Talker.cpp code, we can see:

    // %Tag(PUBLISHER)%
      ros::Publisher chatter_pub = n.advertise<beginner_tutorials::image>("chatter", 1000);
    // %EndTag(PUBLISHER)%

Talker (Ros Node) publishes this MSG through the topic called **chatter**.
So We need an IDSL with a structure containing the types of image.msg and an interface with a topic called chatter.

### Generate an IDSL

Let's generate our IDSL:

    module RoboCompListener{
    	sequence<int> imageVector;
    	struct Image {
    	 	string name;
    	 	uint size;
    	 	imageVector image;
    	 };
    	
    	interface chat{
    		void chatter(Image img);
    	};
    };

**The names of the variables must be identical to those used in the message!**

We have created a structure Image and a topic called chatter.

### Generate our RoboComp Component

Create your .cdsl like this:

    import "/robocomp/interfaces/IDSLs/Listener.idsl";
    Component listener
    {
    	Communications
    	{
    		subscribesTo chat(ros);
    	};
    	language Cpp;
    	gui Qt(QWidget);
    };

And use RoboCompDSL to generate your component.

### Run it

To see if everything worked, try to insert this code in your subscribe method:

    printf("Name: %s, size: %d\n", img.name.c_str(), img.size);
    for(int i=0;i<img.image.size();i++)
        printf("value of img.image[%d] = %d\n", i, img.image[i]);

Launch your component and enjoy!
> We assume that Talker and **roscore** are running.

> If you get an error like this:

> [ERROR] [1471018647.418758838]: Client [/listener] wants topic /chatter to have datatype/md5sum [RoboCompListener/Image/**5d96ac93c863cbaa85815010f7256b9a**], but our version has [beginner_tutorials/image/**163b37c985fb24d5121dd98a6a240a84**]. Dropping connection.

> Then you have not generated an identical message structure.
