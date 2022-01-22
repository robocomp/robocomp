# RoboComp Component & Ros Node chatter

We will create a RoboComp Component that can communicate with a Ros Node using Ros Middleware.

The Ros Node which we will use can be found [here](https://github.com/robocomp/robocomp/tree/development/doc/src/beginner_tutorials). 

**Steps for Creating a Component:**

1. Analyze the Ros Node.
2. Generate an IDSL.
3. Generate our RoboComp Component.
4. Run it.

## 1. Analyze the Ros Node

We need to know what messages or services the Ros Node is using. To find out, we are taking a look at the package documentation and the msg and srv directory.

**For more info** about what are msg and srv directories, visit this [tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) from ROS.

Here is an *image.msg* example, taken from our Ros Node:

```
string name
uint32 size
int32[] image
```

In Talker.cpp:

```cpp
// %Tag(PUBLISHER)%
ros::Publisher chatter_pub = n.advertise<beginner_tutorials::image>("chatter", 1000);
// %EndTag(PUBLISHER)%
```

Talker (the Ros Node) publishes this msg through the topic called `chatter`.

**For more info** about topics visit this [tutorial](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics) from ROS.

Now we know we need an IDSL with a structure containing the types of image.msg, and an interface with a topic called chatter.

## 2. Generate an IDSL

Let's generate our IDSL:

```cpp
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
```

:heavy_exclamation_mark: **The names of the variables must be identical to those used in the message!**

We have created a structure image and a topic called chatter.

**For more info** about IDSL visit this [tutorial](https://github.com/robocomp/robocomp/blob/development/doc/IDSL.md) created by Robocomp.

## 3. Generate the RoboComp Component

Create your .cdsl like this:

```python
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
```    

And use RoboCompDSL to generate your component.

**For more info** about creating components, look at this [tutorial](https://github.com/robocomp/robocomp/blob/development/doc/robocompdsl.md) created by Robocomp.

## 4. Run it

To see if everything worked, try to insert this code in your subscribe method:

```cpp
printf("Name: %s, size: %d\n", img.name.c_str(), img.size);
for(int i=0;i<img.image.size();i++)
    printf("value of img.image[%d] = %d\n", i, img.image[i]);
```

Launch your component and enjoy!

### Error warning:

We assume that **Talker** and **roscore** are running.

If you get an error like this:

`[ERROR] [1471018647.418758838]: Client [/listener] wants topic /chatter to have datatype/md5sum [RoboCompListener/Image/**5d96ac93c863cbaa85815010f7256b9a**], but our version has [beginner_tutorials/image/**163b37c985fb24d5121dd98a6a240a84**]. Dropping connection.`

Then you have not generated an identical message structure.
