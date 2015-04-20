# Ice Demo With Open Computer Vision Components

Requirements: 
* ICE ZEROC (version > 3.4) installed
* OpenCV (Version > 2.3) installed with package configured
* Pthread Library
* CMAKE (Version 2.8 and above)

Overview:
This demo introduces the working of Ice cpp platform. The demo working includes a server and a client. The server initiates the process and makes a room for printing relevant data. The client on the other hand passes the printable data to the server and initiates the OpenCV component. The OpenCV component includes a live videostream from integrated camera of the system and applies two minor utilities of OpenCV, namely, "conversion to grayscale", and "conversion to edge based representation of image".

Compilation instructions:
It is built independently.
* cmake ..
* sudo make

Using instructions:
Open two terminals where the executable are created.
In the first type
* ./server
(This starts the server, without the server being started client cannot handle any operation)

In the other type
* ./client

Two OpenCV windows would open with a message in server terminal "Starting video!", followed by a series of messages "Showing normal video !". One of the windows shows video and the other a trackbar with values ranging from 0 to 3. Using mouse to slide the trackbar,
* Value 0 = Original Video
* Value 1 = Grayscale video
* Value 2 = Implementation of Canny Edge Detector

At every value, a different message will be displayed in the server terminal.
