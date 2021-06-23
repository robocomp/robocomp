from pydsr import *
import numpy as np
import time
import os


def main():

    #x = threading.Thread(target=profile)
    #x.start()

    a = Node(12, "world", "elmundo")
    a.attrs["cam_image"] = Attribute([100], 1)

    print("Update value creating attribute using numpy array")
    for i in range(0,3):
        x = os.urandom(3000000) #bytes(2100000)
        t = time.time()
        a.attrs["cam_image"].value = np.frombuffer(x, dtype=np.uint8)
        t2 = time.time() - t
        print(t2)

    print("Update value creating attribute numpy array")
    for i in range(0,3):
        x = list(os.urandom(3000000))
        t = time.time()
        a.attrs["cam_image"].value = x
        t2 = time.time() - t
        print(t2)

    print("Update value numpy array")
    for i in range(0,3):
        x = os.urandom(3000000) #bytes(2100000)
        t = time.time()
        a.attrs["cam_image"].value = np.frombuffer(x, dtype=np.uint8)
        t2 = time.time() - t
        print(t2)

    print("Update value list")
    for i in range(0,3):
        x = list(os.urandom(3000000))
        t = time.time()
        a.attrs["cam_image"].value = x
        t2 = time.time() - t
        print(t2)

    print("Get attribute")
    for i in range(0,3):
        t = time.time()
        img = a.attrs["cam_image"].value
        t2 = time.time() - t
        print(t2)
        print(type(img))

main()
