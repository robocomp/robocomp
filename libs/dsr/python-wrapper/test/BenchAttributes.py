
"""
def main():

    #x = threading.Thread(target=profile)
    #x.start()

    a = Node(12, "world", "elmundo")
    a.attrs["cam_image"] = Attribute([100], 1)

    print("Update value creating attribute using numpy array")
    for i in range(0,300):
        x = os.urandom(3000000) #bytes(2100000)
        t = time.time_ns()
        a.attrs["cam_image"].value = np.frombuffer(x, dtype=np.uint8)
        t2 = time.time_ns() - t
        print(t2)

    print("Update value creating attribute numpy array")
    for i in range(0,3):
        x = list(os.urandom(3000000))
        t = time.time_ns()
        a.attrs["cam_image"].value = x
        t2 = time.time_ns() - t
        print(t2)

    print("Update value numpy array")
    for i in range(0,3):
        x = os.urandom(3000000) #bytes(2100000)
        t = time.time_ns()
        a.attrs["cam_image"].value = np.frombuffer(x, dtype=np.uint8)
        t2 = time.time_ns() - t
        print(t2)

    print("Update value list")
    for i in range(0,3):
        x = list(os.urandom(3000000))
        t = time.time_ns()
        a.attrs["cam_image"].value = x
        t2 = time.time_ns() - t
        print(t2)

    print("Get attribute")
    for i in range(0,3):
        t = time.time_ns()
        img = a.attrs["cam_image"].value
        t2 = time.time_ns() - t
        print(t2)
        print(type(img))

main()
"""

from pydsr import *
import numpy as np
import time
import os

def insert_with_size (id, size, file):
    for i in range(0,300):
        a = G.get_node(id)
        x = os.urandom(size) #bytes(2100000)
        arr = np.frombuffer(x, dtype=np.uint8)
        t = time.time_ns()
        a.attrs["cam_image"].value = arr
        R = G.update_node(a)
        t2 = time.time_ns()
        file.write(f"{t};{t2};{size}\n")
        time.sleep(0.02)


def insert_node (file):
    for i in range(0,300):
        t = time.time_ns()
        a = Node(12, "testtype", "unnodo")
        id = G.insert_node(a)
        t2 = time.time_ns()
        file.write(f"{t};{t2};insert_node\n")
        time.sleep(0.02)
        t = time.time_ns()
        G.delete_node(id)
        t2 = time.time_ns()
        file.write(f"{t};{t2};delete_node\n")
        time.sleep(0.02)


def insert_edge (id, file):
    for i in range(0,300):
        a = Node(12, "testtype", "unnodo")
        id2 = G.insert_node(a)
        t = time.time_ns()
        e = Edge(id2, id, "testtype_e", 89)
        G.insert_or_assign_edge(e)
        t2 = time.time_ns()
        file.write(f"{t};{t2};insert_edge\n")
        time.sleep(0.02)
        t = time.time_ns()
        G.delete_edge(id, id2, "testtype_e")
        t2 = time.time_ns()
        file.write(f"{t};{t2};delete_edge\n")
        time.sleep(0.02)

with open("results.txt", "a") as file:
    G = DSRGraph(0, "pythonagent", 89)

    a = Node(12, "testtype", "unnodo")
    a.attrs["cam_image"] = Attribute([100], 1)
    id = G.insert_node(a)

    for i in [500, 1000, 2000, 5000, 25000, 60000, 150000, 500000, 1000000, 2000000, 5000000, 10000000]:
        insert_with_size(id, i, file)


    insert_node(file)
    insert_edge(id, file)