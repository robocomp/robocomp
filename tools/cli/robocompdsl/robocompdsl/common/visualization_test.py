import signal
import sys
from pprint import pprint

import networkx as nx
from PySide2.QtWidgets import QGraphicsView, QApplication

from QNetworkxGraph.QNetworkxGraph import QNetworkxWidget, QNetworkxController
from parseSMDSL import SMDSLparsing



if __name__ == '__main__':
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    result = SMDSLparsing.fromFile("/home/robolab/robocomp/components/euroage-tv/components/tvGames/gamestatemachine.smdsl")
    # result = SMDSLparsing.fromFile("/home/robolab/robocomp/tools/robocompdsl/component_generation_test/subStatesTest/statemachine.smdsl")
    pprint(result)
    matching_graph = nx.Graph()
    view = QNetworkxWidget()
    scene = QNetworkxController(view)

    if result["machine"]["contents"]["initialstate"] is not None:
        matching_graph.add_node(result["machine"]["contents"]["initialstate"])
    if result["machine"]["contents"]["finalstate"] is not None:
        matching_graph.add_node(result["machine"]["contents"]["finalstate"])
    if result["machine"]["contents"]["states"] is not None:
        for state in result["machine"]["contents"]["states"]:
            matching_graph.add_node(state)
    if result["machine"]["contents"]["transitions"] is not None:
        for transition in result["machine"]["contents"]["transitions"]:
            for dest in transition["dests"]:
                matching_graph.add_edge(transition["src"], dest)
    if result["substates"] is not None:
        for substate in result["substates"]:
            if substate["contents"]["initialstate"] is not None:
                matching_graph.add_node(substate["contents"]["initialstate"])
            if substate["contents"]["finalstate"] is not None:
                matching_graph.add_node(substate["contents"]["finalstate"])
            if substate["contents"]["states"] is not None:
                for state in substate["contents"]["states"]:
                    matching_graph.add_node(state)
            if substate["contents"]["transitions"] is not None:
                for transition in substate["contents"]["transitions"]:
                    for dest in transition["dests"]:
                        matching_graph.add_edge(transition["src"], dest)
            if substate["parent"] is not None:
                if substate["contents"]["initialstate"] is not None:
                    matching_graph.add_edge(substate["parent"], substate["contents"]["initialstate"])
                else:
                    matching_graph.add_edge(substate["parent"], substate["contents"]["states"][0])
    scene.set_graph(matching_graph)
    view.show()
    app.exec_()
