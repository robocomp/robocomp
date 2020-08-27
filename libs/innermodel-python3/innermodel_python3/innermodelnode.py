'''
Base class for all the elements in an innermodel
Author: ksakash@github.com (Akash Kumar Singh)
'''

import copy

class InnerModelNode (object):
    '''Base class for all the elements in an innermodel'''

    def __init__ (self, id: str, parent: 'InnerModelNode'):
        '''
        param id: identifier for the node
        param parent: parent of the node, None if root
        '''

        self.rtmat = None
        self.id = id
        self.innerModel = None
        self.parent = parent
        if self.parent is not None:
            self.level = parent.level + 1
        else:
            self.level = 0
        self.children = []
        self.attributes = None # redundant

    # can also use __repr__ instead
    def printTree (self, s: str, verbose: bool):
        '''Print the whole tree from the current node'''

        print ("{} id: {}, level: {}, num_children: {}".format(s, self.id, self.level,
                                                                                len(self.children)))

        for child in self.children:
            if verbose:
                child.print (verbose)
            child.printTree (s, verbose)

    # abstract
    def print (self, verbose: bool):
        '''Print info about the current node'''

        raise NotImplementedError

    # abstract
    def save (self, out, tabs):
        '''Save the current ndoe into a file'''

        raise NotImplementedError

    def setParent (self, parent: 'InnerModelNode'):
        '''Set the parent of the node to the given parent'''

        self.parent = parent
        self.level = parent.level + 1

    def addChild (self, child: 'InnerModelNode'):
        '''Add a child to the current node'''

        child.innerModel = self.innerModel
        if child not in self.children:
            self.children.append(child)
        child.parent = self

    def copyNode (self):
        ret = copy.deepcopy (self)
        return ret
