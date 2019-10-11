from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine, CaselessKeyword, \
    delimitedList
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, CaselessLiteral

import sys, traceback, os
import copy


class BColors(str):
    """ Class to print with colors on stdout"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    def __new__(cls, value, type):
        obj = type+str.__new__(cls, value)+cls.ENDC
        return obj


class SMDSLparsing(object):
    @staticmethod
    def fromFile(filename, verbose=False, includeIncludes=True):
        # Open input file
        # inputText = "\n".join([line for line in open(filename, 'r').read().split("\n") if not line.lstrip(" \t").startswith('//')])
        if filename is None or filename == "":
            return None
        try:
            inputText = open(filename, 'r').read()
        except:
            print("Error reading input file for SMDSL")
            return None
        try:
            ret = SMDSLparsing.fromString(inputText)
        except:
            print((BColors('Error parsing %s'%filename,BColors.FAIL)))
            traceback.print_exc()
            sys.exit(-1)
        ret['filename'] = filename
        return ret

    @staticmethod
    def fromString(inputText, verbose=False):
        if verbose: print(('Verbose:', verbose))

        (TRANSITIONS, INITIAL_STATE, END_STATE, STATES, PARALLEL) = list(map(CaselessKeyword, """
        		transitions initial_state end_state states parallel""".split()))

        semicolon = Suppress(Word(";"))
        op = Suppress(Word("{"))
        cl = Suppress(Word("}"))
        to = Suppress(CaselessLiteral("=>"))

        identifier = Word(alphas + "_", alphanums + "_")

        list_identifiers = delimitedList(identifier)

        # parse States
        stateslist = Group(Suppress(STATES) + list_identifiers + semicolon).setResultsName('states')

        # parse Transitions
        transition = identifier.setResultsName('src') + to + list_identifiers.setResultsName('dests') + semicolon
        transitions_list = Group(OneOrMore(Group(transition))).setResultsName("transitions")
        transitions = Suppress(TRANSITIONS) + op + transitions_list + cl + semicolon

        # parse initialstate and finalstate
        initialstate = Suppress(INITIAL_STATE) + identifier.setResultsName('initialstate') + semicolon
        finalstate = Suppress(END_STATE) + identifier.setResultsName('finalstate') + semicolon

        # parse machine
        contents = stateslist | initialstate | finalstate | transitions
        machine_content = op + ZeroOrMore(contents) + cl + semicolon

        parent = Suppress(Word(":")) + identifier.setResultsName('parent')

        substate = parent + Optional(PARALLEL.setResultsName('parallel').setParseAction(lambda t: True)) + Group(machine_content).setResultsName("contents")

        machine = identifier.setResultsName("name") + Group(machine_content).setResultsName("contents") + Group(ZeroOrMore(Group(substate))).setResultsName("substates")

        SMDSL = Group(machine.ignore(cppStyleComment)).setResultsName("machine")

        tree= SMDSL.parseString(inputText)
        return SMDSLparsing.component(tree)

    @staticmethod
    def component(tree, start=''):
        component = {}
        component['machine'] = {}

        # Horrible hack to make robocompdsl work with pyparsing > 2.2
        try:
            component['machine']['name'] = tree['machine']['name']
        except:
            component['machine']['name'] = tree['name']
            tree['machine'] = copy.deepcopy(tree)


        if component['machine']['name'] == "defaultMachine":
            component['machine']['default'] = True
        else:
            component['machine']['default'] = False
        component['machine']['contents'] = {}
        try:
            component['machine']['contents']['states'] = tree['machine']['contents']['states']
        except:
            component['machine']['contents']['states'] = None
        try:
            component['machine']['contents']['finalstate'] = tree['machine']['contents']['finalstate']
        except:
            component['machine']['contents']['finalstate'] = None
        else:
            if component['machine']['contents']['states'] is not None:
                for state in component['machine']['contents']['states']:
                    if component['machine']['contents']['finalstate'] == state:
                        print(("Error: this final state " + component['machine']['contents']['finalstate'] + " is in states"))
        try:

            component['machine']['contents']['initialstate'] = tree['machine']['contents']['initialstate']
        except:
            print("Error: The state machine needs initial state")
        else:
            if component['machine']['contents']['states'] is not None:
                for state in component['machine']['contents']['states']:
                    if component['machine']['contents']['initialstate'] == state:
                        print(("Error: this initial state " + component['machine']['contents'][
                            'initialstate'] + " is in states"))

            if component['machine']['contents']['finalstate'] is not None:
                if component['machine']['contents']['initialstate'] == component['machine']['contents']['finalstate']:
                    print("Error: initial state is equal final state")

        try:
            component['machine']['contents']['transitions'] = tree['machine']['contents']['transitions']
        except:
            component['machine']['contents']['transitions'] = None
        try:
            aux = tree['machine']['substates']
            component['substates'] = []
        except:
            component['substates'] = None
        if component['substates'] is not None:
            for sub in tree['machine']['substates']:
                a = {}
                # a['name'] = sub['name']
                try:
                    a['parallel'] = sub['parallel']
                except:
                    a['parallel'] = False
                try:
                    a['parent'] = sub['parent']
                except:
                    print("Error: substate missing parent")
                    sys.exit(-1)
                a['contents'] = {}
                if a['parallel']:
                    try:
                        a['contents']['states'] = sub['contents']['states']
                    except:
                        print(("Error: substate " + a['parent'] + " missing states"))
                    try:
                        a['contents']['finalstate'] = sub['contents']['finalstate']
                        print(("Error substate " + a['parent'] + " can't have final state"))
                    except:
                        a['contents']['finalstate'] = None
                    try:
                        a['contents']['initialstate'] = sub['contents']['initialstate']
                        print(("Error substate " + a['parent'] + " can't have initial state"))
                    except:
                        a['contents']['initialstate'] = None
                else:
                    try:
                        a['contents']['states'] = sub['contents']['states']
                    except:
                        a['contents']['states'] = None
                    try:
                        a['contents']['finalstate'] = sub['contents']['finalstate']

                    except:
                        a['contents']['finalstate'] = None
                    else:
                        if a['contents']['states'] is not None:
                            for state in a['contents']['states']:
                                if a['contents']['finalstate'] == state:
                                    print(("Error: substate " + a['parent'] + " this final state " + a['contents']['finalstate'] + " is in states"))
                    try:
                        a['contents']['initialstate'] = sub['contents']['initialstate']
                    except:
                        print(("Error substate " + a['parent'] + " needs initial state"))
                    else:
                        if a['contents']['states'] is not None:
                            for state in a['contents']['states']:
                                if a['contents']['initialstate'] == state:
                                    print(("Error: " + a['parent'] + " this initial state " + a['contents']['initialstate'] + " is in states"))
                        if a['contents']['initialstate'] == a['contents']['finalstate']:
                            print(("Error: " + a['parent'] + " initial state is equal final state"))
                try:
                    a['contents']['transitions'] = sub['contents']['transitions']
                except:
                    a['contents']['transitions'] = None
                component['substates'].append(a)

        return component

if __name__ == '__main__':
    from pprint import pprint
    pprint(SMDSLparsing.fromFile(sys.argv[1]))
