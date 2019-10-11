import copy
import sys

from pyparsing import CaselessKeyword, Suppress, Word, CaselessLiteral, alphas, alphanums, delimitedList, Group, \
    OneOrMore, ZeroOrMore, Optional, cppStyleComment

from dsl_parsers.dsl_parser_abstract import DSLParserTemplate


class SMDSLParser(DSLParserTemplate):
    def __init__(self):
        super(SMDSLParser, self).__init__()

    def _create_parser(self):


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

        substate = parent + Optional(PARALLEL.setResultsName('parallel').setParseAction(lambda t: True)) + Group(
            machine_content).setResultsName("contents")

        machine = identifier.setResultsName("name") + Group(machine_content).setResultsName("contents") + Group(
            ZeroOrMore(Group(substate))).setResultsName("substates")

        SMDSL = Group(machine.ignore(cppStyleComment)).setResultsName("machine")

        return SMDSL

    def string_to_struct(self, string):
        parsing_result = self.parse_string(string)
        result_dict = {'machine': {}}

        # Horrible hack to make robocompdsl work with pyparsing > 2.2
        try:
            result_dict['machine']['name'] = parsing_result['machine']['name']
        except:
            result_dict['machine']['name'] = parsing_result['name']
            parsing_result['machine'] = copy.deepcopy(parsing_result)

        if result_dict['machine']['name'] == "defaultMachine":
            result_dict['machine']['default'] = True
        else:
            result_dict['machine']['default'] = False
        result_dict['machine']['contents'] = {}
        try:
            result_dict['machine']['contents']['states'] = parsing_result['machine']['contents']['states']
        except:
            result_dict['machine']['contents']['states'] = None
        try:
            result_dict['machine']['contents']['finalstate'] = parsing_result['machine']['contents']['finalstate']
        except:
            result_dict['machine']['contents']['finalstate'] = None
        else:
            if result_dict['machine']['contents']['states'] is not None:
                for state in result_dict['machine']['contents']['states']:
                    if result_dict['machine']['contents']['finalstate'] == state:
                        print(
                        ("Error: this final state " + result_dict['machine']['contents']['finalstate'] + " is in states"))
        try:

            result_dict['machine']['contents']['initialstate'] = parsing_result['machine']['contents']['initialstate']
        except:
            print("Error: The state machine needs initial state")
        else:
            if result_dict['machine']['contents']['states'] is not None:
                for state in result_dict['machine']['contents']['states']:
                    if result_dict['machine']['contents']['initialstate'] == state:
                        print(("Error: this initial state " + result_dict['machine']['contents'][
                            'initialstate'] + " is in states"))

            if result_dict['machine']['contents']['finalstate'] is not None:
                if result_dict['machine']['contents']['initialstate'] == result_dict['machine']['contents']['finalstate']:
                    print("Error: initial state is equal final state")

        try:
            result_dict['machine']['contents']['transitions'] = parsing_result['machine']['contents']['transitions']
        except:
            result_dict['machine']['contents']['transitions'] = None
        try:
            aux = parsing_result['machine']['substates']
            result_dict['substates'] = []
        except:
            result_dict['substates'] = None
        if result_dict['substates'] is not None:
            for sub in parsing_result['machine']['substates']:
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
                    # TODO: Raise exception. Don't Exit.
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
                                    print(("Error: substate " + a['parent'] + " this final state " + a['contents'][
                                        'finalstate'] + " is in states"))
                    try:
                        a['contents']['initialstate'] = sub['contents']['initialstate']
                    except:
                        print(("Error substate " + a['parent'] + " needs initial state"))
                    else:
                        if a['contents']['states'] is not None:
                            for state in a['contents']['states']:
                                if a['contents']['initialstate'] == state:
                                    print(("Error: " + a['parent'] + " this initial state " + a['contents'][
                                        'initialstate'] + " is in states"))
                        if a['contents']['initialstate'] == a['contents']['finalstate']:
                            print(("Error: " + a['parent'] + " initial state is equal final state"))
                try:
                    a['contents']['transitions'] = sub['contents']['transitions']
                except:
                    a['contents']['transitions'] = None
                result_dict['substates'].append(a)
        self.struct = result_dict
        return result_dict