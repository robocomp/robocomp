#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2019 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.


"""
TODO: add generate_ methods for the specific resultNames of some nodes.
TODO: functions could be "installed" for combination of node type and resultName, that way specific actions could be taken

TODO: add a new mode to generate not random cdsl but try to get all the options of the grammar generated/tested in several components
"""

import os
import random
import pyparsing
import sys

from dsl_parsers.dsl_factory import DSLFactory

sys.path.append('/opt/robocomp/python/robocompdsl')

from dsl_parsers.specific_parsers.cdsl.cdsl_parser import CDSLParser


try:
    from rich.console import Console
    console = Console()

    def printerr(text):
        console.log(text, color='red')
except ImportError as error:
    def printerr(text):
        print(text)

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
IDSL_DIR = os.path.join(ROBOCOMP, 'interfaces', 'IDSLs')

MAX_IMPORTS = 4

COMPONENT_NAMES = ["Camerasy", "VisionRobot", "DevTecnology", "IlluminateDeep", "CyperCamera", "FissionTecnology",
                   "DeskRobot", "MechaNeuronal", "PointCamera", "WireRandom", "Neuronry", "CatalystNeuronal",
                   "CacaComp", "Tecnologyworks", "Tecnologycog", "Camerado", "IntuitionTecnology", "DomainDeep",
                   "OmegaNeuronal",
                   "Camerax", "ControlDeep", "CheckTecnology", "MachRobot", "ElectricCamera", "ElectricTecnology",
                   "Neurongenix", "KeyRobot", "Tecnologycouch", "NomadDeep", "DeskRandom", "EdgeNeuronal", "Robotava",
                   "AcumenNeuronal", "AlgorithmNeuronal", "OverdriveDeep", "LinearDeep", "TitanRobot",
                   "ContinuumCamera", "LevelDeep", "AlgorithmRobot", "VeritasDeep", "ArclightCamera",
                   "Randomava", "NestNeuronal", "GraphicsTecnology", "ScopeDeep", "ModelCamera", "OptimalTecnology",
                   "SurgeRandom", "Robotzilla", "FiberNeuronal", "AtlasDeep", "ExpressionRandom", "ChromaDeep",
                   "DynamicsRandom", "CyperDeep", "AlgorithmDeep", "DevCamera", "OptimizeDeep", "ChromeRandom",
                   "SoftNeuronal", "ChipTecnology", "AugurTecnology", "PingDeep", "GigaRobot", "InsightTecnology",
                   "MechaRobot", "MachDeep", "BitRandom", "PixelRobot", "AltRandom", "Randomworks", "Deeppad",
                   "InteractiveDeep", "LinkTecnology", "TetraRobot", "Deepya", "SonicDeep", "CommandDeep",
                   "CatalystRandom", "InfoCamera", "Cameravio", "SoftDeep", "SecureNeuronal", "AgileCamera",
                   "Neuronhut", "SpireDeep", "CoreNeuronal", "NovusRobot", "DiskDeep", "Neuronlux", "MacroTecnology",
                   "LinkRandom", "IntegrationRandom", "AtlasNeuronal", "PingTecnology", "NetworkNeuronal"]


def get_available_idsls():
    idsls_in_dir = []
    for file in os.listdir(IDSL_DIR):
        if file.endswith(".idsl") and "agm" not in file.lower():
            idsls_in_dir.append(file)
    return idsls_in_dir


class CDSLSampler:
    def __init__(self, ros_enable=False):
        self._available_idsls = []
        self._available_interfaces = []
        self._used_idsl = []
        self._ros_enable = ros_enable
        self._tabs = 0
        self._last_keywords = []
        self._special_interfaces = set()
        self._nodes_with_names = {}

    def _tab(self, text, before=True):
        tabs = ''
        for i in range(self._tabs):
            tabs += "\t"
        if before:
            return tabs + text
        else:
            return text + tabs

    def generate_valid_component(self, grammar):
        """
        Initialice the needed attributes and start the generation

        :param grammar:
        :return:
        """
        self._available_idsls = get_available_idsls()
        self._used_idsl = []
        self._last_keywords = []
        self._nodes_with_names = {}
        return self.generic_sampler(grammar)

    def generic_sampler(self, node):
        """
        Process each node depending on the type of it.

        :param node:
        :return: string of the resultin call to the method for that type of node
        """

        if node.resultsName is not None:
            # setdefault(type(node), set()) is a quick way to check if key exist and if not create an empty set to add to
            self._nodes_with_names.setdefault(type(node), set()).add(node.resultsName)

        if isinstance(node,pyparsing.And):
            # print(tab + 'And')
            return self.generate_and(node)

        elif isinstance(node,pyparsing.Or):
            # print(tab + 'Or')
            return self.generate_or(node)

        elif isinstance(node,pyparsing.ZeroOrMore):
            # print(tab + 'ZeroOrMore')
            return self.generate_zero_or_more(node)

        elif isinstance(node,pyparsing.Each):
            return self.generate_each(node)

        elif isinstance(node,pyparsing.Optional):
            # print(tab + 'Optional')
            return self.generate_optional(node)

        elif isinstance(node,pyparsing.Suppress):
            # print(tab + 'Supress', node.expr)
            return self.generic_sampler(node.expr)

        elif isinstance(node,pyparsing.Literal):
            # print(tab + 'Literal', node.match)
            return self.generate_literal(node)

        elif isinstance(node,pyparsing.Group):
            # print(tab + 'Group')
            return self.generate_group(node)

        elif isinstance(node,pyparsing.MatchFirst):
            # print(tab + 'MatchFirst')
            return self.generate_match_first(node)

        elif isinstance(node,pyparsing.CaselessKeyword):
            # print(tab + 'CaselessKeyword', node.match)
            return self.generate_caseless_keyword(node)

        elif isinstance(node,pyparsing.Word):
            # print(tab + 'Word', node.re)
            return self.generate_word(node)

        elif isinstance(node,pyparsing.CharsNotIn):
            # print(tab + 'CharsNotIn')
            return self.generate_chars_not_in(node)

        elif 'ErrorStop' in str(type(node)):
            # print(tab + '_ErrorStop')
            return ''
        else:
            raise ValueError('Unknown type %s in a node' % type(node))

    def generate_and(self, node):
        """
        Process and return the string that join the result of all the options on the And

        :param node: node to be processed
        :return: text generated by the each expression
        """
        text = ''
        for child in node.exprs:
            text += self.generic_sampler(child)  # + ' '
        return str(text)

    def generate_each(self, node):
        """
        Convenience method that do the same than generate_and

        :param node: node to be processed
        :return: text generated by the each expression
        """
        # it currently do the same becuause the only difference is the order and it's not contemplated right now
        return self.generate_and(node)

    def generate_or(self, node):
        """
        Return the result of one of multiple options

        :param node: node to be processed
        :return: text generated by the random selected option
        """
        text = ''
        random_child = random.choice(node)
        text += self.generic_sampler(random_child)  # + ' '
        return str(text)

    def generate_zero_or_more(self, node):
        text = ''
        times_to_repeat = random.randint(0, MAX_IMPORTS)
        for count in range(times_to_repeat):
            text += self.generic_sampler(node.expr)  # + ' '
        return str(text)

    def generate_group(self, node):
        """
        Process special cases of groups. If there's no available interface, no communication must be generated.

        :param node:
        :return:
        """
        # if the group is "communications" and no IDSL have been imported return empty string ''
        if node.resultsName == "communications" and len(self._available_interfaces) == 0:
            return ''
        else:
            return self.generic_sampler(node.expr)

    def generate_match_first(self, node):
        """
        Return one random option from several defined on the node

        :param node: node to be processed
        :return: only one of multiple options
        """

        # TODO: add some option for weighted random
        text = ''
        random_child = random.choice(node.exprs)
        text += self.generic_sampler(random_child)
        return text

    def generate_optional(self, node):
        """
        Value or empty string is returned based on a random boolean

        :param node: node to be processed
        :return: Value of the inner node or empty string ''
        """
        option = bool(random.getrandbits(1))
        if option:
            return self.generic_sampler(node.expr)
        else:
            return ''

    def generate_caseless_keyword(self, node):
        """
        Process and return the string for a keywords.
        It's taking care of keywords that need to have an <space> after them

        :param node: node to be processed
        :return: keyword processed
        """
        # It's a way to add spaces to the needed keywords
        spaced_keywords = """import language component useQt gui Qt
        requires implements subscribesTo publishes options
        InnerModelViewer statemachine""".split()

        self._last_keywords.append(str(node.match))
        if node.match in spaced_keywords:
            return str(node.match) + " "
        else:
            return str(node.match)

    def generate_literal(self, node):
        """
        Process the literals from the grammar.
        Special cases for tabs and new lines are checked and treated

        :param node:
        :return: text treated to have new lines and tabs
        """
        text = ''
        if node.match in '{':
            self._tabs += 1
        if node.match in '}':
            self._tabs -= 1
        if node.match in ';{':
            text += "\n"
        # text = self._tab(text, before=False)

        # It's a way to add spaces to the needed literals
        spaced_literals = """,""".split()
        if node.match in spaced_literals:
            return str(node.match) + " "

        return str(node.match) + text

    def generate_word(self, node):
        """
        Process some special words based on the resultName of the node

        :param node: node to be processed
        :return: resultName dependant strings or just the characters of the word
        """
        if node.resultsName is not None:
            return self.generate_by_result_name(node)
        else:
            return node.initCharsOrig

    def generate_chars_not_in(self, node):
        """
        Process some special words based on the resultName of the node

        :param node: node to be processed
        :return: resultName dependant strings or just the characters of the chars
        """
        if node.resultsName is not None:
            return self.generate_by_result_name(node)
        else:
            return 'CharsNotIn(' + node.notChars + ')'

    def generate_by_result_name(self, node):
        """
        Return resultName dependant strings.
        One special case is for the strings names of the interfaces of the communication.

        :param node: node to be processed
        :return: resultName dependant strings
        """

        if node.resultsName == 'idsl_path':
            return self.get_random_idsl()
        if node.resultsName == 'name':
            return random.choice(COMPONENT_NAMES)
        if node.resultsName == 'machine_path':
            return "statemachine.smdsl"
        comm_identifiers = ['reqIdentifier', 'pubIdentifier', 'impIdentifier', 'subIdentifier']
        if any(node.resultsName == name for name in comm_identifiers):
            if len(self._available_interfaces) > 0:
                return random.choice(self._available_interfaces)
            else:
                # It would never arrive here becuase a previous check on communications of len(self._available_interfaces) > 0
                return "<" + str(node.resultsName) + ">"
        else:
            raise ValueError('Unknown result name to generate for: %s' % node.resultsName)


    def get_random_idsl(self):
        """
        Return an available .idsl filename from available ones, remove from that list,
        add it to the _used_idsls list and check if the .idsl have an available interface inside.

        :return: random chosen .idsl filename from available ones
        """
        next_idsl = None
        if len(self._available_idsls) > 0:
            next_idsl = random.choice(self._available_idsls)
            try:
                interface_name = self.get_interface_name_for_idsl(next_idsl)
                self._available_interfaces.append(interface_name)
            except:
                self._special_interfaces.add(next_idsl)
                printerr("IDSL %s without interface" % next_idsl)
            self._available_idsls.remove(next_idsl)
            self._used_idsl.append(next_idsl)
        return next_idsl

    def get_interface_name_for_idsl(self, idsl_filename):
        """
        Extract the name of the interface for a idsl file.

        :param idsl_filename: .idsl file name
        :return: name of the interface defined inside the .idsl file.
        """
        try:
            interface_name = DSLFactory().from_file(idsl_filename)['interfaces'][0]['name']
        except:
            # There's some .idsl files without interfaces defined on it, just data structures definitions
            raise ValueError("Couldn't get the interface name for idsl file: %s" % idsl_filename)
        else:
            return interface_name

    @property
    def special_interfaces(self):
        return self._special_interfaces

    @property
    def nodes_with_names(self):
        return self._nodes_with_names


if __name__ == '__main__':
    root = CDSLParser().parser
    generator = CDSLSampler()
    for i in range(100):
        output_cdsl = generator.generate_valid_component(root)
        print(output_cdsl)
    # print(generator.special_interfaces)
    # pprint(generator.nodes_with_names)
