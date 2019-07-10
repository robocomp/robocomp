import os
import random

import pyparsing
import sys

sys.path.append('/opt/robocomp/python')
import parseCDSL
import parseIDSL

from pprint import pprint

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
IDSL_DIR = os.path.join(ROBOCOMP ,'interfaces','IDSLs')

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
	return os.listdir(IDSL_DIR)


MAX_IMPORTS = 4


class CDSLSampler:
	def __init__(self, ros_enable=False):
		self._available_idsls = get_available_idsls()
		self._used_idsl = []
		self._ros_enable = ros_enable
		self._tabs = 0
		self._last_keywords = []

	def _tab(self, text, before = True):
		tabs = ''
		for i in range(self._tabs):
			tabs+="\t"
		if before:
			return tabs + text
		else:
			return text+tabs

	def generic_sampler(self, node, main_tag=''):
		if type(node) == pyparsing.And:
			# print(tab + 'And')
			return self.generate_and(node)
		elif type(node) == pyparsing.Or:
			# print(tab + 'Or')
			return self.generate_or(node)
		elif type(node) == pyparsing.ZeroOrMore:
			# print(tab + 'ZeroOrMore')
			return self.generate_zero_or_more(node)
		elif type(node) == pyparsing.Suppress:
			# print(tab + 'Supress', node.expr)
			return self.generic_sampler(node.expr)
		elif type(node) == pyparsing.CaselessKeyword:
			# print(tab + 'CaselessKeyword', node.match)
			return self.generate_caseless_keyword(node)
		elif 'ErrorStop' in str(type(node)):
			# print(tab + '_ErrorStop')
			return ''
		elif type(node) == pyparsing.Word:
			# print(tab + 'Word', node.re)
			return self.generate_word(node)
		elif type(node) == pyparsing.CharsNotIn:
			# print(tab + 'CharsNotIn')
			return self.generate_chars_not_in(node)
		elif type(node) == pyparsing.Literal:
			# print(tab + 'Literal', node.match)
			return self.generate_literal(node)
		elif type(node) == pyparsing.Group:
			# print(tab + 'Group')
			return self.generate_group(node)
		elif type(node) == pyparsing.MatchFirst:
			# print(tab + 'MatchFirst')
			return self.generate_match_first(node)
		elif type(node) == pyparsing.Optional:
			# print(tab + 'Optional')
			return self.generate_optional(node)
		elif type(node) == pyparsing.Each:
			return self.generate_each(node)
		else:
			print("Unknown type")
			print(type(node))
			sys.exit(0)

	def generate_and(self, node):
		text = ''
		for child in node:
			text += self.generic_sampler(child)  # + ' '
		return str(text)

	def generate_or(self, node):
		text = ''
		random_child = random.choice(node)
		text += self.generic_sampler(random_child)  # + ' '
		return str(text)

	def generate_zero_or_more(self, node):
		text = ''
		times_to_repeat = random.randint(0, MAX_IMPORTS)
		# TODO: look for a better way to check if is the clause for implement|require|subscribes|publishes
		# DEPRECATED: communications now are an EACH statement
		if "implements" in str(node.expr):
			# it's the communications generation.
			if len(self._used_idsl) > 0:
				# 4 different types of communications
				times_to_repeat = random.randint(0, 4)
			else:
				# if no IDSL is imported no communication should be generated
				times_to_repeat = 0
		for count in range(times_to_repeat):
			text += self.generic_sampler(node.expr)  # + ' '
		return str(text)

	def generate_each(self, node):
		text = ''
		# if it a requires state
		for child in node.exprs:
			text += self.generic_sampler(child)  # + ' '
		return str(text)

	def generate_group(self, node):
		# if the group is "communications" and no IDSL have been imported return empty string ''
		if node.resultsName == "communications" and len(self._used_idsl) == 0:
			return ''
		else:
			return self.generic_sampler(node.expr)

	def generate_match_first(self, node):
		# TODO: add some option for weighted random
		text = ''
		random_child = random.choice(node.exprs)
		text += self.generic_sampler(random_child)
		return text

	def generate_optional(self, node):
		option = bool(random.getrandbits(1))
		if option:
			# # TODO: Look for a better way to check if its ros or not
			# if len(str(node.expr)) < 30 and "{\"ice\" | \"ros\"}" in str(node.expr) and not self._ros_enable:
			# 	return ''
			# else:
			return self.generic_sampler(node.expr)
		else:
			return ''

	def generate_caseless_keyword(self, node):
		spaced_keywords = """import language component useQt gui Qt qt4 qt5
		requires implements subscribesTo publishes options
		InnerModelViewer statemachine""".split()
		# It's a way to add spaces to the needed keywords
		self._last_keywords.append(str(node.match))
		if node.match in spaced_keywords:
			return str(node.match) + " "
		else:
			return str(node.match)

	def generate_literal(self, node):
		"""

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


		return str(node.match) + text


	def generate_word(self, node):
		if node.resultsName is not None:
			return self.generate_by_result_name(node)
		else:
			return node.initCharsOrig

	def generate_chars_not_in(self, node):
		if node.resultsName is not None:
			return self.generate_by_result_name(node)
		else:
			return 'CharsNotIn(' + node.notChars + ')'

	def generate_by_result_name(self, node):
		# TODO: Change path name on parseCDSL for a more descriptive name
		if node.resultsName == 'idsl_path':
			return self.get_random_idsl()
		if node.resultsName == 'name':
			return random.choice(COMPONENT_NAMES)
		comm_identifiers = ['reqIdentifier', 'pubIdentifier', 'impIdentifier', 'subIdentifier']
		if any(node.resultsName == name for name in comm_identifiers):
			if len(self._used_idsl) > 0:
				return self.get_interface_name_for_idsl(random.choice(self._used_idsl))
			else:
				# TODO: it's not an option to return none. It
				return "<" + str(node.resultsName) + ">"
		else:
			return "<" + str(node.resultsName) + ">"

	def get_random_idsl(self):
		next_idsl = None
		if len(self._available_idsls) > 0:
			next_idsl = random.choice(self._available_idsls)
			self._available_idsls.remove(next_idsl)
			self._used_idsl.append(next_idsl)
		return next_idsl

	def get_interface_name_for_idsl(self, idsl_filename):
		try:
			interface_name = parseIDSL.IDSLParsing.gimmeIDSL(idsl_filename)['interfaces'][0]['name']
		except:
			print("ERR: Couldn't get the interface name for idsl file: %s"%idsl_filename)
			return basestring(idsl_filename)
		else:
			return interface_name


if __name__ == '__main__':
	root = parseCDSL.CDSLParsing.getCDSLParser()
	generator = CDSLSampler()
	output_cdsl = generator.generic_sampler(root)
	print(output_cdsl)
