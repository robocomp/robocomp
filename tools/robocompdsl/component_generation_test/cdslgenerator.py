import random

import pyparsing
import sys

sys.path.append('/opt/robocomp/python')
import parseCDSL

from pprint import pprint

AVAILABLE_INTERFACES = {
	"Apriltags.idsl":
		{
			"name": "AprilTags",
			"communications": "implements, requires"
		}
}

MAX_IMPORTS = 4



def generic_sampler(node, tab='', main_tag = ''):
	if type(node) == pyparsing.And:
		# print(tab + 'And')
		return generate_and(node, tab)
	elif type(node) == pyparsing.Or:
		# print(tab + 'Or')
		return generate_or(node, tab)
	elif type(node) == pyparsing.ZeroOrMore:
		# print(tab + 'ZeroOrMore')
		return generate_zero_or_more(node, tab)
	elif type(node) == pyparsing.Suppress:
		# print(tab + 'Supress', node.expr)
		return generic_sampler(node.expr)
	elif type(node) == pyparsing.CaselessKeyword:
		# print(tab + 'CaselessKeyword', node.match)
		return str(node.match)
	elif 'ErrorStop' in str(type(node)):
		# print(tab + '_ErrorStop')
		return ''
	elif type(node) == pyparsing.Word:
		# print(tab + 'Word', node.re)
		return generate_word(node, tab)
	elif type(node) == pyparsing.CharsNotIn:
		# print(tab + 'CharsNotIn')
		return generate_chars_not_in(node, tab)
	elif type(node) == pyparsing.Literal:
		# print(tab + 'Literal', node.match)
		return generate_literal(node, tab)
	elif type(node) == pyparsing.Group:
		# print(tab + 'Group')
		return generic_sampler(node.expr)
	elif type(node) == pyparsing.MatchFirst:
		# print(tab + 'MatchFirst')
		return generate_match_first(node, tab)
	elif type(node) == pyparsing.Optional:
		# print(tab + 'Optional')
		return generate_optional(node, tab)
	else:
		print("Unknown type")
		sys.exit(0)


def generate_literal(node, tab):
	if node.match in ';{':
		return str(node.match) + "\n"
	else:
		return str(node.match)


def generate_and(node, tab):
	text = ''
	for child in node:
		text += generic_sampler(child) + ' '
	return str(text)


def generate_or(node, tab):
	text = ''
	random_child = random.choice(node)
	text += generic_sampler(random_child) + ' '
	return str(text)


def generate_zero_or_more(node, tab):
	text = ''
	times_to_repeat = random.randint(0, MAX_IMPORTS)
	for count in range(times_to_repeat):
		text += generic_sampler(node.expr) + ' '
	return str(text)


def generate_word(node, tab):
	if node.resultsName is not None:
		return "<" + str(node.resultsName) + ">"
	else:
		return node.initCharsOrig


def generate_match_first(node, tab):
	# TODO: add somw option for weighted random
	text = ''
	random_child = random.choice(node.exprs)
	text += generic_sampler(random_child)
	return text


def generate_optional(node, tab):
	option = bool(random.getrandbits(1))
	if option:
		return generic_sampler(node.expr)
	else:
		return ''

def generate_chars_not_in(node, tab):

	if node.resultsName is not None:
		return "<" + node.resultsName + ">"
	else:
		return 'CharsNotIn(' + node.notChars + ')'


if __name__ == '__main__':
	root = parseCDSL.CDSLParsing.getCDSLParser()
	output_cdsl = generic_sampler(root)
