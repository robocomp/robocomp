from collections import OrderedDict
from operator import attrgetter

from pyparsing import Suppress, Word, alphas, alphanums, Group, \
    OneOrMore, ZeroOrMore, Optional, cppStyleComment, Literal, CharsNotIn

from dsl_parsers.dsl_parser_abstract import DSLParserTemplate
from dsl_parsers.parsing_utils import generate_recursive_imports


class IDSLParser(DSLParserTemplate):
    def __init__(self):
        super(IDSLParser, self).__init__()

    def _create_parser(self):

        semicolon = Suppress(Word(";"))
        quote = Suppress(Word("\""))
        op = Suppress(Word("{"))
        cl = Suppress(Word("}"))
        opp = Suppress(Word("("))
        clp = Suppress(Word(")"))
        lt = Suppress(Word("<"))
        gt = Suppress(Word(">"))
        eq = Suppress(Word("="))
        identifier = Word(alphas + "_", alphanums + "_")
        typeIdentifier = Word(alphas + "_", alphanums + "_:")
        structIdentifer = Group(
            typeIdentifier.setResultsName('type') + identifier.setResultsName('identifier') + Optional(eq) + Optional(
                CharsNotIn(";").setResultsName('defaultValue')) + semicolon)
        structIdentifers = Group(OneOrMore(structIdentifer))

        ## Imports
        idslImport = Suppress(Word("import")) + quote + CharsNotIn("\";").setResultsName('path') + quote + semicolon
        idslImports = ZeroOrMore(idslImport)

        structDef = Word("struct").setResultsName('type') + identifier.setResultsName(
            'name') + op + structIdentifers.setResultsName("structIdentifiers") + cl + semicolon
        dictionaryDef = Word("dictionary").setResultsName('type') + lt + CharsNotIn("<>").setResultsName(
            'content') + gt + identifier.setResultsName('name') + semicolon
        sequenceDef = Word("sequence").setResultsName('type') + lt + typeIdentifier.setResultsName(
            'typeSequence') + gt + identifier.setResultsName('name') + semicolon
        enumDef = Word("enum").setResultsName('type') + identifier.setResultsName('name') + op + CharsNotIn(
            "{}").setResultsName('content') + cl + semicolon
        exceptionDef = Word("exception").setResultsName('type') + identifier.setResultsName('name') + op + CharsNotIn(
            "{}").setResultsName('content') + cl + semicolon

        raiseDef = Suppress(Word("throws")) + typeIdentifier + ZeroOrMore(Literal(',') + typeIdentifier)
        decoratorDef = Literal('idempotent') | Literal('out')
        retValDef = typeIdentifier.setResultsName('ret')

        firstParam = Group(Optional(decoratorDef.setResultsName('decorator')) + typeIdentifier.setResultsName(
            'type') + identifier.setResultsName('name'))
        nextParam = Suppress(Word(',')) + firstParam
        params = firstParam + ZeroOrMore(nextParam)

        remoteMethodDef = Group(Optional(decoratorDef.setResultsName('decorator')) + retValDef.setResultsName(
            'ret') + typeIdentifier.setResultsName('name') + opp + Optional(params).setResultsName(
            'params') + clp + Optional(raiseDef.setResultsName('raise')) + semicolon)
        interfaceDef = Word('interface').setResultsName('type') + typeIdentifier.setResultsName('name') + op + Group(
            ZeroOrMore(remoteMethodDef)).setResultsName('methods') + cl + semicolon

        moduleContent = Group(structDef | enumDef | exceptionDef | dictionaryDef | sequenceDef | interfaceDef)
        module = Suppress(Word("module")) + identifier.setResultsName("name") + op + ZeroOrMore(
            moduleContent).setResultsName("contents") + cl + semicolon

        IDSL = idslImports.setResultsName("imports") + module.setResultsName("module")
        IDSL.ignore(cppStyleComment)
        return IDSL

    def string_to_struct(self, string, **kwargs):
        parsing_result = self.parse_string(string)
        result_dict = OrderedDict()

        # Hack to make robocompdsl work with pyparsing > 2.2
        try:
            result_dict['name'] = parsing_result['module']['name']
        except:
            result_dict['name'] = parsing_result['name']

        result_dict['imports'] = ''
        result_dict['recursive_imports'] = ''
        if 'imports' in parsing_result:
            # print result_dict['name'], parsing_result['imports']
            result_dict['imports'] = '#'.join(parsing_result['imports'])
            result_dict['recursive_imports'] = '#'.join(generate_recursive_imports(list(parsing_result['imports'])))
        # INTERFACES DEFINED IN THE MODULE
        result_dict['interfaces'] = []

        # Hack to make robocompdsl work with pyparsing > 2.2
        try:
            contents = parsing_result['module']['contents']
        except:
            contents = parsing_result['contents']

        for contentDef in contents:
            if contentDef[0] == 'interface':
                interface = {'name': contentDef[1], 'methods': OrderedDict()}
                for method in sorted(contentDef['methods'], key=attrgetter('name')):
                    interface['methods'][method['name']] = {}

                    interface['methods'][method['name']]['name'] = method['name']
                    try:
                        interface['methods'][method['name']]['decorator'] = method['decorator']
                    except:
                        interface['methods'][method['name']]['decorator'] = ''

                    interface['methods'][method['name']]['return'] = method['ret']

                    params = []
                    try:
                        for p in method['params']:
                            try:
                                params.append({'decorator': p['decorator'], 'type': p['type'], 'name': p['name']})
                            except:
                                params.append({'decorator': 'none', 'type': p['type'], 'name': p['name']})
                    except:
                        pass
                    interface['methods'][method['name']]['params'] = params

                    try:
                        interface['methods'][method['name']]['throws'] = method['raise'].asList()
                    except:
                        interface['methods'][method['name']]['throws'] = 'nothing'
                result_dict['interfaces'].append(interface)
        # TYPES DEFINED IN THE MODULE
        result_dict['types'] = []
        # print '---\n---\nPARSE IDSL TYPES'
        for contentDef in contents:
            # print contentDef[0]
            if contentDef[0] in ['enum', 'struct', 'exception']:
                # typedef = {'name': contentDef[1], 'type': contentDef[0]}
                typedef = contentDef.asDict()

                # print typedef
                result_dict['types'].append(typedef)
            elif contentDef[0] in ['sequence', 'dictionary']:
                typedef = contentDef.asDict()
                # print typedef
                result_dict['types'].append(typedef)
            elif contentDef[0] in ['interface']:
                pass
            else:
                print(('Unknown module content', contentDef))
        # SEQUENCES DEFINED IN THE MODULE
        result_dict['sequences'] = []
        result_dict['simpleSequences'] = []
        for contentDef in contents:
            if contentDef['type'] == 'sequence':
                seqdef = {'name': result_dict['name'] + "/" + contentDef['name'], 'type': contentDef['type'], 'typeSequence': contentDef['typeSequence']}
                simpleSeqdef = {'name': result_dict['name'], 'strName': contentDef['name']}
                # print structdef
                result_dict['sequences'].append(seqdef)
                result_dict['simpleSequences'].append(simpleSeqdef)
        # STRUCTS DEFINED IN THE MODULE
        result_dict['structs'] = []
        result_dict['simpleStructs'] = []
        for contentDef in contents:
            if contentDef['type'] == 'struct':
                structdef = {'name': result_dict['name'] + "/" + contentDef['name'], 'type': contentDef['type'], 'structIdentifiers':contentDef['structIdentifiers'].asList()}
                simpleStructdef = {'name': result_dict['name'], 'strName': contentDef['name']}
                # print structdef
                result_dict['structs'].append(structdef)
                result_dict['simpleStructs'].append(simpleStructdef)

        self.struct = result_dict
        return result_dict
