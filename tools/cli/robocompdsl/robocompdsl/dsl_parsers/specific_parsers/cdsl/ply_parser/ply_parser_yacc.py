import ply.yacc as yacc
from robocompdsl.dsl_parsers.specific_parsers.cdsl.ply_parser.ply_parser_lex import CDSLLexer

class CCDSLPlyParser(object):
    tokens = CDSLLexer.tokens
    def p_component(self, p):
        '''component : importslist COMPONENT IDENTIFIER OBRACE content CBRACE SEMI'''
        p[0] = {"name": p[3], "imports": p[1]}
        p[0].update(p[5])

    def p_importslist(self, p):
        '''
        importslist  : idslimport importslist
                     |
        '''
        if len(p) > 2:
            p[0] = p[2]
            p[0].append(p[1])
        elif len(p) > 1:
            p[0] = [p[1]]
        else:
            p[0] = []

    def p_idslimport(self, p):
        '''
        idslimport  : IMPORT QUOTE PATH QUOTE SEMI
        '''
        if len(p)>1:
            p[0] = p[3]
        else:
            p[0] = []


    def p_content(self, p):
        '''content  : commblock language otherslist'''
        p[0] = {'language': p[2]}
        p[0].update(p[1])
        if p[3] is not None:
            p[0].update(p[3])

    def p_language(self, p):
        '''language	: LANGUAGE langopts SEMI'''
        p[0] = p[2].lower()

    def p_langopts(self, p):
        '''
        langopts    : CPP
                    | CPP11
                    | PYTHON
        '''
        p[0] = p[1]

    def p_otherslist(self, p):
        '''
        otherslist  : otherslist others
                    |
        '''
        # if there's others statements repeated, the last one will be taken
        if len(p) > 2:
            p[0] = p[1]
            p[0].update(p[2])
        else:
            p[0] = {}


    def p_others(self, p):
        '''
        others  : gui
                | statemachine
                | options
        '''
        p[0] = p[1]

    def p_gui(self, p):
        '''
        gui		: GUI QT OPAR guiopts CPAR SEMI
        '''
        if len(p)> 1:
            p[0] = {'gui': [p[2], p[4]]}
        else:
            p[0] = {'gui': None}

    def p_guiopts(self, p):
        '''
        guiopts	: QWIDGET
                | QMAINWINDOW
                | QDIALOG
        '''
        p[0] = p[1]

    def p_options(self, p):
        '''
        options : OPTIONS optionsoptlist SEMI
        '''
        if len(p)>1:
            p[0] = p[2]
        else:
            p[0] = []

    def p_optionsoptlist(self, p):
        '''
        optionsoptlist  : optionsoptlist COMMA optionsopt
                        | optionsopt
        '''
        if len(p) > 2:
            p[0] = p[1]
            p[0]['options'].append(p[3])
        elif len(p) > 1:
            p[0] = {'options': [p[1]]}
        else:
            p[0] = {'options': []}

    def p_optionsopt(self, p):
        '''
        optionsopt	: INNERMODELVIEWER
                    | AGMAGENT
                    | AGM2AGENT
                    | AGM2AGENTICE
                    | AGM2AGENTROS
        '''
        p[0] = p[1].lower()

    def p_statemachine(self, p):
        '''
        statemachine    : STATEMACHINE QUOTE PATH QUOTE stmvisual SEMI
        '''
        if len(p) > 1:
            p[0] = {'statemachine': [p[3], p[5]]}
        else:
            p[0] = {'statemachine': None}

    def p_stmvisual(self, p):
        '''
        stmvisual   : VISUAL
                    |
        '''
        if len(p) == 2:
            p[0] = True
        else:
            p[0] = False


    def p_commblock(self, p):
        '''commblock	: COMMUNICATIONS OBRACE commlist CBRACE SEMI'''
        p[0] = p[3]

    def p_commlist(self, p):
        '''commlist : commlist comm
                    | '''
        if len(p) > 2:
            p[0] = p[1]
            # in case of repetitions of the same commkeywords
            if all(key in p[0] for key in p[2].keys()):
                for key in p[2].keys():
                    p[0][key].extend(p[2][key])
            else:
                p[0].update(p[2])
        else:
            p[0] = {}


    def p_comm(self, p):
        '''comm : commkeywords commalist SEMI'''
        if len(p)> 2:
            p[0] = {p[1].lower():p[2]}

    def p_commkeywords(self, p):
        '''commkeywords : IMPLEMENTS
                        | REQUIRES
                        | SUBSCRIBESTO
                        | PUBLISHES'''
        p[0] = p[1]

    def p_commalist(self, p):
        '''commalist    : commalist COMMA commitem
                        | commitem'''
        if len(p) > 2:
            p[0] = p[1]
            p[0].append(p[3])
        else:
            p[0] = [p[1]]


    def p_commitem(self, p):
        '''
        commitem    : IDENTIFIER commtype
                    | IDENTIFIER
        '''

        if len(p) > 2:
            p[0] = [p[1], p[2]]
        else:
            p[0] = [p[1], 'ice']


    def p_commtype(self, p):
        '''commtype : OPAR ICE CPAR
                    | OPAR ROS CPAR
        '''
        p[0] = p[2]


    def p_error(self, p):
      print("Syntax error at %s"%p.value)

    # def p_empty(p):
    #     '''empty : '''
    #     pass

    def parseString(self, string):
        self.lexer = CDSLLexer()
        bparser = yacc.yacc(module=self)
        bparser.error = 0
        return bparser.parse(string)


# ###  TESTING CODE
#
# data = '''
# import "blabblaba1.idsl";
# import "blabblaba2.idsl";
# import "blabblaba3.idsl";
# component lamacusa
# {
#     communications
#     {
#         implements nube(ros), manzana(ice), pera;
#         requires nube2(ros), manzana2(ice), pera2;
#     };
#     language cpp11;
#     options innermodelviewer;
#     gui qt (qwidget);
#     statemachine "casalama.smdsl" visual;
# };
# '''
#
# parser = CDSLParser()
# p = parser.parse_string(data)
# pprint(p)

