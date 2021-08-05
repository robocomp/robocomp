import ply.lex as lex

class CDSLLexer(object):
    # List of token names.   This is always required
    tokens = ("OBRACE",
              "CBRACE",
              "SEMI",
              "COMMA",
              "OPAR",
              "CPAR",
              "QUOTE",
              "IMPORT",
              "COMMUNICATIONS",
              "LANGUAGE",
              "COMPONENT",
              "PYTHON",
              "CPP",
              'CPP11',
              "GUI",
              "QWIDGET",
              "QMAINWINDOW",
              "QDIALOG",
              "QT",
              "REQUIRES",
              "IMPLEMENTS",
              "SUBSCRIBESTO",
              "PUBLISHES",
              "OPTIONS",
              "INNERMODELVIEWER",
              "STATEMACHINE",
              "VISUAL",
              "AGMAGENT",
              "AGM2AGENT",
              "AGM2AGENTROS",
              "AGM2AGENTICE",
              "ICE",
              "ROS",
              "IDENTIFIER",
              "PATH")


    # Regular expression rules for simple tokens
    t_OBRACE = r'\{'
    t_CBRACE = r'\}'
    t_SEMI = r';'
    t_COMMA= r','
    t_OPAR = r'\('
    t_CPAR = r'\)'
    t_QUOTE = r'\"'
    t_IDENTIFIER = r'(?i)[A-Za-z0-9\_]+'

    # A string containing ignored characters (spaces and tabs)
    t_ignore = ' \t'

    def __init__(self):
        # Build the lexer
        self.lexer = lex.lex(module=self)

    # Moved here because a priority problem making this to be interpreted as t_IDENTIFIER
    # https://stackoverflow.com/questions/2910338/python-yacc-lexer-token-priority
    def t_PATH(self, t):
        r'(?i)[a-zA-Z0-9]+\.[a-zA-Z0-9]+'
        return t

    def t_IMPORT(self, t):
        r'(?i)IMPORT'
        return t

    def t_PYTHON(self, t):
        r'(?i)PYTHON'
        return t

    def t_CPP11(self, t):
        r'(?i)CPP11'
        return t

    def t_CPP(self, t):
        r'(?i)CPP'
        return t

    def t_GUI(self, t):
        r'(?i)GUI'
        return t

    def t_QWIDGET(self, t):
        r'(?i)QWIDGET'
        return t

    def t_QMAINWINDOW(self, t):
        r'(?i)QMAINWINDOW'
        return t

    def t_QDIALOG(self, t):
        r'(?i)QDIALOG'
        return t

    def t_QT(self, t):
        r'(?i)QT'
        return t

    def t_REQUIRES(self, t):
        r'(?i)REQUIRES'
        return t

    def t_IMPLEMENTS(self, t):
        r'(?i)IMPLEMENTS'
        return t

    def t_SUBSCRIBESTO(self, t):
        r'(?i)SUBSCRIBESTO'
        return t

    def t_PUBLISHES(self, t):
        r'(?i)PUBLISHES'
        return t

    def t_OPTIONS(self, t):
        r'(?i)OPTIONS'
        return t

    def t_INNERMODELVIEWER(self, t):
        r'(?i)INNERMODELVIEWER'
        return t

    def t_STATEMACHINE(self, t):
        r'(?i)STATEMACHINE'
        return t

    def t_VISUAL(self, t):
        r'(?i)VISUAL'
        return t

    def t_AGM2AGENTICE(self, t):
        r'(?i)AGM2AGENTICE'
        return t

    def t_AGM2AGENTROS(self, t):
        r'(?i)AGM2AGENTROS'
        return t

    def t_AGM2AGENT(self, t):
        r'(?i)AGM2AGENT'
        return t

    def t_AGMAGENT(self, t):
        r'(?i)AGMAGENT'
        return t

    def t_ICE(self, t):
        r'(?i)ICE'
        return t

    def t_ROS(self, t):
        r'(?i)ROS'
        return t

    def t_LANGUAGE(self, t):
        r'(?i)LANGUAGE'
        return t

    def t_COMMUNICATIONS(self, t):
        r'(?i)COMMUNICATIONS'
        return t

    def t_COMPONENT(self, t):
        r'(?i)COMPONENT'
        return t


    # Define a rule so we can track line numbers
    def t_newline(self, t):
        r'\n+'
        t.lexer.lineno += len(t.value)

    # Error handling rule
    def t_error(self, t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    t_ignore_COMMENT = r'//.*'




# cdsl_lexer = CDSLLexer()
#
# # Give the lexer some input
# data = '''
# 	language python;
# '''
# cdsl_lexer.lexer.input(data)
#
# # Tokenize
# while True:
#     tok = cdsl_lexer.lexer.token()
#     if not tok:
#         break  # No more input
#     print(tok.type, tok.value, tok.lineno, tok.lexpos)

