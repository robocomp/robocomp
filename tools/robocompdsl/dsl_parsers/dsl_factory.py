import errno
import os
import sys
import traceback
from os import path
from pprint import pprint

from termcolor import cprint

from dsl_parsers.specific_parsers.cdsl_parser import CDSLParser
from dsl_parsers.specific_parsers.idsl_parser import IDSLParser
from dsl_parsers.specific_parsers.smdsl_parser import SMDSLParser

class Singleton(object):
    """
    Singleton implementation class with a cache dict
    """
    __instance = None

    def __new__(classtype, *args, **kwargs):
        if classtype != type(classtype.__instance):
            classtype.__instance = object.__new__(classtype, *args, **kwargs)
            classtype.__instance._cache = {}

        return classtype.__instance


class DSLFactory(Singleton):
    """
    This class create a cache of parsers for different dsl files. This is a singleton class. A single instance of
    this is shared no matter how many times it's "created". Given a file the method "from_file" generate and return (
    and store in it's cache) the structure representing the dsl file. If from_file method is called again to generate
    and return the same file this will be obtained from the cache unless the "update" parameter is passed to this
    method.
    """
    def __init__(self):
        super(DSLFactory, self).__init__()

    def from_string(self, string, dsl_type, **kwargs):
        """
        Return a struct/dict representing constructed from a string representing a dsl with the type dsl_type
        :param string: string containing the dsl with a syntax of the dsl_type
        :param dsl_type: type of the dsl string to be parsed
        :return: struct/dict containing the information of the dsl contained in the string. This also result the created parser
        """
        # get the parser for the dsl type
        parser = self.create_parser(dsl_type)
        # get the result as a dict from the string
        result = parser.string_to_struct(string, **kwargs)
        return result, parser

    def from_file(self, file_path, update=False, **kwargs):
        """
        Return a struct/dict representing constructed from a file representing a dsl with the type dsl_type
        By default the result of parsing a file is cached.
        :param file_path: path to the file containing the dsl. Format is extracted from the file extension.
        :param update: force the update of any cached file
        :return: struct/dict containing the information of the dsl contained in the file
        """
        if file_path is None:
            return None
        if not os.path.isfile(file_path):
            # local import to avoid problem with mutual imports
            from dsl_parsers.parsing_utils import idsl_robocomp_path
            new_file_path = idsl_robocomp_path(file_path)
            if new_file_path is None or not os.path.isfile(new_file_path):
                print("DSLFactory. %s could not be found in Robocomp"%file_path)
                raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), file_path)
            else:
                file_path = new_file_path
        else:
            file_path = os.path.abspath(file_path)
        # if update is false and file_path exists in the cache, it's returned
        if file_path in self. _cache and update is False:
            # print("______________________Cached %s______________" % file_path)
            result = self._cache[file_path].struct
        else:
            # print("______________________Parsing %s______________" % file_path)
            if file_path is None or file_path == "":
                # TODO: Raise Exception
                return None

            # get format from filename
            dsl_type = path.splitext(file_path)[1][1:]

            # get string from file
            try:
                with open(file_path, 'r') as reader:
                    string = reader.read()
            except:
                print("DSLFactory: Error reading input file %s"%file_path)
                # TODO: Raise Exception
                return None

            # get the result from string
            try:
                result, parser = self.from_string(string, dsl_type, **kwargs)
            except:
                cprint('Error parsing %s' % file_path, 'red')
                traceback.print_exc()
                # TODO: remove Exit. Add Exception.
                sys.exit(-1)
            else:
                result['filename'] = file_path
                # store the parser with the result in the cache fo the factory
                self._cache[file_path] = parser
        return result

    def create_parser(self, dsl_type):
        """
        Return the corresponding parser for a specific dsl type.
        :param dsl_type: type of the dsl to get the parser for
        :return:
        """
        if dsl_type.lower() == 'cdsl':
            return CDSLParser()
        elif dsl_type.lower() == 'smdsl':
            return SMDSLParser()
        elif dsl_type.lower() == 'idsl':
            return IDSLParser()
        else:
            raise ValueError("Invalid dsl type '%s'. No valid parser found for it."%dsl_type)


if __name__ == '__main__':
    factory = DSLFactory()
    for idsl_path in os.listdir("/opt/robocomp/interfaces/IDSLs"):
        if idsl_path.endswith(".idsl"):
            idsl = factory.from_file(idsl_path)
            from pprint import pprint
            print("#ifndef " + idsl['name'].upper() + "_ICE")
            print("#define " + idsl['name'].upper() + "_ICE")
            if 'imports' in idsl and idsl["imports"] != '':
                for imp in idsl['imports'].split('#'):
                    if imp != '':
                        print("#include <" + os.path.basename(imp).split('.')[0] + ".ice>")
            print("module " + idsl['name'] + "\n{")
            if 'types' in idsl:
                for next_type in idsl["types"]:
                    if "exception" == next_type["type"]:
                        print("<TABHERE>exception " + next_type['name'] + "{" + next_type['content'] + "};")
                    if "struct" == next_type["type"]:
                        struct= next_type
                        print("<TABHERE>struct " + struct['name'] + "\n<TABHERE>{")
                        for var in struct['structIdentifiers']:
                            print("<TABHERE><TABHERE> " + var['type'] + " " + var['identifier']),
                            try:
                                print(" =" + var['defaultValue'] + ";")
                            except:
                                print(";")
                        print("<TABHERE>};")
                    if "sequence" == next_type["type"]:
                        print("<TABHERE>sequence <" + next_type['typeSequence'] + "> " + next_type['name'] + ";")
                    if "dictionary" == next_type['type']:
                        print("<TABHERE>dictionary <" + next_type['content'] + "> " + next_type['name'] + ";")
                    if "enum" == next_type['type']:
                        print("<TABHERE>enum " + next_type['name'] + " { " + next_type['content'] + " };")
            if "interfaces" in idsl:
                for interface in idsl['interfaces']:
                    print("<TABHERE>interface " + interface['name'] + "\n<TABHERE>{")
                    for method in interface['methods'].values():
                        if method['decorator'] != '':
                            print("<TABHERE><TABHERE>" + method['decorator'] + " " + method['return'] + " "),
                        else:
                            print("<TABHERE><TABHERE>" + method['return'] + " "),
                        print(method['name'] + " ("),
                        try:
                            paramStrA = ''
                            for p in method['params']:
                                # delim
                                if paramStrA == '':
                                    delim = ''
                                else:
                                    delim = ', '
                                # STR
                                if p['decorator'] != "none" and p['decorator'] != '':
                                    paramStrA += delim + p['decorator'] + ' ' + p['type'] + ' ' + p['name']
                                else:
                                    paramStrA += delim + p['type'] + ' ' + p['name']
                            print(paramStrA + ")"),
                        except:
                            print(")"),
                        try:
                            if method['throws'] != "nothing":
                                print(" throws "),
                                for p in method['throws']:
                                    # STR
                                    print(p),
                        except:
                            pass
                        print(";")
                    print("<TABHERE>};")





