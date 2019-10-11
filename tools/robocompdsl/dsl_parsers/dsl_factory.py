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
    This class create an cache parsers for different dsl files.
    This is a singleton class. A single instance of this is shared no matter how many it's "created",

    """
    def __init__(self):
        super(DSLFactory, self).__init__()
        

    def from_string(self, string, dsl_type):
        """
        Return a struct/dict representing constructed from a string representing a dsl with the type dsl_type
        :param string: string containing the dsl with a syntax of the dsl_type
        :param dsl_type: type of the dsl string to be parsed
        :return: struct/dict containing the information of the dsl contained in the string. This also result the created parser
        """
        # get the parser for the dsl type
        parser = self.create_parser(dsl_type)
        # get the result as a dict from the string
        result = parser.string_to_struct(string)
        return result, parser

    def from_file(self, file_path, update=False):
        """
        Return a struct/dict representing constructed from a file representing a dsl with the type dsl_type
        By default the result of parsing a file is cached.
        :param file_path: path to the file containing the dsl. Format is extracted from the file extension.
        :param update: force the update of any cached file
        :return: struct/dict containing the information of the dsl contained in the file
        """
        # if update is false and file_path exists in the cache, it's returned
        if file_path in self. _cache and update is False:
            result = self._cache[file_path].struct
        else:
            if file_path is None or file_path == "":
                # TODO: Raise Exception
                return None

            # get format from filename
            format = path.splitext(file_path)[1][1:]


            # get string from file
            try:
                with open(file_path, 'r') as reader:
                    string = reader.read()
            except:
                print("Error reading input file for SMDSL")
                # TODO: Raise Exception
                return None

            # get the result from string
            try:
                result, parser = self.from_string(string, format)
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


if __name__ == '__main__':
    factory = DSLFactory()
    a = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/gamestatemachine.smdsl")
    pprint(dict(a))
    factory2 = DSLFactory()
    b = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/gamestatemachine.smdsl")
    factory3 = DSLFactory()
    c = factory.from_file("/home/robolab/robocomp/interfaces/IDSLs/JointMotor.idsl")
    factory4 = DSLFactory()
    d = factory.from_file("/home/robolab/robocomp/interfaces/IDSLs/JointMotor.idsl")
    factory5 = DSLFactory()
    e = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/tvgames.cdsl")
    pprint(dict(a))
    factory6 = DSLFactory()
    f = factory.from_file("/home/robolab/robocomp/components/euroage-tv/components/tvGames/tvgames.cdsl",update=True)
    print((b==a) and (c==d) and (e==f))
