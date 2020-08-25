import errno
import os
import traceback
from os import path
import pyparsing
from termcolor import cprint

from dsl_parsers.specific_parsers.cdsl.jcdsl_parser import CDSLJsonParser
from dsl_parsers.specific_parsers.cdsl.cdsl_parser import CDSLParser
# from dsl_parsers.specific_parsers.cdsl.cdsl_ply_parser import CDSLParser
from dsl_parsers.specific_parsers.idsl_parser import IDSLParser
from dsl_parsers.specific_parsers.smdsl_parser import SMDSLParser


class Singleton(object):
    """
    Singleton implementation class with a cache dict
    """
    __instance = None

    def __new__(class_, *args, **kwargs):
        if not isinstance(class_.__instance, class_):
            class_.__instance = object.__new__(class_, *args, **kwargs)
            class_.__instance._cache = {}
        return class_.__instance


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
                print("DSLFactory. %s could not be found in Robocomp" % file_path)
                raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), file_path)
            else:
                file_path = new_file_path
        else:
            file_path = os.path.abspath(file_path)
        # if update is false and file_path exists in the cache, it's returned
        if file_path in self._cache and update is False:
            # print("______________________Cached %s______________" % file_path)
            result = self._cache[file_path]
        else:
            # print("______________________Parsing %s______________" % file_path)
            if file_path is None or file_path == "":
                raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), file_path)

            # get format from filename
            dsl_type = path.splitext(file_path)[1][1:]

            # get string from file
            try:
                with open(file_path, 'r') as reader:
                    string = reader.read()
            except Exception:
                print("DSLFactory: Error reading input file %s" % file_path)
                raise

            # get the result from string
            try:
                result, parser = self.from_string(string, dsl_type, **kwargs)
            except (pyparsing.ParseException, ValueError) as e:
                e.filepath = file_path
                raise

            else:
                result['filename'] = file_path
                # store the parser with the result in the cache fo the factory
                self._cache[file_path] = result
        return result

    @staticmethod
    def create_parser(dsl_type):
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
        elif dsl_type.lower() == 'jcdsl':
            return CDSLJsonParser()
        else:
            raise ValueError("Invalid dsl type '%s'. No valid parser found for it." % dsl_type)
