import abc
import os


class DSLParserTemplate:
    """
    Abstract class to be inherited by other dsl parsers.
    This class have the needed attributes and methods to parse and store the information related to a dsl file or string
    There's also some abstract methods that must be reimplemented on the inherited class.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.__file = None
        self.__string = None
        self.__struct = None
        self.__parser = None
        self.__pyparsing_result = None

    @property
    def parser(self):
        if self.__parser is None:
            self.__parser = self._create_parser()
        return self.__parser


    @parser.setter
    def parser(self, value):
        pass


    @property
    def string(self):
        return self.__string

    @string.setter
    def string(self, value):
        self.__string = value

    @property
    def file(self):
        return self.__file

    @file.setter
    def file(self, file_path):
        assert file_path is not None, "Can't set None for dsl_parser file"
        assert file_path != "", "Can't set empty strin \"\" for dsl_parser file"
        assert os.path.isfile(file_path), "%s must be an existing file to be set in dsl_parser"
        self.__file = file_path

    @property
    def struct(self):
        return self.__struct

    @struct.setter
    def struct(self, value):
        self.__struct = value

    @property
    def pyparsing_result(self):
        return self.__pyparsing_result

    @pyparsing_result.setter
    def pyparsing_result(self, value):
        pass

    def parse_string(self, string):
        if self.string is None:
            self.string = string
        self.__pyparsing_result = self.parser.parseString(string)
        return self.pyparsing_result

    @abc.abstractmethod
    def _create_parser(self):
        pass

    @abc.abstractmethod
    def string_to_struct(self, string, **kwargs):
        pass
