import rcExceptions
from dsl_parsers.parsing_utils import IDSLPool


# class ComponentFacade(dict):
#     """
#     The main purpose of this class is to be used as a facade for
#     the Component used by the templates and other parts of robocompdsl code.
#     This is useful to avoid the need of making changes in all the accesses to
#     the component attributes when something changes in the dict representation
#     of the cdsl returned by the parser (AST).
#
#     The class is created from the nested dict, calling the from_nested_dict method.
#     All the keys (and nested keys) on the dict are then accessed as class attributes
#     of the ComponentFacade instance.
#
#     A Mapping function have also been implemented to be able to access keys renamed,
#     restructured or moved because of a change in the parser.
#     For example, imagine that you have this dict representing a component:
#     component = {
#         'name': 'uno',
#         'language': 'cpp'
#     }
#     After using the ComponentFacade class you would access to the language attribute like this:
#     component.language and you would get the 'cpp' string values.
#
#     Then you change your mind and you decide to add modules options depending on the language and
#     it is represented in the component dict like this:
#     component = {
#         'name': 'uno',
#         'language': {
#             'name': 'cpp',
#             'modules': ['boost', 'opencv']
#         }
#     }
#
#     The problem is that now every place that previously accessed the language of the component with
#     component.language, now are getting a dict, not the expected string.
#     But in this case you could use the method
#             mapping = {'language': ['language', 'name']}
#             component.set_mapping(mapping)
#     And after installing this map when you call component.language the ComponentFacade class
#     is internally looking for this in the internal dict as ['language']['name']
#     """
#     def __init__(self, *args, **kwargs):
#         super(ComponentFacade, self).__init__(*args, **kwargs)
#         self._mapping = {}
#
#     @staticmethod
#     def from_nested_dict(data):
#         if not isinstance(data, dict):
#             return data
#         else:
#             return ComponentFacade({key: ComponentFacade.from_nested_dict(data[key]) for key in data})
#
#     def __getitem__(self, item):
#         raise ValueError()
#
#     def __setitem__(self, item, value):
#         if item == 'filename':
#             super(ComponentFacade, self).__setitem__(item, value)
#         else:
#             raise ValueError()
#
#     def __setattr__(self, key, value):
#         if key != '_mapping':
#             super(ComponentFacade, self).__setitem__(key, value)
#         else:
#             super(ComponentFacade, self).__setattr__(key, value)
#
#     def __getattr__(self, attr):
#         if attr.endswith('_name') or attr.endswith('_names'):
#             result = super(ComponentFacade, self).__getitem__(attr)
#             while not isinstance(result, str):
#                 result = result[0]
#             return result
#         else:
#             return super(ComponentFacade, self).__getitem__(attr)
#
#     @property
#     def name(self):
#         return super(ComponentFacade, self).__getitem__('name')
#
#     def set_mapping(self, mapping):
#         self._mapping = mapping
#
#     def get_property_with_mapping(self, property):
#         if property in self._mapping:
#             return reduce(dict.get, self._mapping[property], self)
#         else:
#             return super(ComponentFacade, self).__getitem__(property)
#
#     @property
#     def language(self):
#         return self.get_property_with_mapping('language')
#
#     @property
#     def ice_interfaces_names(self):
#         names = []
#         for item in self.get_property_with_mapping('iceInterfaces'):
#             if isinstance(item, list):
#                 names.append(item[0])
#             else:
#                 names.append(item)
#         return names
#
#     @property
#     def idsl_pool(self):
#         if 'idsl_pool' in self:
#             return self.idsl_pool
#         else:
#             the_idsls = ''.join([imp + '#' for imp in self.imports])
#             idsl_pool = IDSLPool(the_idsls, [])
#             self.idsl_pool = idsl_pool

class Interface(list):
    def __init__(self, params):
        super(Interface, self).__init__()
        self.extend(params)

    @property
    def name(self):
        return self[0]

    @property
    def type(self):
        return self[1]

    def __hash__(self):
        return tuple(self).__hash__()

    def __eq__(self, other):
        return tuple(self) == tuple(other)

class Interfaces(list):
    def __init__(self, params):
        super(Interfaces, self).__init__()
        the_list = [Interface(x) for x in params]
        self.extend(the_list)

    def append(self, item):
        super(Interfaces, self).append(Interface(item))

    def __hash__(self):
        return tuple(self).__hash__()

    def __eq__(self, other):
        return tuple(self) == tuple(other)

class Gui(list):
    def __init__(self, params):
        super(Gui, self).__init__()
        self.extend(params)

    @property
    def library(self):
        return self[0]

    @property
    def widget(self):
        return self[1]

class Options(list):
    def __init__(self, params):
        super(Options, self).__init__()
        self.extend(params)

    def __getattr__(self, item):
        return (item in [x.lower() for x in self])


CLASS_TYPE_MAP = {
    'implements': Interfaces,
    'requires': Interfaces,
    'subscribesTo': Interfaces,
    'publishes': Interfaces,
    'iceInterfaces': Interfaces,
    'rosInterfaces': Interfaces,
    'options': Options,
    'gui': Gui
}

class ComponentFacade:
    def __init__(self, nested_dict=None):
        if nested_dict is not None:
            for key, value in nested_dict.items():
                setattr(self, key, value)

    def __setattr__(self, key, value):
        if isinstance(value, (list, tuple, dict)):
            if key in CLASS_TYPE_MAP:
                new_class = CLASS_TYPE_MAP[key]
                value = new_class(value)
        super(ComponentFacade, self).__setattr__(key, value)

    @property
    def idsl_pool(self):
        if hasattr(self, '__idsl_pool'):
            return self.__idsl_pool
        else:
            idsl_pool = IDSLPool(self.imports, [])
            interface_list = self.requires + self.implements + self.subscribesTo + \
                             self.publishes
            for interface_required in interface_list:
                if not idsl_pool.module_providing_interface(interface_required.name):
                    raise rcExceptions.InterfaceNotFound(interface_required.name, idsl_pool.interfaces())
            self.__idsl_pool = idsl_pool
            return idsl_pool

    @property
    def statemachine(self):
        if not hasattr(self, '__statemachine'):
            from ...dsl_factory import DSLFactory
            self.__statemachine = DSLFactory().from_file(self.statemachine_path)
        return self.__statemachine

    def is_agm_agent(self):
        return self.options.agmagent

    @property
    def ice_interfaces_names(self):
        names = []
        for item in self.iceInterfaces:
            if isinstance(item, list):
                names.append(item[0])
            else:
                names.append(item)
        return names

    def __setitem__(self, key, value):
        """
        For compatibility with the smdsl and idsl object format.
        """
        if key == 'filename':
            setattr(self, key, value)
        else:
            raise TypeError('Invalid acces to ComponentFacade by item %s' % key)


    def __eq__(self, other):
        equal = True
        if not isinstance(other, ComponentFacade):
            return False
        for attr in self.__dict__:
            if hasattr(other, attr):
                equal = equal and getattr(self, attr) == getattr(other, attr)
            else:
                return False
        return equal
