from functools import reduce


class ComponentFacade(dict):
    """
    The main purpose of this class is to be used as a facade for
    the Component used by the templates and other parts of robocompdsl code.
    This is useful to avoid the need of making changes in all the accesses to
    the component attributes when something changes in the dict representation
    of the cdsl returned by the parser (AST).

    The class is created from the nested dict, calling the from_nested_dict method.
    All the keys (and nested keys) on the dict are then accessed as class attributes
    of the ComponentFacade instance.

    A Mapping function have also been implemented to be able to access keys renamed,
    restructured or moved because of a change in the parser.
    For example, imagine that you have this dict representing a component:
    component = {
        'name': 'uno',
        'language': 'cpp'
    }
    After using the ComponentFacade class you would access to the language attribute like this:
    component.language and you would get the 'cpp' string values.

    Then you change your mind and you decide to add modules options depending on the language and
    it is represented in the component dict like this:
    component = {
        'name': 'uno',
        'language': {
            'name': 'cpp',
            'modules': ['boost', 'opencv']
        }
    }

    The problem is that now every place that previously accessed the language of the component with
    component.language, now are getting a dict, not the expected string.
    But in this case you could use the method
            mapping = {'language': ['language', 'name']}
            component.set_mapping(mapping)
    And after installing this map when you call component.language the ComponentFacade class
    is internally looking for this in the internal dict as ['language']['name']
    """
    def __init__(self, *args, **kwargs):
        super(ComponentFacade, self).__init__(*args, **kwargs)
        self._mapping = {}

    @staticmethod
    def from_nested_dict(data):
        if not isinstance(data, dict):
            return data
        else:
            return ComponentFacade({key: ComponentFacade.from_nested_dict(data[key]) for key in data})

    def __getitem__(self, item):
        raise ValueError()

    def __setitem__(self, item, value):
        if item == 'filename':
            super(ComponentFacade, self).__setitem__(item, value)
        else:
            raise ValueError()

    def __setattr__(self, key, value):
        if key != '_mapping':
            super(ComponentFacade, self).__setitem__(key, value)
        else:
            super(ComponentFacade, self).__setattr__(key, value)

    def __getattr__(self, attr):
        return super(ComponentFacade, self).__getitem__(attr)

    @property
    def name(self):
        return super(ComponentFacade, self).__getitem__('name')

    def set_mapping(self, mapping):
        self._mapping = mapping

    def get_property_with_mapping(self, property):
        if property in self._mapping:
            return reduce(dict.get, self._mapping[property], self)
        else:
            return super(ComponentFacade, self).__getitem__(property)

    @property
    def language(self):
        return self.get_property_with_mapping('language')

    @property
    def ice_interfaces_names(self):
        names = []
        for item in self.get_property_with_mapping('iceInterfaces'):
            if isinstance(item, list):
                names.append(item[0])
            else:
                names.append(item)
        return names
