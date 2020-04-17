from functools import reduce


class ComponentFacade(dict):
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

