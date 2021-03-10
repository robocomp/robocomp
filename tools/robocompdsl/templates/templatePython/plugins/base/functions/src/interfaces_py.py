from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from templates.common.templatedict import TemplateDict
from templates.templatePython.plugins.base.functions import function_utils as utils

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
"""

# TODO: Check if this can be reduced to an abstract class and some inheriting from that.
LIST_CLASSES_STR = """\
class ${list_type}(list):
    def __init__(self, iterable=list()):
        super(${list_type}, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, ${item_type})
        super(${list_type}, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).insert(index, item)

setattr(${module_name}, "${list_type}", ${list_type})
"""

SUBSCRIBESTO_STR = """
self.${iface_name} = self.create_adapter("${iface_name}Topic", ${iface_name_lower}I.${iface_name}I(default_handler))
"""

REQUIRE_STR = """
self.${iface_name}${num} = self.create_proxy("${iface_name}${num}Proxy", ${module_name}.${iface_name}Prx)
"""

PUBLISHES_STR = """
self.${iface_name_lower} = self.create_topic("${iface_name}", ${module_name}.${iface_name}Prx)
"""

IMPLEMENTS_STR = """\
self.${iface_name_lower} = self.create_adapter("${iface_name}", ${iface_name_lower}I.${iface_name}I(default_handler))
"""


class src_interfaces_py(TemplateDict):
    def __init__(self, component):
        super(src_interfaces_py, self).__init__()
        self.component = component
        self['load_slice_and_create_imports'] = self.load_slice_and_create_imports()
        self['create_lists_classes'] = self.create_lists_classes()
        self['implements_and_subscribes_imports'] = self.implements_and_subscribes_imports()
        self['require_proxy_creation'] = self.require_proxy_creation()
        self['publish_proxy_creation'] = self.publish_proxy_creation()
        self['implements_adapters_creation'] = self.implements_adapters_creation()
        self['subscribes_adapters_creation'] = self.subscribes_adapters_creation()

    # TODO: Check if can be merged with SERVANT_PY.py slice_loading function
    def load_slice_and_create_imports(self, includeDirectories=None):
        result = ""
        import os
        for imp in sorted(set(self.component.recursiveImports + self.component.imports)):
            file_name = os.path.basename(imp)
            name = os.path.splitext(file_name)[0]
            result += Template(SLICE_LOAD_STR).substitute(interface_name=name)
            module = DSLFactory().from_file(file_name, includeDirectories=includeDirectories)
            result += f"import {module['name']}\n"
        return result

    def create_lists_classes(self):
        result = ""
        for idsl in sorted(set(self.component.recursiveImports + self.component.imports)):
            try:
                module = self.component.idsl_pool.module_providing_interface(idsl.split('.')[0])
            except Exception as e:
                print(e.message)
                exit(-1)

            if module is not None:  # For modules without interface
                for sequence in module['sequences']:
                    item_type = utils.get_type_string(sequence['typeSequence'], module['name'])
                    if item_type == 'bytes': continue
                    result += Template(LIST_CLASSES_STR).substitute(list_type=sequence['name'].split('/')[1],
                                                                    item_type=item_type,
                                                                    module_name=module['name'])
        return result

    def implements_and_subscribes_imports(self):
        result = ""
        for im in self.component.implements + self.component.subscribesTo:
            if communication_is_ice(im):
                result += 'import ' + im.name.lower() + 'I\n'
        return result

    def require_proxy_creation(self):
        result = ""
        for iface, num in get_name_number(self.component.requires):
            if communication_is_ice(iface):
                name = iface.name
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                result += Template(REQUIRE_STR).substitute(iface_name=name, module_name=module['name'], iface_name_lower=name.lower(), num=num)
        return result

    def publish_proxy_creation(self):
        result = ""
        for iface, num in get_name_number(self.component.publishes):
            if communication_is_ice(iface):
                name = iface.name
                module = self.component.idsl_pool.module_providing_interface(iface.name)
                result += Template(PUBLISHES_STR).substitute(iface_name=name,
                                                             iface_name_lower=name.lower(),
                                                             module_name=module['name'])
        return result

    def implements_adapters_creation(self):
        result = ""
        for iface in self.component.implements:
            if communication_is_ice(iface):
                name = iface[0]
                result += Template(IMPLEMENTS_STR).substitute(iface_name=name, iface_name_lower=name.lower())
        return result

    def subscribes_adapters_creation(self):
        result = ""
        for sut in self.component.subscribesTo:
            if communication_is_ice(sut):
                name = sut[0]
                result += Template(SUBSCRIBESTO_STR).substitute(iface_name=name, iface_name_lower=name.lower())
        return result
