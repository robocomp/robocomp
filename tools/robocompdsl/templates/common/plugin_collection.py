import inspect
import os
import pkgutil
import importlib

from templates.common.templatedict import TemplateDict


class Plugin(object):
    """Base class that each plugin must inherit from. within this class
    you must define the methods that all of your plugins must implement
    """

    def __init__(self):
        self.classes = {}
        self.description = 'UNKNOWN'

    def load_functions(self):
        for root, dirs, files in os.walk(self.abs_path + "/functions"):
            for file in files:
                relative_path = root.replace(self.abs_path + "/functions", '')
                if relative_path and relative_path[0] == os.sep: relative_path = relative_path[1:]
                relative_path = os.path.join(relative_path, file)
                if relative_path not in self.classes:
                    try:
                        module = importlib.machinery.SourceFileLoader(file.split('/')[-1].split('.')[0],
                                                                      os.path.join(root, file)).load_module()
                    except AssertionError as e:
                        pass
                    except ValueError as e:
                        pass
                    for _, the_class in inspect.getmembers(module, lambda x: inspect.isclass(x) and issubclass(x, TemplateDict) and x.__name__!="TemplateDict"):
                        print(f"Loading plugin: {file}")
                        self.classes[relative_path] = the_class

    def get_template_dict(self, file, ast, interface_name=None):
        file = file.replace(".", "_")+".py"
        if file in self.classes:
            if interface_name:
                return self.classes[file](ast, interface_name)
            else:
                return self.classes[file](ast)
        else:
            return {}




class PluginCollection(list):
    """Upon creation, this class will read the plugins package for modules
    that contain a class definition that is inheriting from the Plugin class
    """

    def __init__(self, plugin_package):
        """Constructor that initiates the reading of all available plugins
        when an instance of the PluginCollection object is created
        """
        super(PluginCollection, self).__init__()
        self.plugin_package = plugin_package
        self.reload_plugins()


    def reload_plugins(self):
        """Reset the list of all plugins and initiate the walk over the main
        provided plugin package to load all available plugins
        """
        self.seen_paths = []
        print()
        print(f'Looking for plugins under package {self.plugin_package}')
        self.walk_package(self.plugin_package)


    def apply_all_plugins_on_value(self, argument):
        """Apply all of the plugins on the argument supplied to this function
        """
        print()
        print(f'Applying all plugins on value {argument}:')
        for plugin in self:
            print(f'    Applying {plugin.description} on value {argument} yields value {plugin.perform_operation(argument)}')

    def walk_package(self, package):
        """Recursively walk the supplied package to retrieve all plugins
        """
        imported_package = __import__(package, fromlist=['blah'])
        for _, pluginname, ispkg in pkgutil.iter_modules(imported_package.__path__, imported_package.__name__ + '.'):
            if not ispkg:
                plugin_module = __import__(pluginname, fromlist=['blah'])
                clsmembers = inspect.getmembers(plugin_module, inspect.isclass)
                for (_, c) in clsmembers:
                    # Only add classes that are a sub class of Plugin, but NOT Plugin itself
                    if issubclass(c, Plugin) & (c is not Plugin):
                        print(f'    Found plugin class: {c.__module__}.{c.__name__}')
                        self.append(c())

            # Now that we have looked at all the modules in the current package, start looking
            # recursively for additional modules in sub packages
        all_current_paths = []
        if isinstance(imported_package.__path__, str):
            all_current_paths.append(imported_package.__path__)
        else:
            all_current_paths.extend([x for x in imported_package.__path__])

        for pkg_path in all_current_paths:
            if pkg_path not in self.seen_paths:
                self.seen_paths.append(pkg_path)

                # Get all sub directory of the current package path directory
                child_pkgs = [p for p in os.listdir(pkg_path) if os.path.isdir(os.path.join(pkg_path, p))]

                # For each sub directory, apply the walk_package method recursively
                for child_pkg in child_pkgs:
                    self.walk_package(package + '.' + child_pkg)