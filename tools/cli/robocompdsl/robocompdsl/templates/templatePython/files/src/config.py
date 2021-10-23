import yaml

KNOW_KEYS = ['ice', 'communications']


class ConfigManager:
    def __init__(self):
        self.config_dict = None
        self.__custom_properties = None

    def from_file_path(self, filepath):
        """ Create a config manager from a config filepath"""

        assert filepath.endswith(".yml") or filepath.endswith(".yaml"),\
            "Config file must be a Yaml file with [.yml|.yaml] extension "
        assert self.config_dict is None,\
            "Config file should only be read once per execution. Tried to read a second time."

        with open(filepath, "r") as config_file:
            try:
                self.config_dict = yaml.safe_load(config_file)
                self.__custom_properties = \
                    {key: value for key, value in self.config_dict.items() if key not in KNOW_KEYS}
            except yaml.YAMLError as exc:
                print(exc)

    # def from_dict(self, filepath):
    #     """ Create a config manager from a config filepath"""

    @property
    def custom_properties(self):
        """Returns custom properties for a config"""
        return self.__custom_properties

    @property
    def parameters(self):
        """Convenience name for custom_properties"""
        return self.custom_properties

    def ice_properties(self):
        """Return ice properties as a dict"""
        return self.config_dict['ice']

    @property
    def ice_properties_args(self):
        """Returns ice properties with the expected Ice.Initialize args format"""
        if self.config_dict:
            synthetic_argv = ["comp_name"]
            synthetic_argv.extend(
                [f"--{param_name}={param_value}" for param_name, param_value in self.config_dict['ice'].items()]
            )
            return synthetic_argv
        else:
            raise FileNotFoundError("Config file have not been loaded")

    # TODO: check misspelling of known keys


CONFIG_MANAGER = ConfigManager()
