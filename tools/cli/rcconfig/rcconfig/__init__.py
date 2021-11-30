from pathlib import Path
from pprint import pprint
from typing import Optional

from ruamel.yaml import YAML

yaml = YAML(typ="unsafe")

class RCConfig:
    def __init__(self):
        self.ROBOCOMP_SRC_DIR: Path = Path("~/robocomp").expanduser()
        self.ROBOCOMP_CONFIG_DIR = Path.home() / ".config" / "robocomp"
        self.ROBOCOMP_CONFIG_FILE = self.ROBOCOMP_CONFIG_DIR / "robocomp.yml"
        self.ROBOCOMP_INSTALL_DIR = Path("/opt/robocomp/")
        self.ROBOCOMP_BUILD_DIR = self.ROBOCOMP_SRC_DIR / "build"
        self.CUSTOM = {}
        self.load_config()
        if not self.ROBOCOMP_CONFIG_DIR.exists():
            try:
                self.ROBOCOMP_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
            except PermissionError as pe:
                print(f"ERR: {str(pe)}")
        if not self.ROBOCOMP_BUILD_DIR.exists():
            try:
                self.ROBOCOMP_BUILD_DIR.mkdir(parents=True, exist_ok=True)
            except PermissionError as pe:
                print(f"ERR: {str(pe)}")
        self.load_config()

    def save_config(self) -> None:
        if not self.ROBOCOMP_CONFIG_DIR.exists():
            self.ROBOCOMP_CONFIG_DIR.mkdir(mode=0o777, parents=True, exist_ok=True)
        with self.ROBOCOMP_CONFIG_FILE.open(mode="w") as config_file:
            yaml.dump(self.to_dict(), config_file)

    def load_config(self) -> None:
        if self.ROBOCOMP_CONFIG_FILE.is_file():
            with self.ROBOCOMP_CONFIG_FILE.open("r") as config_file:
                data = yaml.load(config_file)
                for data_key in data:
                    if data_key in self.__dict__:
                        setattr(self, data_key, data[data_key])

        else:
            print(f"ERR: No {self.ROBOCOMP_CONFIG_FILE} exists.")
            print(f"You need to initialize first.")

    def print_config(self, variable: Optional[str] = None):
        if variable is None:
            pprint(RC_CONFIG.to_dict(), sort_dicts=False)
        elif variable in self.__dict__:
            print(f"The value stored in RC_CONFIG[\"{variable}\"] is: {self.__dict__[variable]}")
        else:
            print(f"No '{variable}' found in RC_CONFIG")

    def print_config_file_location(self):
        pprint(str(self.ROBOCOMP_CONFIG_FILE))


    def set_variable(self, variable: str, value: str):
        if variable in self.__dict__:
            setattr(self, variable, value)
        else:
            self.CUSTOM[variable] = value
        self.save_config()

    def get_variable(self, variable: str):
        if variable in self.__dict__:
            return getattr(self, variable)
        elif variable in self.CUSTOM:
            return self.CUSTOM[variable]
        else:
            return None

    def set_src_dir(self, robocomp_src_dir: Path = None):
        if robocomp_src_dir is None:
            home = Path.home()
            if (home / "robocomp" / "README.md").is_file():
                self.ROBOCOMP_SRC_DIR = home / "robocomp"
            elif (Path.cwd() / "README.md").is_file():
                self.ROBOCOMP_SRC_DIR = Path.cwd()
            else:
                print(f'None {str(home / "robocomp" / "README.md")} or {str(Path.cwd() / "README.md")} exist.')
                print(f'ROBOCOMP_SRC_DIR is expected to have "README.md" file.')
        self.save_config()

    def to_dict(self):
        return {a: getattr(self, a) for a in dir(self) if
                not a.startswith('__') and not callable(getattr(self, a))}


RC_CONFIG = RCConfig()
