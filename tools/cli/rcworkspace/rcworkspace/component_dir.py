import os
from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from robocomp import execute_command


class ComponentCheck(ABC):
    @classmethod
    @abstractmethod
    def check(cls, component_dir: Path, component=None):
        """Abstract method"""

class ComponentCDSLCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        cdsl_files = defaultdict(list)
        for item in component_dir.iterdir():
            if item.is_file() and str(item).endswith(".cdsl"):
                filepath = component_dir / item
                cdsl_files[component_dir].append(filepath)
                component.cdsl_file_path = filepath
        return len(cdsl_files) > 0, component

class ComponentCMakeListsCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        if "CMakeLists.txt" in os.listdir(component_dir):
            component.cmakelists_file_path = os.path.join(component_dir, "CMakeLists.txt")
            return True, component
        return False, component

class ComponentConfigCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        config_files = defaultdict(list)
        if full_path := component_dir / "etc":
            if full_path.is_dir():
                for file in list(full_path.iterdir()):
                    if "config" in str(file.name):
                        filepath = full_path / file
                        config_files[component_dir].append(filepath)
                        component._config_file_path = filepath
        return len(config_files) > 0, component

class ComponentNoTrashCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        return '.local/share/Trash' not in str(component_dir.absolute()), component

@dataclass(init=False)
class ComponentDir:
    """Class for keeping track of a component in workspace"""

    bin_dir_path: Path
    bin_file_path: Path
    cdsl_file_path: Path
    cmakelists_file_path: Path
    config_file_path: Path
    language: Enum('Python', 'C++')
    checks = [ComponentCDSLCheck, ComponentNoTrashCheck, ComponentConfigCheck, ComponentCMakeListsCheck]
    path: Path = None
    _name: str = None
    _src_path: Path = None
    _build_path: Path = None


    @classmethod
    def create_component(cls, path: Path):
        component = ComponentDir()
        for current_check in cls.checks:
            result, component = current_check.check(path, component)
            if not result:
                return None
        component.path = path
        return component

    @property
    def name(self):
        if self._name is None:
            if self.path is not None:
                self._name = self.path.name
        return self._name

    @property
    def src_path(self):
        if self._src_path is None:
            src_path = self.path / "src"
            if src_path.exists():
                self._src_path = src_path
        return self._src_path

    @property
    def bin_path(self):
        if self._bin_path is None:
            bin_path = self.path / "bin"
            if bin_path.exists():
                self._bin_path = bin_path
        return self._bin_path

    @property
    def bin_file_path(self):
        ''' find the directory containing component executable'''
        if self.bin_file_path is None:
            if self.bin_path.is_dir():
                bin_file_path = self.bin_path / self.name
                if os.access(bin_file_path, os.X_OK):
                    self._bin_file_path = bin_file_path
                    self.language = 'C++'
            elif self.src_path.is_dir():
                python_main_path = self.src_path / (self.name+".py")
                if python_main_path.exists():
                    self._bin_file_path = python_main_path
                    self.language = 'Python'
        return self.bin_file_path

    @property
    def etc_path(self):
        if self._etc_path is None:
            etc_path = self.path / "etc"
            if etc_path.exists():
                self._etc_path = etc_path
        return self._etc_path

    @property
    def config_file_path(self):
        if self.config_file_path is None:
            if self.bin_path.is_dir():
                config_file_path = self.bin_path / self.name
                if os.access(config_file_path, os.X_OK):
                    self._config_file_path = config_file_path
            elif self.src_path.is_dir():
                python_main_path = self.src_path / (self.name+".py")
                if python_main_path.exists():
                    self._config_file_path = python_main_path
        return self._config_file_path

    @property
    def build_path(self):
        if self._build_path is None:
            build_path = self.path / "build"
            if build_path.exists():
                self._build_path = build_path
        return self._build_path

    def build(self, clean=False):
        self.build_path.mkdir(exist_ok=True)
        if clean:
            self.clean_build()
        os.chdir(self.build_path)
        print(f"Working on dir {os.getcwd()}")
        execute_command('cmake ..')
        execute_command('make')

    def clean_build(self):
        if (self.build_path / "Makefile").is_file():
            execute_command("make clean")
        if self.build_path.is_dir():
            execute_command(f"rm -r {self.build_path}")
        if (cmakecache_file := self.path / "CMakeCache.txt").is_file():
            os.remove(cmakecache_file)
