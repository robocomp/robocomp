import multiprocessing
import os
from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import typer
from robocomp import execute_command
try:
    from pyaku.pyaku import Yaku
except ModuleNotFoundError:
    Yaku = False


class ComponentCheck(ABC):
    @classmethod
    @abstractmethod
    def check(cls, component_dir: Path, component=None):
        """Abstract method"""

class ComponentCDSLCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        if not component_dir.exists():
            return False, component
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
        full_path = component_dir / "etc"
        if full_path.exists():
            if full_path.is_dir():
                if full_path.glob("config"):
                    filepath = full_path / "config"
                    component._config_file_path = filepath
                else:
                    for file in list(full_path.iterdir()):
                        if "config" in str(file.name):
                            filepath = full_path / file
                            config_files[component_dir].append(filepath)
                            component._config_file_path = filepath
        return component._config_file_path is not None, component

class ComponentNoTrashCheck(ComponentCheck):
    @classmethod
    def check(cls, component_dir: Path, component=None):
        return '.local/share/Trash' not in str(component_dir.absolute()), component

class LanguageType(Enum):
    PYTHON = 1
    CPP = 2

class ComponentDir:
    """Class for keeping track of a component in workspace"""
    checks = [ComponentCDSLCheck, ComponentNoTrashCheck, ComponentConfigCheck, ComponentCMakeListsCheck]

    def __init__(self):
        self.cdsl_file_path: Path = None
        self.cmakelists_file_path: Path = None

        self._language: LanguageType = None

        self.path: Path = None
        self._etc_path = None
        self._name: str = None
        self._src_path: Path = None
        self._build_path: Path = None
        self._bin_path: Path = None
        self._bin_file_path: Path = None
        self._config_file_path: Path = None

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
            self._src_path = self.path / "src"
        return self._src_path

    @property
    def bin_path(self):
        if self._bin_path is None:
            self._bin_path = self.path / "bin"
        return self._bin_path

    @property
    def bin_file_path(self):
        ''' find the directory containing component executable'''
        if self._bin_file_path is None:
            if self.bin_path.is_dir():
                bin_file_path = self.bin_path / self.name
                if os.access(bin_file_path, os.X_OK):
                    self._bin_file_path = bin_file_path
                    self._language = LanguageType.CPP
            elif self.src_path.is_dir():
                python_main_path = self.src_path / (self.name+".py")
                if python_main_path.exists():
                    self._bin_file_path = python_main_path
                    self._language = LanguageType.PYTHON
        return self._bin_file_path

    @property
    def etc_path(self):
        if self._etc_path is None:
            self._etc_path = self.path / "etc"
        return self._etc_path

    @property
    def config_file_path(self):
        if self._config_file_path is None:
            if self.etc_path.is_dir():
                config_file_path = self.etc_path / "config"
                if config_file_path.is_file():
                    self._config_file_path = config_file_path
        return self._config_file_path

    @property
    def build_path(self):
        if self._build_path is None:
            self._build_path = self.path / "build"
        return self._build_path

    @property
    def language(self) -> LanguageType:
        if self._language is None:
            # getting the value forces the property to be updated
            self.bin_file_path
        return self._language

    def build(self, nproc=None, clean=False):
        if nproc is None:
            nproc = int(multiprocessing.cpu_count() / 2)
        print()
        typer.secho(f"Starting build of {self.name}", fg=typer.colors.GREEN)
        typer.secho(f"=============================", fg=typer.colors.GREEN)
        self.build_path.mkdir(exist_ok=True)
        if clean:
            self.clean_build()
        os.chdir(self.build_path)
        print(f"Working on dir {os.getcwd()}")
        execute_command('cmake ..')
        execute_command(f'make -j {nproc}')

    def clean_build(self, delete_bin=True, delete_new=True, delete_ice=True):
        # TODO: make more generic. config file to select files to delete?
        # TODO: interactive mode
        items_to_remove = []
        if self.build_path:
            if (self.build_path / "Makefile").is_file():
                execute_command("make clean")
                something_have_been_cleaned=True
            if self.build_path.is_dir():
                items_to_remove.append(str(self.build_path))
        cmakecache_file = self.path / "CMakeCache.txt"
        if cmakecache_file.is_file():
            items_to_remove.append(str(cmakecache_file))
        if delete_bin and self.bin_path and self.bin_path.is_dir():
            items_to_remove.append(str(self.bin_path))
        if delete_ice and self.path and self.path.is_dir():
            for ice_file in self.path.rglob('*.ice'):
                items_to_remove.append(str(ice_file))
        if delete_new and self.path and self.path.is_dir():
            for new_file in self.path.rglob('*.new'):
                items_to_remove.append(str(new_file))
        for build_dir in self.path.rglob('cmake-build-*'):
            items_to_remove.append(str(build_dir))
        # generated_interface_files = self.src_path.glob('*I.h')
        # for interface_file in generated_interface_files:
        #     interface_name = interface_file.name.split('I')[0]
        #     cpp_file = self.src_path / (interface_name+"I.cpp")
        #     if interface_name.islower() and cpp_file.is_file():
        #         items_to_remove.append(str(interface_file))
        #         items_to_remove.append(str(cpp_file))
        for item in items_to_remove:
            execute_command(f"rm -r {item}")
        return bool(len(items_to_remove))

    def run_component(self, use_yaku=True):
        if self.language == LanguageType.PYTHON:
            if Yaku:
                yaku = Yaku()
                yaku.run_in_new_tab(f"cd {self.path}; python3 {self.bin_file_path} {self.config_file_path}&", self.name)
            else:
                os.chdir(self.path)
                execute_command(f"python3 {self.bin_file_path} {self.config_file_path}")
