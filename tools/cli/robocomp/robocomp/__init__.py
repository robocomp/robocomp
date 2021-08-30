import shlex
import subprocess
import sys

import typer

# TODO: make robocomp a class with its own attributes like SRC dir, INSTALL dir and operations as ¿download?, install, clean, etc

def print_command_result(command, result):
    if result == 0:
        typer.secho(f"${command} executed OK",  fg=typer.colors.GREEN)
    else:
        typer.secho(f"${command} FAILED with result = {result}",  fg=typer.colors.RED)

def execute_command(command, dry_command=False):
    result = True
    if not dry_command:
        result = subprocess.call(shlex.split(command))
    print_command_result(command, result)

def is_interactive():
    return sys.__stdin__.isatty()
