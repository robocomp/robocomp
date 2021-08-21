import shlex
import subprocess
import sys

import typer


def print_command_result(command, result):
    if result == 0:
        typer.secho(f"Command $ {command} executed OK",  fg=typer.colors.GREEN)
    else:
        typer.secho(f"$ {command} FAILED with result = {result}",  fg=typer.colors.RED)

def execute_command(command):
    result = subprocess.call(shlex.split(command))
    print_command_result(command, result)

def is_interactive():
    return sys.__stdin__.isatty()
