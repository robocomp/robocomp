import shlex
import subprocess
import sys
import typer

# TODO: make robocomp a class with its own attributes like SRC dir, INSTALL dir and operations as ¿download?, install, clean, etc


def execute_command(command, dry_command=False):
    print("======EXECUTING COMMAND======")
    print(f"${command}")
    if not dry_command:
        output = ""
        return_code = 0
        try:
            output = subprocess.check_output(shlex.split(command), stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            return_code = e.returncode
            output = e.output
    if return_code == 0:
        typer.secho(f"OK", fg=typer.colors.GREEN)
    else:
        typer.secho(f"FAILED with {output} and return_code = {return_code}", fg=typer.colors.RED)
    print("=============================")

def is_interactive():
    return sys.__stdin__.isatty()
