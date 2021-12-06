from pathlib import Path
from typing import Optional

import typer
from rcconfig import RC_CONFIG
from robocomp import execute_command_in_current_shell

app = typer.Typer(help=typer.style("Config robocomp variables system wide.", fg=typer.colors.GREEN))


@app.command("print")
def rc_print(variable: Optional[str] = typer.Argument(None)):
    RC_CONFIG.print_config(variable)


@app.command("init", help="Initialize the config file setting the Robocomp src directory")
def set_src_dir(robocomp_src_dir: Path = None):
    RC_CONFIG.print_config_file_location()
    RC_CONFIG.set_src_dir(robocomp_src_dir)

@app.command("set")
def set_variable(variable: str, value: str):
    RC_CONFIG.set_variable(variable, value)

@app.command("shell")
def to_shell():
    all_variables = RC_CONFIG.to_dict()
    commands = [f"export {variable}='{str(value)}'" for variable, value in all_variables.items() if isinstance(value, (str, Path))]
    command = "; ".join(commands)
    execute_command_in_current_shell(f"eval \"{command}\"")
    # print(f"eval \"{command}\"")

@app.command("location")
def print_config_file_location():
    RC_CONFIG.print_config_file_location()


if __name__ == '__main__':
    app()
