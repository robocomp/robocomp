#!/usr/bin/env python3
import os
import sys
from typing import Optional

import typer
from robocomp import is_interactive, execute_command_in_current_shell

from rcworkspace.workspace import Workspace

app = typer.Typer(help=typer.style("Change directory to Robocomp components", fg=typer.colors.GREEN))


workspace = Workspace()


@app.command(name="cd")
def cd_exec(
        component_name: str = typer.Argument(...,
                                             help="The name of the component, part of a path or part of the name to try to cd to this."),
        option_index: int = typer.Argument(None, help="Index of the selection if multiple options available"),
        interactive: Optional[bool] = typer.Option(is_interactive(), "--interactive/--auto",
                                                   help="Interactive selection of options.")
):
    dir_options = workspace.find_components(component_name)

    if not dir_options:
        print(f"No component name or path found matching {component_name}.")
        return False
    elif len(dir_options) == 1:
        execute_command_in_current_shell(f"cd {str(dir_options[0].path)}")
        return True
    else:
        if option_index:
            if option_index < len(dir_options):
                execute_command_in_current_shell(f"cd {str(dir_options[option_index].path)}")
                return True
        if not interactive:
            for index, option in enumerate(dir_options):
                print(f"[{index}]  {option.name}:\t\t\t({option.path})")
            print("Add the index number of your selection to the end of the last command.")
        return False


if __name__ == '__main__':
    app()
