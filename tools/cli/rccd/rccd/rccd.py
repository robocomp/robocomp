#!/usr/bin/env python3
import os
import sys
import tempfile
import typer
from typing import Optional

from rcworkspace.workspace import Workspace
from robocomp import is_interactive

app = typer.Typer(help=typer.style("Change directory to Robocomp components", fg=typer.colors.GREEN))


def save_output(output):
    fd, path = tempfile.mkstemp("_rccd.output", text=True)
    file = os.fdopen(fd, 'w')
    file.write(output)
    file.close()


class RCcd:
    def __init__(self):
        self.ws = Workspace()

    def save_filtered_component(self, searched_component):
        components = self.filtered_components()
        if components is not None:
            if len(components) == 1:
                save_output(f"{components[0]}")
            elif len(components) > 1:
                selected = self.ws.ask_for_path_selection(components)
                if selected is None:
                    save_output("")
                else:
                    save_output(selected)
            else:
                save_output("")
        else:
            save_output("")

    def filtered_components(self, searched_component: str, interactive: bool = True) -> list:
        if interactive:
            return [self.ws.find_component(searched_component, interactive)]
        else:
            return self.ws.find_components(searched_component)


dir_changer = RCcd()


# Horrible hack to be able to change directory in the current terminal session
def change_parent_process_directory(dest):
    def quote_against_shell_expansion(s: str):
        import pipes
        return pipes.quote(s)

    def put_text_back_into_terminal_input_buffer(text: str):
        if os.isatty(sys.stdout.fileno()):
            import fcntl, termios
            try:
                # (and if the user types while it runs they could insert characters between the characters in 'text'!)
                for c in text:
                    fcntl.ioctl(1, termios.TIOCSTI, c)
            except OSError as e:
                print(f"Problem finding interactive terminal to execute cd.\n>>{e}")
        else:
            print(f"Problem finding interactive terminal to execute cd.")
    # the horror
    put_text_back_into_terminal_input_buffer("cd "+quote_against_shell_expansion(dest)+"\n")


@app.command(name="cd")
def cd_exec(
        component_name: str = typer.Argument(..., help="The name of the component, part of a path or part of the name to try to cd to this."),
        option_index: int = typer.Argument(None, help="Index of the selection if multiple options available"),
        interactive: Optional[bool] = typer.Option(None, "--interactive/--auto", help="Interactive selection of options.")
):
    if interactive is None:
        interactive = is_interactive()
    dir_options = dir_changer.filtered_components(component_name, interactive)

    if len(dir_options) == 1:
        change_parent_process_directory(str(dir_options[0].path))
        return True
    else:
        if option_index:
            if option_index < len(dir_options):
                change_parent_process_directory(str(dir_options[option_index].path))
                return True
        if not interactive:
            for index, option in enumerate(dir_options):
                print(f"[{index}]  {option.name}:\t\t\t({option.path})")
            print("Add the index number of your selection to the end of the last command.")
        return False


if __name__ == '__main__':
    try:
        app()
    except KeyboardInterrupt:
        save_output("")





