#!/usr/bin/env python3
import os
import sys
import tempfile
import typer

sys.path.append('/opt/robocomp/python')
from workspace import Workspace

app = typer.Typer()


def save_output(output):
    fd, path = tempfile.mkstemp("_rccd.output", text=True)
    file = os.fdopen(fd, 'w')
    file.write(output)
    file.close()


class rccd:
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

    def filtered_components(self, searched_component):
        return self.ws.find_components(searched_component)


dir_changer = rccd()


# Horrible hack to be able to change directory in the current terminal session
def change_parent_process_directory(dest):
    def quote_against_shell_expansion(s):
        import pipes
        return pipes.quote(s)

    def put_text_back_into_terminal_input_buffer(text):
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


@app.command()
def cd(
        component_name: str = typer.Argument(..., help="The name of the component, part of a path or part of the name to try to cd to this."),
        option_index: int = typer.Argument(None, help="Index of the selection if multiple options available")
):
    dir_options = dir_changer.filtered_components(component_name)
    if len(dir_options) == 1:
        change_parent_process_directory(dir_options[0])
        return True
    else:
        if option_index:
            if option_index < len(dir_options):
                change_parent_process_directory(dir_options[option_index])
                return True
        for index, option in enumerate(dir_options):
            print(f"[{index}] {option}")
        print("Add the index number of your selection to the end of the last command.")
        return False


if __name__ == '__main__':
    try:
        app()
    except KeyboardInterrupt:
        save_output("")





