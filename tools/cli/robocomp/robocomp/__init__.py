import os
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

def execute_command_in_current_shell(command, dry_command=False):
    # Horrible hack to be able to change directory in the current terminal session
    def quote_against_shell_expansion(s: str):
        import pipes
        return pipes.quote(s)

    def put_text_back_into_terminal_input_buffer(text: str):
        if os.isatty(sys.stdout.fileno()):
            import fcntl
            import termios
            try:
                # (and if the user types while it runs they could insert characters between the characters in 'text'!)
                for c in text:
                    fcntl.ioctl(1, termios.TIOCSTI, c)
            except OSError as e:
                print(f"Problem finding interactive terminal to execute cd.\n>>{e}")
        else:
            print(f"Problem finding interactive terminal to execute cd.")

    args = " ".join([quote_against_shell_expansion(argument) for argument in shlex.split(command)[1:]])
    # the horror
    put_text_back_into_terminal_input_buffer(f"{command} {args} \n")

def is_interactive():
    return sys.__stdin__.isatty()
