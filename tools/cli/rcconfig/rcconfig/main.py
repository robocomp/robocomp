from pathlib import Path
from typing import Optional

import typer
from rcconfig import RC_CONFIG

app = typer.Typer(help=typer.style("Config robocomp variables system wide.", fg=typer.colors.GREEN))


@app.command("print")
def rc_print(variable: Optional[str] = typer.Argument(None)):
    RC_CONFIG.print(variable)


@app.command("init")
def set_src_dir(robocomp_src_dir: Path = None):
    RC_CONFIG.print_config_file_location()
    RC_CONFIG.set_src_dir(robocomp_src_dir)


@app.command("location")
def print_config_file_location():
    RC_CONFIG.print_config_file_location()


if __name__ == '__main__':
    app()
