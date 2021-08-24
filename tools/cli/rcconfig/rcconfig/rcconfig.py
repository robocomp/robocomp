import json
import os
from pathlib import Path
import typer
from pprint import pprint
from typing import Optional



ROBOCOMP_CONFIG_PATH = Path.home() / ".config/RoboComp"
ROBOCOMP_CONFIG_FILE = ROBOCOMP_CONFIG_PATH / "config.json"

app = typer.Typer(help=typer.style("Config robocomp variables system wide.", fg=typer.colors.GREEN))


def save_config():
    home = os.path.expanduser("~")
    config_file_path = os.path.join(home, ROBOCOMP_CONFIG_FILE)
    if not os.path.exists(os.path.join(home, ROBOCOMP_CONFIG_PATH)):
        config_path = os.path.join(home, ROBOCOMP_CONFIG_PATH)
        os.makedirs(config_path)
        os.chmod(config_path, 0o777)
    try:
        config_file = open(config_file_path, "w")
        json.dump(RC_CONFIG, config_file)
        config_file.close()
    except FileNotFoundError:
        pass

def load_config():
    home = os.path.expanduser("~")
    config_file_path = os.path.join(home, f".config/RoboComp/config.json")

    if not os.path.exists(os.path.join(home, ".config/RoboComp")):
        config_path = os.path.join(home, ".config/RoboComp")
        os.makedirs(config_path)
        os.chmod(config_path, 0o777)

    try:
        config_file = open(config_file_path, "r")
        config = json.load(config_file)
        config_file.close()
        if "ROBOCOMP_SRC" not in config:
            config["ROBOCOMP_SRC"] = Path.home() / "robocomp"
        return config
    except FileNotFoundError:
        return {}

RC_CONFIG = load_config()

@app.command("print")
def rc_print(variable: Optional[str] = typer.Argument(None)):
    if variable is None:
        typer.secho("RC_CONFIG{...}", fg=typer.colors.GREEN)
        pprint(RC_CONFIG, sort_dicts=False)
    elif variable in RC_CONFIG:
        print(f"The value stored in RC_CONFIG[\"{variable}\"] is: {RC_CONFIG[variable]}")
    else:
        typer.secho(f"No '{variable}' found in RC_CONFIG", fg=typer.colors.YELLOW)


@app.command("set-src-dir")
def set_robocomp_src_dir(robocomp_src_dir: Path = None):
    if robocomp_src_dir is None:
        home = Path.home()
        if (home / "robocomp" / "README.md").is_file():
            RC_CONFIG["ROBOCOMP_SRC"] = str(home / "robocomp")
        else:
            RC_CONFIG["ROBOCOMP_SRC"] = str(Path.cwd())
    print(f"Set ROBOCOMP_SRC to {RC_CONFIG['ROBOCOMP_SRC']}")
    save_config()

@app.command("location")
def print_config_file_location():
    pprint(str(ROBOCOMP_CONFIG_FILE))

if __name__ == '__main__':
    app()