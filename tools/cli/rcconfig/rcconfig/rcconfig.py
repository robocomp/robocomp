import json
import os
from pathlib import Path
import typer

app = typer.Typer()


def save_config():
    home = os.path.expanduser("~")
    config_file_path = os.path.join(home, f".config/RoboComp/config.json")
    if not os.path.exists(os.path.join(home, ".config/RoboComp")):
        config_path = os.path.join(home, ".config/RoboComp")
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
        return config
    except FileNotFoundError:
        return {}

RC_CONFIG = load_config()

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

if __name__ == '__main__':
    app()