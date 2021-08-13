import typer as typer
from rcbuild import rcbuild
from rccd import rccd
from rcconfig import rcconfig
from rcdocker import rcdocker
from rcportchecker import rcportchecker
from rcrun import rcrun
from rcworkspace import rcworkspace
from robocompdsl import main


app = typer.Typer()
app.add_typer(rcbuild.app, name="build")
app.add_typer(rcconfig.app, name="config")
app.add_typer(rcdocker.app, name="docker")
app.add_typer(rcportchecker.app, name="list")
app.add_typer(rcrun.app, name="run")
app.add_typer(rcworkspace.app, name="workspace")
app.command(name="cd")(rccd.cd_exec)
app.command(name="dsl")(main.generate)


if __name__ == "__main__":
    app()
