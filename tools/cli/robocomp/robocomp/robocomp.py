import typer as typer
from robocompdsl import main
from rcportchecker import rcportchecker
from rccd import rccd
from rcbuild import rcbuild
from rcrun import rcrun


app = typer.Typer()
app.command(name="dsl")(main.generate)
app.add_typer(rcportchecker.app, name="list")
app.add_typer(rcbuild.app, name="build")
app.add_typer(rcrun.app, name="run")
app.command(name="cd")(rccd.cd_exec)


if __name__ == "__main__":
    app()
