#!/usr/bin/env python3

from __future__ import print_function
import sys
from pathlib import Path
from typing import Optional

import typer

from rcworkspace.workspace import Workspace

ws = Workspace()
execute_default = True
app = typer.Typer(help=typer.style("Tool to find and check available components.", fg=typer.colors.GREEN))

@app.command()
def initialize():
    """
    Initialize workspaces searching in the given directory
    """
    try:
        ws.interactive_workspace_init(initialize)
    except KeyboardInterrupt:
        print("\nCanceled")


@app.command()
def delete(directory: Optional[Path] = typer.Argument(None, help="Dir to start removing workspace")):
    """
    Remove workspaces searching in the given directory
    """
    try:
        ws.delete_workspace(directory)
    except KeyboardInterrupt:
        print("\nCanceled")


@app.command()
def add(directory: Optional[Path] = typer.Argument(None, help="Dir to start adding workspace"), accept_all: bool = False):
    """
    Add workspaces searching in the given directory
    """
    try:
        ws.add_workspace(directory, interactive=False, accept_all=accept_all)
    except KeyboardInterrupt:
        print("\nCanceled")


@app.command()
def update():
    """
    Update the components for the existing workspaces
    """
    try:
        ws.update_components_in_workspaces()
    except KeyboardInterrupt:
        print("\nCanceled")


@app.command()
def clear_all():
    """
    Clear all the information of the workspaces (requires confirmation)
    """
    try:
        ws.clear_all()
    except KeyboardInterrupt:
        print("\nCanceled")


@app.command(name="list")
def list_components():
    """
    List the current existing workspaces and components
    """
    try:
        ws.list_workspaces()
    except KeyboardInterrupt:
        print("\nCanceled")


if __name__ == '__main__':
    app()

