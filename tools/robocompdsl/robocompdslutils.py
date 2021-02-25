import os
import sys
from distutils import spawn
from rich.console import Console

sys.path.append("/opt/robocomp/python")

DIFF_TOOLS = ["meld", "kdiff3", "diff"]

console = Console()


def get_diff_tool(prefered=None):
    if prefered in DIFF_TOOLS:
        tool_path = spawn.find_executable(prefered)
        if tool_path != "":
            return prefered, tool_path
    for tool in DIFF_TOOLS:
        tool_path = spawn.find_executable(tool)
        if tool_path != "":
            return tool, tool_path
    return None, None


def replaceTagsInFile(path):
    i = open(path, 'r', encoding='utf-8', errors='ignore')
    text = i.read()
    reps = [
        ["\n<@@<", ""],
        [">@@>\n", ""],
        ["<TABHERE>", '\t'],
        ["<S1>", ' '],
        ["<S2>", '  '],
        ["<S4>", '    ']
    ]
    for r in reps:
        text = text.replace(r[0], r[1])
    i.close()
    w = open(path, 'w')
    w.write(text)
    w.close()


def create_directory(directory):
    try:
        console.print(f"Creating {directory}")
        os.mkdir(directory)
        console.print('')
    except OSError:
        if os.path.isdir(directory):
            console.print('(already existed)', style='yellow')
            pass
        else:
            raise RuntimeError('\nCOULDN\'T CREATE %s' % directory)
