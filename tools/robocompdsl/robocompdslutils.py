import os
import sys
from distutils import spawn

sys.path.append("/opt/robocomp/python")

DIFF_TOOLS = ["meld", "kdiff3", "diff"]


def get_diff_tool(prefered=None):
    if prefered in DIFF_TOOLS:
        tool_path = spawn.find_executable(prefered)
        if tool_path is not "":
            return prefered, tool_path
    for tool in DIFF_TOOLS:
        tool_path = spawn.find_executable(tool)
        if tool_path is not "":
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
        print('Creating', directory,)
        os.mkdir(directory)
        print('')
    except OSError:
        if os.path.isdir(directory):
            print('(already existed)')
            pass
        else:
            raise RuntimeError('\nCOULDN\'T CREATE %s' % directory)
