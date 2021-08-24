rccd
==

[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Simple tool to cd to Robocomp components directories. 

It's pretty common to have many components in many path and it's usually needed to open new terminal and cd to the full
path where the component are located.  

With this tool it's as simple as using the name of the component or part of the path to get suggestions of paths
for component with that name or to just chnage directory if only one exists.

```bash
$ rccd --help                                                                                       
Usage: rccd [OPTIONS] COMPONENT_NAME [OPTION_INDEX]

Arguments:
  COMPONENT_NAME  The name of the component, part of a path or part of the
                  name to try to cd to this.  [required]

  [OPTION_INDEX]  Index of the selection if multiple options available

Options:
  --interactive / --auto          Interactive selection of options.
  --install-completion [bash|zsh|fish|powershell|pwsh]
                                  Install completion for the specified shell.
  --show-completion [bash|zsh|fish|powershell|pwsh]
                                  Show completion for the specified shell, to
                                  copy it or customize the installation.

  --help                          Show this message and exit.
```