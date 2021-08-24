rcbuild
==
[![Join the chat at https://gitter.im/robocomp/robocomp](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/robocomp/robocomp?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Robocomp command to build components or robocomp itself. There are also several options to clean, rebuild or install. The name of the component to be used doesn't need to be a full path. 
The passed name will be searched inside the existing workspaces.

```bash
$ rcbuild --help    
Usage: rcbuild [OPTIONS] COMMAND [ARGS]...

  Robocomp command to build components or robocomp itself. There are
  also several options to clean, rebuild or install. The name of the
  component to be used doesn't need to be a full path. The passed name will
  be looked for inside the defined workspaces.

Options:
  --install-completion [bash|zsh|fish|powershell|pwsh]
                                  Install completion for the specified shell.
  --show-completion [bash|zsh|fish|powershell|pwsh]
                                  Show completion for the specified shell, to
                                  copy it or customize the installation.

  --help                          Show this message and exit.

Commands:
  clean     Clean CMake dir and Make files of the component.
  comp      Build the specified component.
  doc       Build documentation of the specified component
  robocomp  Build robocomp itself.
```