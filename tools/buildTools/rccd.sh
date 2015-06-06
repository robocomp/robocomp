#!/usr/bin/bash

rccd()
{
    flag=0;
    if [[ $1 = "--help" ]] || [[ $1 = "-h" ]] || [[ $# -gt 1 ]] || [[ -z $1 ]]; then
        echo -e "usage: rccd Component \n\n Jump to target Component\n";
        return 1;
    fi;

    if [[ ! -f ~/.config/RoboComp/rc_workspace.config ]]; then
        echo -e "No Robocomp workspaces found \n use rc_init_ws to create one";
        return 1;
    fi;

    pwd=$(pwd);
    while read -r line || [[ -n $line ]] && [[ flag -eq 0 ]] ; do
        #echo $line
        cd $line;cd src;
        for file in *; do
            #echo $file;
            if [[ "$file" == "$1" ]] && [[ -d "$file" ]] ; then
                cd $file;
                flag=1;
                break;
            fi;
        done;
    done < ~/.config/RoboComp/rc_workspace.config ;
    
    if [[ $flag -eq 0 ]]; then
        cd $pwd
        echo "  No such component exists "
    fi

}
