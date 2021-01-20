# rbcd - a tool to switch between component directories quickly
# the tool is derived from rosbash
# lists all available robocomp components
function _rc_list_locations {
    local packages
    packages=$(rcfind list)
    echo $packages | tr ' ' '\n'
    return 0
}

# finds path of the component passed as an argument 
function _rc_location_find {
    local out

    out=$(rcfind $1 2>/dev/null)
    if [ -z "${out}" ]; then
        return 1
    fi
    echo $out
    return 0
}

# takes argument as component name 
# returns 0 if no argument, else if path exists 1
# on success with arguments returns [rcname, abspath, relpath, basename] 
function _rc_decode_path {
    local rcname rcdir reldir last
    rcvals=()
    if [[ -z "$1" ]]; then
        return 0
    fi

    if [[ $1 =~ .+/.* ]]; then
        rcname=${1%%/*}
        reldir=/${1#*/}
        last=${reldir##*/}
        reldir=${reldir%/*}/
    else
        rcname="$1"
        if [[ -z "$2" || "$2" != "forceeval" ]]; then
           rcvals=("${rcname}")
           return 1
        fi
    fi
    rcdir=$(_rc_location_find "$rcname")
    if [[ $? != 0 ]]; then
        rcvals=("${rcname}")
        return 1
    fi

    rcvals=("${rcname}" "${rcdir}" "${reldir}" "${last}")
}

# helps autocomplete components
function _rccomplete_sub_dir {
    local arg opts rcvals
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"
    _rc_decode_path ${arg}
    if [[ -z "${rcvals[2]}" ]]; then
        opts=$(_rc_list_locations)
        COMPREPLY=($(compgen -W "${opts}" -S / -- ${rcvals[0]}))
    else
        if [ -e ${rcvals[1]}${rcvals[2]} ]; then
          opts=$(find -L ${rcvals[1]}${rcvals[2]} -maxdepth 1 -mindepth 1 -type d ! -regex ".*/[.][^./].*" -print0 | tr '\000' '\n' | sed -e "s/.*\/\(.*\)/\1\//g")
        else
          opts=''
        fi
        COMPREPLY=($(compgen -P ${rcvals[0]}${rcvals[2]} -W "${opts}" -- ${rcvals[3]}))
    fi
}

# shows usage and an error when unknown name passed
# redirects to robocomp's default directory when no argument is passed
 
function rbcd {
    local rcvals
    if [[ $1 = "--help" ]] || [[ $# -gt 1 ]]; then
        echo -e "usage: rbcd component"
        return 0
    fi
    if [ -z $1 ]; then
        cd ~/robocomp
        return 1
    fi

    _rc_decode_path $1 forceeval
    if [ $? != 0 ]; then
      echo "rbcd: No such component '$1'"
      return 1
    else
      cd ${rcvals[1]}${rcvals[2]}${rcvals[3]}
      return 0
    fi
}

complete -F "_rccomplete_sub_dir" -o "nospace" "rbcd"
