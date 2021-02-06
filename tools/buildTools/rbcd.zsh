# rbcd - a tool to switch between component directories quickly
# the tool is derived from rosbash
# lists all available robocomp components
function _rc_list_locations {
    local packages
    packages=`rcfind list`
    echo $packages | tr ' ' '\n'
    return 0
}

# finds path of the component passed as an argument 
function _rc_location_find {
    local out

    out=`rcfind $1 2>/dev/null`
    if [[ $? != 0 ]]; then
            return 1
    fi
    echo $out
    return 0
}

# takes argument as component name 
# returns 0 if no argument, else if path exists 1
# on success with arguments returns [rcname, abspath, relpath, basename] 
function _rc_decode_path {
    local rcname rcdir reldir last rcpackdir
    rcvals=()
    if [[ -z $1 ]]; then
        return 0
    fi

    echo $1 | grep -G '.\+/.*' > /dev/null
    if [[ $? == 0 ]]; then
        rcname=${1%%/*}
        reldir=/${1#*/}
        last=${reldir##*/}
        reldir=${reldir%/*}/
    else
        rcname=$1
        if [[ -z $2 || $2 != "forceeval" ]]; then
            rcvals=(${rcname})
            return 1
        fi
    fi

        rcpackdir=`rcfind $rcname 2> /dev/null`
        rcpack_result=$?
        if [[ $rcpack_result == 0 ]]; then
            rcdir=$rcpackdir
        else
            rcvals=(${rcname})
            return 1
        fi
    rcvals=(${rcname} ${rcdir} ${reldir} ${last})
}

# helps autocomplete components
function _rccomplete_sub_dir {
    local arg opts rcvals
    reply=()
    arg="$1"
    _rc_decode_path ${arg}
    if [[ -z ${rcvals[3]} ]]; then
        opts=`_rc_list_locations`
        reply=(${=opts})
    else
        if [ -e ${rcvals[2]}${rcvals[3]} ]; then
	    opts=`find ${rcvals[2]}${rcvals[3]} -maxdepth 1 -mindepth 1 -type d ! -regex ".*/[.].*" -print0 | tr '\000' '\n' | sed -e "$sedcmd"`
        else
            opts=''
        fi
        reply=(${=opts})
    fi
}

# shows usage and an error when unknown name passed
# redirects to robocomp's default directory when no argument is passed
 
function rbcd {
    local rcvals
    if [[ $1 = "--help" ]] | [[ $# -gt 1 ]]; then
        echo -e "usage: rbcd component."
        return 0
    fi
    if [ -z $1 ]; then
        cd ~/robocomp
        return 1
    fi

    _rc_decode_path $1 forceeval
    if [ $? != 0 ]; then
      echo -e "rbcd: No such component '$1'"
      return 1
    else
      cd ${rcvals[2]}${rcvals[3]}${rcvals[4]}
      return 0
    fi
}

compctl -K "_rccomplete_sub_dir" -S / "rbcd"
