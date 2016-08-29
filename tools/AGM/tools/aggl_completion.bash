_agglcomplete_ ()
{
	local IFS=$'\n'
	local LASTCHAR=' '

	COMPREPLY=($(compgen -o plusdirs -f -X '!*.aggl' -- "${COMP_WORDS[COMP_CWORD]}"))

	if [ ${#COMPREPLY[@]} = 1 ];
	then
		[ -d "$COMPREPLY" ] && LASTCHAR=/
		COMPREPLY=$(printf %q%s "$COMPREPLY" "$LASTCHAR")
	else
		for ((i=0; i < ${#COMPREPLY[@]}; i++));
		do
			[ -d "${COMPREPLY[$i]}" ] && COMPREPLY[$i]=${COMPREPLY[$i]}/
		done
	fi

	return 0
}

complete -o nospace -F _agglcomplete_ AGGLEditor
complete -o nospace -F _agglcomplete_ aggleditor




