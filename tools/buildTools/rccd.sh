rccd()
{
    rccd_utils $@
    directory=$(cat /tmp/*_rccd.output | tr -cd '[:print:]' )
    rm /tmp/*_rccd.output
    [ ! -z "$directory" ] && cd "$directory" || echo "No valid component found"
    yaku
}

