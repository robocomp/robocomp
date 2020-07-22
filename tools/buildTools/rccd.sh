rccd()
{
    rccd_utils $@
    directory=$(cat /tmp/*_rccd.output | tr -cd '[:print:]' || echo "")
    rm /tmp/*_rccd.output > /dev/null 2>&1
    [ ! -z "$directory" ] && cd "$directory" && yaku || echo "No valid component found"
}

