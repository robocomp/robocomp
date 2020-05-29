rccd()
{
    rccd_utils $@ |& tee log.txt
    directory=$(cat log.txt| tail -n3 | head -n1 | sed -e "s,[^\/]*\(\/[[:print:]]\+\),\1,g" | tr -cd '[:print:]' | grep '/')
    [ ! -z "$directory" ] && cd "$directory" || echo "No folder specified"
}

