rccd()
{
    rccd_utils $@ |& tee log.txt
    directory=$(sed -e "s,[^\/]*\(\/[[:print:]]\+\),\1,g" log.txt | grep '/' | tail -n1 | tr -cd '[:print:]' )
    [ ! -z "$directory" ] && cd "$directory" || echo "No folder specified"
}

