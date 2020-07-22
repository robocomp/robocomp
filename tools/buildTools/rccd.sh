rccd()
{
  if [ "$#" -ne 1 ];
  then
    echo "usage: rccd name"
    return
  fi
  rccd_utils $@
  if [ `ls -1 /tmp/*_rccd.output 2>/dev/null | wc -l` -gt 0 ];
  then
    directory=$(cat /tmp/*_rccd.output | tr -cd '[:print:]' || echo "")
    rm /tmp/*_rccd.output > /dev/null 2>&1
    [ ! -z "$directory" ] && cd "$directory" && yaku || echo "No valid component found"
  else
    echo "usage: rccd name"
  fi
}

