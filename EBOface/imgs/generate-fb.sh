for entry in "$search_dir"./*
do
  if [[ $entry == *.png ]];  then
    sudo fbi -T 2 -d /dev/fb0 -noverbose -a "$entry"
    sleep 1
    cat /dev/fb0 > frameBuffer/${entry%.png}.fb
    echo "$entry"
  fi
done
