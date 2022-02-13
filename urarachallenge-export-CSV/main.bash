#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
_USER=${SCRIPT_DIR##*home/}
USER=${_USER%%/*}

CACHE_PATH=$SCRIPT_DIR/cache

rm -rf $CACHE_PATH
mkdir -p $CACHE_PATH
echo "CACHE_PATH: $CACHE_PATH"
VIDEO_PATH=`python3 $SCRIPT_DIR/select-folder-file-dialog-pyqt5/select-file-folder-dialog.py -f -e $HOME`

ffmpeg -i $VIDEO_PATH  -vcodec png -r 2 -vf scale=640:360 $CACHE_PATH/image_%03d.png

python3 $SCRIPT_DIR/qt_image_viewer.py --image-cache-path $CACHE_PATH --csv-folder $SCRIPT_DIR
# python3 qt_image_viewer.py --image-cache-path /home/ubuntu/Documents/github/urarachallenge-CheckGUI/cache --csv-folder /home/ubuntu/Documents/github/urarachallenge-CheckGUI
exit 0