#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)
_USER=${SCRIPT_DIR##*home/}
USER=${_USER%%/*}

SELECT_FOLDER_FILE_DIALOG=$SCRIPT_DIR/../select-file-folder-dialog-PyQt5/select-folder-file-dialog.py

MP4S_DIR=`python3 $SELECT_FOLDER_FILE_DIALOG -t "Select mp4s folder" -e $HOME/Videos`
CSV_FILE=`python3 $SELECT_FOLDER_FILE_DIALOG -t "Select csv file" -e $HOME/Videos -f`

for mp4 in $MP4S_DIR/*.mp4; do
    echo $mp4
    python3 $SCRIPT_DIR/urarachallenge.py --csv_path $CSV_FILE --video_file $mp4
done