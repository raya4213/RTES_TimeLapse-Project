#/bin/sh
DATESTAMP=$(date +'%d%m%Y%H%M%S')
VIDEO_FILE="timelapse_$DATESTAMP.mp4"
LOG_FILE="log_$DATESTAMP.log"
TIMESTAMPER="date +'%D %H:%M:%S'"

echo "Image to Video shell script called" > $LOG_FILE


#check for input from user 
if test -z "$1"  ; then
  echo "INFO: No fps provided ,Choose default 30 fps "  >> $LOG_FILE
  #change to input from Code
  fps=30
else
  fps="$1" # use the fps passed from C code 
fi

echo "value of fps" $fps >> $LOG_FILE
#use ffmpeg to convert to video 
ffmpeg -r $fps -pattern_type glob -i '*.ppm' -c:v libx264 $VIDEO_FILE

echo "INFO: Conversion complete using "  $fps " fps "  >> $LOG_FILE
