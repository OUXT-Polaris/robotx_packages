gource \
    -s .06 \
    -1920x1280 \
    --auto-skip-seconds .1 \
    --multi-sampling \
    --stop-at-end \
    --key \
    --highlight-users \
    --hide mouse,progress \
    --file-idle-time 0 \
    --max-files 0  \
    --background-colour 000000 \
    --font-size 22 \
    --title "robotx-packages_development_log" \
    --output-ppm-stream - \
    --output-framerate 30 \
    | avconv -y -r 30 -f image2pipe -vcodec ppm -i - -b 65536K movie.mp4