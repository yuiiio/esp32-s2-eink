
NUM=0
if [ -n $3 ]; then
    NUM=$3
fi

while read file; do
    convert "$1"/"${file}" -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -remap hald:4 -depth 4 -compress none $(printf "%03d" ${NUM}).tif
    echo $((NUM++))
done < <(ls -A -1 "$1")
