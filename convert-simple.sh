# 1, 2, 3.xxx => 001, 002, 003.tif

CHAP_DIR=$1

PAGE_NUM=1

while read file; do
    convert "${CHAP_DIR}"/"${file}" -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -unsharp 0x1.0 -clahe 20x20%+128+3 -dither FloydSteinberg -depth 2 -compress none $(printf "%03d" ${PAGE_NUM}).tif
    echo $((PAGE_NUM++))
done < <(ls -1v "${CHAP_DIR}")
