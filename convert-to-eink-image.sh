# thanks for https://middleriver.chagasi.com/electronics/epd_photo/
convert $1 -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -remap hald:4 -depth 4 -compress none output.tif
convert $1 -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -remap hald:4 -depth 4 -compress none output.png # for preview
