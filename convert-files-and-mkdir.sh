JPG_DIR=$1
CHAP_NUM=1

while :
do
    CHAP_NUM_4D=$(printf "%04d" ${CHAP_NUM})
    CHECK=$(ls "${JPG_DIR}" | grep "${CHAP_NUM_4D}")
    if [ ${#CHECK} -eq 0 ]; then
        break
    fi

    PAGE_NUM=0
    mkdir ${CHAP_NUM_4D}

    while read file; do
        convert "${JPG_DIR}"/"${file}" -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -remap hald:4 -depth 4 -compress none ${CHAP_NUM_4D}/$(printf "%03d" ${PAGE_NUM}).tif
        echo $((PAGE_NUM++))
    done < <(ls -A -1 "${JPG_DIR}" | grep "${CHAP_NUM_4D}")

    echo "NEXT_CHAP"
    echo $((CHAP_NUM++))
done
