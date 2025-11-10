JPG_DIR=$1
CHAP_NUM=1

CHILD_DIR_OFFSET=$3 # should default 0

while :
do
    CHAP_NUM_4D=$(printf "%04d" ${CHAP_NUM})
    CHECK=$(ls "${JPG_DIR}" | grep "${CHAP_NUM_4D}")
    if [ ${#CHECK} -eq 0 ]; then
        break
    fi

    PAGE_NUM=0

    CHILD_DIR_NUM=$(( $CHAP_NUM+$CHILD_DIR_OFFSET ))
    CHILD_DIR=$(printf "%04d" ${CHILD_DIR_NUM})
    mkdir ${CHILD_DIR}

    while read file; do
        convert "${JPG_DIR}"/"${file}" -alpha off -rotate $2 -resize 1448x1072 -gravity center -extent 1448x1072 -colorspace gray -unsharp 0x1.0 -clahe 20x20%+128+3 -dither FloydSteinberg -depth 2 -compress none ${CHILD_DIR}/$(printf "%03d" ${PAGE_NUM}).tif
        echo $((PAGE_NUM++))
    done < <(ls -A -1 "${JPG_DIR}" | grep "${CHAP_NUM_4D}")

    echo "NEXT_CHAP"
    echo $((CHAP_NUM++))
done
