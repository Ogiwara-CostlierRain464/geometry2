OUTPUT_DIR=/tmp/tf-rw/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
  ../cmake-build-release/devel/lib/tf2/rw_lock_check --thread=$T --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" rw.plg