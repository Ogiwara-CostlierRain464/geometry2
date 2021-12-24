OUTPUT_DIR=/tmp/tf-rw/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for I in 10000 100000 1000000; do
  ../cmake-build-release/devel/lib/tf2/rw_lock_check --iter=$I --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" rw.plg