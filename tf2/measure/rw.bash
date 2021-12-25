OUTPUT_DIR=/tmp/tf-rw/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for I in 1000 2000 3000 4000 5000 6000 7000 8000 9000; do
  ../cmake-build-release/devel/lib/tf2/rw_lock_check --iter=$I --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" rw.plg