OUTPUT_DIR=/tmp/tf-clock/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for T in $(seq 14 14 $(nproc)); do
  ../cmake-build-release/devel/lib/tf2/clock_check --thread=$T --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" clock.plt

