OUTPUT_DIR=/tmp/tf-wait/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
  ../cmake-build-release/devel/lib/tf2/wait_check  --thread=$T --output=$OUTPUT --loop_sec=60
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" wait.plg