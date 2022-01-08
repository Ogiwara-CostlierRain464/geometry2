I=10000

OUTPUT_DIR=/tmp/tf-tbb/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for T in 1 2 3 4; do
  ../cmake-build-release/devel/lib/tf2/tbb_check --thread=$T --iter=$I --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" tbb.plt

