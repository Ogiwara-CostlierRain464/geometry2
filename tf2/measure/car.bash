T=$(nproc)
V=67
READ_RATIO=0.5
READ_LEN=67
WRITE_LEN=1
LOOP_SEC=60
FREQUENCY=10

OUTPUT_DIR=/tmp/tf-car/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
../cmake-build-release/devel/lib/tf2/car_check --thread=$T --vehicle=$V --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --frequency=$FREQUENCY --loop_sec=$LOOP_SEC
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" car.plt