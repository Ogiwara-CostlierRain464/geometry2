READ_T=1
WRITE_T=67
V=67
READ_LEN=67
WRITE_LEN=1
LOOP_SEC=60
F=10

OUTPUT_DIR=/tmp/tf-car/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png

mkdir -p $OUTPUT_DIR

for F in 10 100 1000; do
../cmake-build-release/devel/lib/tf2/car_check --vehicle=$V --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --frequency=$F --loop_sec=$LOOP_SEC --read_thread=$READ_T --write_thread=$WRITE_T
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT';" car.plt