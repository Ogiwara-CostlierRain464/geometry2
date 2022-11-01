OUTPUT_DIR=/tmp/tf-drone/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/drone_check --thread=$T --output=$OUTPUT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/read-latency.pdf'; out_offset=3" drone.plt