V=67            # Number of vehicles
READ_RATIO=0.5  # Ratio of read only threads
READ_LEN=67     # Number of joints which read only thread reads
WRITE_LEN=1     # Number of joints which write only thread writes
FREQUENCY=150   # Frequency to access TF tree
LOOP_SEC=60     # Loop sec to measure performance
INSERT_SPAN=1   # Span to insert new vehicle
# 0: All, 1: Only old, 2: Only TF-2PL, 3: Only TF-Silo
ONLY=0

OUTPUT_DIR=/tmp/tf-car/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/car_check --thread=$T --vehicle=$V --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --frequency=$FREQUENCY --loop_sec=$LOOP_SEC --insert_span=$INSERT_SPAN --only=$ONLY
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/read-latency.pdf';" car.plt