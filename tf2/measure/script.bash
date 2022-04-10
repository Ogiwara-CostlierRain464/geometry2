J=1000000             # Number of joints
READ_RATIO=1          # Ratio of read only threads
READ_LEN=16           # Number of joints which read only thread reads
WRITE_LEN=16          # Number of joints which write only thread writes
FREQUENCY=0           # Frequency to access TF tree
LOOP_SEC=60           # Loop sec to measure performance
OPPOSITE_WRITE=true   # When true: write from parent to child. otherwise, write from child to parent. This may effect to abort ratio.
MAKE_READ_STAT=false  # When true: make statistics while reading. To enhance performance, turned off.
# 0: All, 1: Only TF-Par, 2: Only TF-2PL, 3: except old, 4: Only old , 5: except TF-Par, 6: Only TF-Silo
ONLY=0

OUTPUT_DIR=/tmp/tf/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat # Location of measure result data
mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/speed_check --thread=$T --joint=$J --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY --frequency=$FREQUENCY --loop_sec=$LOOP_SEC --opposite_write_direction=$OPPOSITE_WRITE --make_read_stat=$MAKE_READ_STAT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/throughput.pdf'; var=1; out_offset=7; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/read-latency.pdf'; var=1; out_offset=11; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/read-latency2.pdf'; var=1; out_offset=11; only=3" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/delay.pdf'; var=1; out_offset=14; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/abort.pdf'; var=1; out_offset=10; only=$ONLY" plot.plg
