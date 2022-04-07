J=1000000
READ_RATIO=1
READ_LEN=16
WRITE_LEN=16
FREQUENCY=0
LOOP_SEC=60
OPPOSITE_WRITE=true
MAKE_READ_STAT=false

OUTPUT_DIR=/tmp/tf/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
# 0: All, 1: Only TF-Par, 2: Only TF-2PL, 3: except old, 4: Only old , 5: except TF-Par, 6: Only TF-Silo
ONLY=0
# throughput = 7, latency = 11, delay = 14
OUT_OFFSET=7

mkdir -p $OUTPUT_DIR


for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/speed_check --thread=$T --joint=$J --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY --frequency=$FREQUENCY --loop_sec=$LOOP_SEC --opposite_write_direction=$OPPOSITE_WRITE --make_read_stat=$MAKE_READ_STAT
done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/opposite-throughput.pdf'; var=1; out_offset=7; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/opposite-read-latency.pdf'; var=1; out_offset=11; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/opposite-read-latency2.pdf'; var=1; out_offset=11; only=3" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/opposite-delay.pdf'; var=1; out_offset=14; only=$ONLY" plot.plg
gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_DIR/opposite-abort.pdf'; var=1; out_offset=10; only=$ONLY" plot.plg
