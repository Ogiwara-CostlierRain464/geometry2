SNAKE_NUM=10000
SNAKE_JOINS=100
READ_RATIO=0.5
READ_LEN=16
WRITE_LEN=16
ONLY=0
FREQUENCY=0
LOOP_SEC=60
OPPOSITE_WRITE=true
MAKE_READ_STAT=false

OUTPUT_DIR=/tmp/snake/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
mkdir -p $OUTPUT_DIR

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/many_snakes --thread=$T --snake_num=$SNAKE_NUM --snake_joints=$SNAKE_JOINS --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY --frequency=$FREQUENCY --loop_sec=$LOOP_SEC --opposite_write_direction=$OPPOSITE_WRITE --make_read_stat=$MAKE_READ_STAT
done

