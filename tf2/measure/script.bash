T=$(nproc)
I=100
J=100
READ_RATIO=0.7
READ_LEN=16
WRITE_LEN=16
OUTPUT_DIR=/tmp/tf/$(uuidgen)
OUTPUT=$OUTPUT_DIR/data.dat
OUTPUT_PLOT=$OUTPUT_DIR/plot.png
ONLY=0
STAT=true
FREQUENCY=600

mkdir -p $OUTPUT_DIR

# Thread, Joint, Iter, Read_Ratio, Read_Len, Write_len
VAR=1
for T in $(seq 14 14 224); do
#VAR=2
#for J in 10 100 1000 10000; do
#VAR=3
#for I in 10 100 1000 10000 100000; do
#VAR=4
#for READ_RATIO in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
#VAR=5
#for READ_LEN in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
#VAR=6
#for WRITE_LEN in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY --stat=$STAT --frequency=$FREQUENCY
done

#VAR=2
#for J in 10 100 1000 10000; do
#../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY
#done

#VAR=3
#for I in 10 100 1000 10000 100000; do
#../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY
#done

#VAR=4
#for READ_RATIO in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
#../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY
#done

#VAR=5
#for READ_LEN in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
#../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY
#done

#VAR=6
#for WRITE_LEN in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
#../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT --only=$ONLY
#done

gnuplot -e "data='$OUTPUT'; filename='$OUTPUT_PLOT'; var=$VAR" plot.plg