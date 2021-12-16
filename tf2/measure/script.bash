THREAD=$(nproc)
ITER=1000
READ_RATIO=0.7
READ_LEN=1.0
WRITE_LEN=1.0
OUTPUT=/tmp/tf-$(uuidgen).dat

for J in 10 100 1000 5000; do
../cmake-build-release/devel/lib/tf2/speed_check --thread=$THREAD --joint=$J --iter=$ITER --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT
done

gnuplot -e "filename='$OUTPUT'" plot.plg