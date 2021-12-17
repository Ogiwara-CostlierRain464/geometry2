T=$(nproc)
I=1000
J=200
READ_RATIO=0.7
READ_LEN=1.0
WRITE_LEN=1.0
OUTPUT=/tmp/tf-$(uuidgen).dat

for T in $(seq 1 "$(nproc)"); do
../cmake-build-release/devel/lib/tf2/speed_check --thread=$T --joint=$J --iter=$I --read_ratio=$READ_RATIO --read_len=$READ_LEN --write_len=$WRITE_LEN --output=$OUTPUT
done

gnuplot -e "filename='$OUTPUT'" plot.plg