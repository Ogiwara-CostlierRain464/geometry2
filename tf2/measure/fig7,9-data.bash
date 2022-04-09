for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/speed_check --thread=$T --joint=1000000 --read_ratio=0.5 --read_len=16 --write_len=16 --output="/tmp/fig7,9.dat" --only=0 --frequency=0 --loop_sec=60 --opposite_write_direction=true --make_read_stat=false
done