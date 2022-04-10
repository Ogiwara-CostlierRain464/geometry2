# 0: All, 1: Only TF-Par, 2: Only TF-2PL, 3: except old, 4: Only old , 5: except TF-Par, 6: Only TF-Silo
ONLY=0

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/speed_check --thread=$T --joint=1000000 --read_ratio=0.5 --read_len=16 --write_len=16 --output="/tmp/fig7,9.dat" --only=$ONLY --frequency=0 --loop_sec=60 --opposite_write_direction=true --make_read_stat=false
done