# 0: All, 1: Only old, 2: Only TF-2PL, 3: Only TF-Silo
ONLY=0

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/car_check --thread=$T --vehicle=67 --read_ratio=0.5 --read_len=67 --write_len=1 --output="/tmp/fig12.dat" --frequency=10 --loop_sec=60 --insert_span=1 --only=$ONLY
done

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/car_check --thread=$T --vehicle=67 --read_ratio=0.5 --read_len=67 --write_len=1 --output="/tmp/fig13.dat" --frequency=100 --loop_sec=60 --insert_span=1 --only=$ONLY
done

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/car_check --thread=$T --vehicle=67 --read_ratio=0.5 --read_len=67 --write_len=1 --output="/tmp/fig14.dat" --frequency=150 --loop_sec=60 --insert_span=1 --only=$ONLY
done