# Bit representation of enabled methods
# Silo, 2PL, Par, and Old from left to right bit.
# e.g., "1000" means "execute only Silo."
ONLY=1111

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/many_snakes --thread=$T --snake_num=10000 --snake_joints=100 --read_ratio=1 --read_len=8 --write_len=8 --output="/tmp/fig5-6.dat" --only=$ONLY --frequency=0 --loop_sec=60 --opposite_write_direction=true --make_read_stat=false
done