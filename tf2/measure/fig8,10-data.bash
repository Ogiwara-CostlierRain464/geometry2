# Bit representation of enabled methods
# Silo, 2PL, Par, and Old from left to right bit.
# e.g., "1000" means "execute only Silo."
ONLY=1111

for T in $(seq 28 28 224); do
/root/ros_ws/devel/lib/tf2/speed_check --thread=$T --joint=1000000 --read_ratio=0.5 --read_len=16 --write_len=16 --output="/tmp/fig5-6.dat" --only=$ONLY --frequency=0 --loop_sec=60 --opposite_write_direction=true --make_read_stat=true
done