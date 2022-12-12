if [ "$1" = "data" ]; then
  for T in $(seq 28 28 224); do
      ../../../../devel/lib/tf2/speed_check --thread=$T \
      --joint=1000000 \
      --read_ratio=0.5 \
      --read_len=16 \
      --write_len=16 \
      --output="/tmp/fig-stat.dat" \
      --only="111101" \
      --frequency=0 \
      --loop_sec=60 \
      --opposite_write_direction=true \
      --make_read_stat=true
  done
fi

if [ "$1" = "plot" ]; then
  gnuplot -e "data='/tmp/fig-stat.dat'; filename='/tmp/fig9.pdf'; type=2; only=3" print.plt
  gnuplot -e "data='/tmp/fig-stat.dat'; filename='/tmp/fig12.pdf'; type=2; only=4" print.plt
  gnuplot -e "data='/tmp/fig-stat.dat'; filename='/tmp/fig14.pdf'; type=3; only=0" print.plt
fi
