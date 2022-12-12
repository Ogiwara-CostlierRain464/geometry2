if [ "$1" = "data" ]; then

  for T in $(seq 28 28 224); do
      ../../../../devel/lib/tf2/speed_check --thread=$T \
      --joint=1000000 \
      --read_ratio=0.5 \
      --read_len=16 \
      --write_len=16 \
      --output="/tmp/fig-no-stat.dat" \
      --only="111111" \
      --frequency=0 \
      --loop_sec=60 \
      --opposite_write_direction=true \
      --make_read_stat=false
  done
fi

if [ "$1" = "plot" ]; then
  gnuplot -e "data='/tmp/fig-no-stat.dat'; filename='/tmp/fig8.pdf'; type=0; only=1" print.plt
  gnuplot -e "data='/tmp/fig-no-stat.dat'; filename='/tmp/fig10.pdf'; type=1; only=1" print.plt
  gnuplot -e "data='/tmp/fig-no-stat.dat'; filename='/tmp/fig11.pdf'; type=0; only=2" print.plt
  gnuplot -e "data='/tmp/fig-no-stat.dat'; filename='/tmp/fig13.pdf'; type=1; only=2" print.plt
fi
