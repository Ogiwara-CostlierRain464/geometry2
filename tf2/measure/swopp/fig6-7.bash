# data or plot

if [ "$1" = "data" ]; then

  for T in $(seq 28 28 224); do
      ../../../../devel/lib/tf2/speed_check --thread=$T \
      --joint=1000000 \
      --read_ratio=1 \
      --read_len=16 \
      --write_len=16 \
      --output="/tmp/fig6-7.dat" \
      --only="111111" \
      --frequency=0 \
      --loop_sec=60 \
      --opposite_write_direction=true \
      --make_read_stat=false
  done
fi

if [ "$1" = "plot" ]; then
  gnuplot -e "data='/tmp/fig6-7.dat'; filename='/tmp/fig6.pdf'; type=0; only=0" print.plt
  gnuplot -e "data='/tmp/fig6-7.dat'; filename='/tmp/fig7.pdf'; type=1; only=0" print.plt
fi
