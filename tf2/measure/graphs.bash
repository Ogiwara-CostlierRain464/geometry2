for name in opposite direct; do
  gnuplot -e "data='./${name}.dat'; filename='./${name}-throughput.png'; var=1; out_offset=7; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-read-throughput.png'; var=1; out_offset=21; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-write-throughput.png'; var=1; out_offset=24; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-write-throughput2.png'; var=1; out_offset=24; only=5" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-read-latency.png'; var=1; out_offset=11; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-read-latency2.png'; var=1; out_offset=11; only=3" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-write-latency.png'; var=1; out_offset=18; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-write-latency2.png'; var=1; out_offset=18; only=3" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-delay.png'; var=1; out_offset=14; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-delay2.png'; var=1; out_offset=14; only=3" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-stddiv.png'; var=1; out_offset=17; only=0" plot.plg
  gnuplot -e "data='./${name}.dat'; filename='./${name}-abort.png'; var=1; out_offset=10; only=0" plot.plg
done