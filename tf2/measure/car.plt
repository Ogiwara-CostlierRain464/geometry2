set terminal pdf size 4in,3in
set output filename
set key left top
set size ratio 0.75

set xlabel "thread"
set ylabel "read latency (ms)"

plot data using 1:9 w lp title "old", data using 1:10 w lp title "latest"
