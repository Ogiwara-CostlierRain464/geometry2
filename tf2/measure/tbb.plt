set terminal png
set output filename
set xlabel "thread"
set ylabel "time(lower is better)"

#set logscale x

plot data using 1:2 w lp title "tbb vec"
