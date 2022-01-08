set terminal png
set output filename
set ylabel "time(lower is better)"

#set logscale x

plot data using 1:3 w lp title "tbb vec"
