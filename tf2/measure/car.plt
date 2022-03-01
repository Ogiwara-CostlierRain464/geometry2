set terminal pdf size 4in,3in font "arial,16"
set output filename
set key left top font "arial,14"
set size ratio 0.75

set linetype 1 lc rgb "dark-magenta" lw 2 pt 1 ps 1.5
set linetype 2 lc rgb "#009e73" lw 2 pt 2 ps 1.5
set linetype 3 lc rgb "#56b4e9" lw 2 pt 3 ps 1.5
set linetype 4 lc rgb "#e69f00" lw 2 pt 4 ps 1.5

set xlabel "thread"
set ylabel "read latency (ms)"

plot data using 1:9 w lp ls 2 title "{/Times-Italic LT}", data using 1:10 w lp ls 3 title "{/Times-Italic LT-2PL}", data using 1:13 w lp ls 4 title "{/Times-Italic LT-Silo}"
