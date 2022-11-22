set terminal pdf size 4in,3in font "arial,18"
set output filename
set key left top font "arial,14"
set size ratio 0.75

set linetype 1 lc rgb "dark-magenta" lw 2 pt 1 ps 1.5
set linetype 2 lc rgb "#009e73" lw 2 pt 2 ps 1.5
set linetype 3 lc rgb "#56b4e9" lw 2 pt 3 ps 1.5
set linetype 4 lc rgb "#e69f00" lw 2 pt 4 ps 1.5

set xtics font "arial,15"

if(out_offset == 3){
    set ylabel "read latency (ms)" font "arial,20"
}
if(out_offset == 7){
    set ylabel "write latency (ms)" font "arial,20"
}

plot data using 1:out_offset w lp ls 1 title "TF",
     data using 1:out_offset+1 w lp ls 2 title "TF-Par",
     data using 1:out_offset+2 w lp ls 3 title "TF-2PL",
     data using 1:out_offset+3 w lp ls 4 title "TF-Silo"