set terminal pdf size 4in,3in font "arial,18"
set output filename
set key left top font "arial,14"
set size ratio 0.75

tf = "TF"
tpl = "TF-2PL"
silo = "TF-Silo"
tpl_latest = "TF-2PL-latest"
silo_latest = "TF-Silo-latest"

set linetype 1 lc rgb "dark-magenta" lw 2 pt 1 ps 1.5
set linetype 2 lc rgb "#009e73" lw 2 pt 2 ps 1.5
set linetype 3 lc rgb "#56b4e9" lw 2 pt 3 ps 1.5
set linetype 4 lc rgb "#e69f00" lw 2 pt 4 ps 1.5
set linetype 5 lc rgb "brown" lw 2 pt 5 ps 1.5

tf_throughput = 7
tf_latency = 11
tf_delay = 14

tpl_latest_throughput = 9
tpl_latest_latency = 13
tpl_latest_delay = 16
tpl_latest_abort = 10

silo_latest_throughput = 28
silo_latest_latency = 30
silo_latest_delay = 32
silo_latest_abort = 29

tpl_throughput = 34
tpl_latency = 35
tpl_delay = 37
tpl_abort = 36

silo_throughput = 38
silo_latency = 39
silo_delay = 41
silo_abort = 40

# throughtput
if(type == 0){
    set ylabel "Throughput(task/sec)" font "arial,20"
    tf_var = tf_throughput
    tpl_var = tpl_throughput
    silo_var = silo_throughput
    tpl_latest_var = tpl_latest_throughput
    silo_latest_var = silo_latest_throughput
}
# latency
if(type == 1){
    set ylabel "read latency (ms)" font "arial,20"
    tf_var = tf_latency
    tpl_var = tpl_latency
    silo_var = silo_latency
    tpl_latest_var = tpl_latest_latency
    silo_latest_var = silo_latest_latency
}
# abort ratio
if(type == 2){
    set ylabel "abort ratio(abort count/task)" font "arial,20"
    tpl_var = tpl_abort
    silo_var = silo_abort
    tpl_latest_var = tpl_latest_abort
    silo_latest_var = silo_latest_abort
}
# freshness
if(type == 3){
    set ylabel "delay (ms)" font "arial,20"
    set yrange [0:50000]
    tf_var = tf_delay
    tpl_var = tpl_delay
    silo_var = silo_delay
    tpl_latest_var = tpl_latest_delay
    silo_latest_var = silo_latest_delay
}
if(only == 1){
    plot data using 1:tf_var w lp ls 1 title tf, data using 1:tpl_var w lp ls 2 title tpl, data using 1:silo_var w lp ls 3 title silo
    exit
}
if(only == 2){
    plot data using 1:tf_var w lp ls 1 title tf, data using 1:tpl_latest_var w lp ls 4 title tpl_latest, data using 1:silo_latest_var w lp ls 5 title silo_latest
    exit
}

plot data using 1:tf_var w lp ls 1 title tf, data using 1:tpl_var w lp ls 2 title tpl, data using 1:silo_var w lp ls 3 title silo, data using 1:tpl_latest_var w lp ls 4 title tpl_latest, data using 1:silo_latest_var w lp ls 5 title silo_latest

