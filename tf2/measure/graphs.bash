gnuplot -e "data='./opposite.dat'; filename='./opposite-throughput-old.pdf'; var=1; out_offset=7; only=0" plot.plg
gnuplot -e "data='./opposite.dat'; filename='./opposite-read-latency-old.pdf'; var=1; out_offset=11; only=0" plot.plg
gnuplot -e "data='./opposite.dat'; filename='./opposite-read-latency2-old.pdf'; var=1; out_offset=11; only=3" plot.plg
gnuplot -e "data='./opposite.dat'; filename='./opposite-delay-old.pdf'; var=1; out_offset=14; only=0" plot.plg
gnuplot -e "data='./opposite.dat'; filename='./opposite-abort-old.pdf'; var=1; out_offset=10; only=0" plot.plg

# For IROS paper, remove fonts
gs -o opposite-throughput.pdf -dNoOutputFonts -sDEVICE=pdfwrite opposite-throughput-old.pdf
gs -o opposite-read-latency.pdf -dNoOutputFonts -sDEVICE=pdfwrite opposite-read-latency-old.pdf
gs -o opposite-read-latency2.pdf -dNoOutputFonts -sDEVICE=pdfwrite opposite-read-latency2-old.pdf
gs -o opposite-delay.pdf -dNoOutputFonts -sDEVICE=pdfwrite opposite-delay-old.pdf
gs -o opposite-abort.pdf -dNoOutputFonts -sDEVICE=pdfwrite opposite-abort-old.pdf
