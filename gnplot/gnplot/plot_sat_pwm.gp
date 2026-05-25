set datafile separator comma

set terminal pngcairo size 1200,700 enhanced font "Arial,12"
set output "plot_gnuplot.png"

set title "Grafik Respon Metode SAT-PWM"
set xlabel "Waktu (ms)"
set ylabel "Nilai Pembacaan / Persentase"

set grid
set key outside right
set yrange [0:110]

set arrow 1 from graph 0, first 45 to graph 1, first 45 nohead dashtype 2 linewidth 2 linecolor rgb "orange"
set label 1 "Batas ALERT 45 C" at graph 0.03, first 48 textcolor rgb "orange"

plot "data_sat_pwm.csv" every ::1 using 1:2 with linespoints linewidth 2 pointtype 7 linecolor rgb "blue" title "Suhu LM35 (C)", \
     "data_sat_pwm.csv" every ::1 using 1:4 with linespoints linewidth 2 pointtype 5 linecolor rgb "green" title "Risk (%)", \
     "data_sat_pwm.csv" every ::1 using 1:5 with linespoints linewidth 2 pointtype 9 linecolor rgb "red" title "PWM (%)"