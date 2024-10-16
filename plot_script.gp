set title 'Quaternion Plots'
set key box opaque
set key outside
set xlabel 'Time Step'
set ylabel 'Quaternion Component Value'
plot 'quaternion_data.dat' using 1:2 title 'Desired Quat. W' with lines lw 3 dt 3 lt rgb 'dark-gray', \
     'quaternion_data.dat' using 1:3 title 'Desired Quat. X' with lines lw 3 dt 3 lt rgb 'dark-gray', \
     'quaternion_data.dat' using 1:4 title 'Desired Quat. Y' with lines lw 3 dt 3 lt rgb 'dark-gray', \
     'quaternion_data.dat' using 1:5 title 'Desired Quat. Z' with lines lw 3 dt 3 lt rgb 'dark-gray', \
     'quaternion_data.dat' using 1:6 title 'Process Quat. W' with lines lw 3 lt rgb 'red', \
     'quaternion_data.dat' using 1:7 title 'Process Quat. X' with lines lw 3 lt rgb 'blue', \
     'quaternion_data.dat' using 1:8 title 'Process Quat. Y' with lines lw 3 lt rgb 'dark-turquoise', \
     'quaternion_data.dat' using 1:9 title 'Process Quat. Z' with lines lw 3 lt rgb 'green'