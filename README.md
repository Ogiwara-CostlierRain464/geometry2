## About
Implementation of `Transactional Transform Library on ROS`

## For IROS paper reviewer

### Reproduction with Docker

```bash
# Step1: clone and start container
git clone --recursive https://github.com/Ogiwara-CostlierRain464/geometry2; cd geometry2
docker build -t geometry2 .  # To run with x86_64 architecture. If you encounter an error, see FAQ1.
docker run -it  geometry2    # To run with x86_64 architecture

# Step2: Inside of the container, make data for fig 5~14.
source devel/setup.bash  # source ROS workspace overlay
source src/geometry2/tf2/measure/fig5-6-data.bash  # Make data for fig 5,6.
source src/geometry2/tf2/measure/fig7,9-data.bash  # Make data for fig 7,9.
source src/geometry2/tf2/measure/fig8,10-data.bash  # Make data for fig 8,10.
source src/geometry2/tf2/measure/fig12-14-data.bash  # Make data for fig 12-14.

# Step3: Make plot with your computer which has GUI and has installed gnuplot.

# Copy /tmp/fig5-6.dat /tmp/fig7,9.dat /tmp/fig8,10.dat
# /tmp/fig12.dat /tmp/fig13.dat /tmp/fig14.dat 
# to your computer

source src/geometry2/tf2/measure/fig5-6-plot.bash # Make fig5.pdf and fig6.pdf
source src/geometry2/tf2/measure/fig7,9-plot.bash # Make fig7.pdf and fig9.pdf
source src/geometry2/tf2/measure/fig8,10-plot.bash # Make fig8.pdf and fig10.pdf
source src/geometry2/tf2/measure/fig12-14-plot.bash # Make fig12.pdf ~ fig14.pdf
```

### FAQ
Q1. Docker says `Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock`

A1. Run following command
```bash
sudo gpasswd -a $(whoami) docker
sudo chgrp docker /var/run/docker.sock
sudo chgrp docker /var/run/docker.sock
exit # Logout, and try again.
```

Q2. I want to evaluate separately `TF`, `TF-Par`, `TF-2PL`, and `TF-Silo`.

A2. Change `ONLY` variable in each *-data.bash files.

Q3. I want to try more detailed workloads.

A3. See `src/geometry2/tf2/measure/script.bash` and `src/geometry2/tf2/measure/car.bash` 

Q4. I want to use this package in read workload.

A4. You have to recompile tf2_py, tf2_ros, rviz, and tf with `-MMD -MP` flag and reinstall them,