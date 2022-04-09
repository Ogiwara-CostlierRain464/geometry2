you have to reinstall tf2, tf2_py, tf2_ros, rviz, and tf

be sure to recompile dirty include files with -MMD -MP flag!!!


### Reproduce work with Docker

```bash
git clone --recursive https://github.com/Ogiwara-CostlierRain464/geometry2; cd geometry2
docker build -t geometry2 .  # to run with x86_64 architecture
docker run -it  geometry2    # to run with x86_64 architecture

# inside of the container
source devel/setup.bash  # source ROS workspace overlay
source src/geometry2/tf2/measure/fig5-6-data.bash  # Make data for fig 5,6.

# With your computer which has GUI and installed gnuplot
source src/geometry2/tf2/measure/fig5-6-plot.bash # Make fig5.pdf and fig6.pdf
```

