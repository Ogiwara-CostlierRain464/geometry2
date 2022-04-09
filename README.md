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
source src/geometry2/tf2/measure/fig7,9-data.bash  # Make data for fig 7,9.
source src/geometry2/tf2/measure/fig8,10-data.bash  # Make data for fig 8,10.
source src/geometry2/tf2/measure/fig12-14-data.bash  # Make data for fig 12-14.

# With your computer which has GUI and installed gnuplot
source src/geometry2/tf2/measure/fig5-6-plot.bash # Make fig5.pdf and fig6.pdf
source src/geometry2/tf2/measure/fig7,9-plot.bash # Make fig7.pdf and fig9.pdf
source src/geometry2/tf2/measure/fig8,10-plot.bash # Make fig8.pdf and fig10.pdf
source src/geometry2/tf2/measure/fig12-14-plot.bash # Make fig12.pdf ~ fig14.pdf
```

