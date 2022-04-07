you have to reinstall tf2, tf2_py, tf2_ros, rviz, and tf

be sure to recompile dirty include files with -MMD -MP flag!!!


### Reproduce work with Docker

```bash
git clone https://github.com/Ogiwara-CostlierRain464/geometry2; cd geometry2
docker build -t geometry2 .  # to run with x86_64 architecture
docker run -it  geometry2    # to run with x86_64 architecture

# inside of the container
source devel/setup.bash  # source ROS workspace overlay
source src/geometry2/tf2/measure/script.bash  # launch script for demo
```