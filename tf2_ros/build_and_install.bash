# remove old package build info

cd ..
rm ros-melodic-*.ddeb
rm ros-melodic-*.deb

cd tf2_ros || exit
rm -rf debian
rm -rf obj-x64_64-linux-gnu

bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro melodic
fakeroot debian/rules binary

cd ..
sudo dpkg -i ros-melodic-tf2-ros_0.6.7-0bionic_amd64.deb