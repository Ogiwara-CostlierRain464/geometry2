# remove old package build info
rm ros-melodic-tf2-*.ddeb
rm ros-melodic-tf2*.deb
cd tf2 || exit
rm -rf debian
rm -rf obj-x64_64-linux-gnu

bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro melodic
fakeroot debian/rules binary

cd ..
sudo dpkg -i ros-melodic-tf2_0.6.7-0bionic_amd64.deb