## Allow SSH'ing for Distributed Processing

To run distributed processing, each machine needs to allow full communication with each other. This can be done by setting them each up as an SSH server:

`sudo apt install openssh-server`

Permissions also need to be given for the machines to communicate:

`sudo ufw allow ssh`

# ROS IP

Each machine needs their `ROS_HOSTNAME` and `ROS_MASTER_URI` environment variables set appropriately. Assuming each machine uses Bash, set the following environment variables (for example, in a .bashrc file):

`export ROS_HOSTNAME=<this computers IP address>`

`export ROS_MASTER_URI=http://<this computers IP address>:11311`

When a different machine on the network runs the ROS core, the `ROS_MASTER_URI` should point to the IP address of the master machine, as follows:

`export ROS_MASTER_URI=http://<host machines IP address>:11311`
