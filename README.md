# HackTheSky2016
HackTheSky 2016 in San Diego : Team BIGMAC
### Description
We implemented HMAC and AES128 for all sent and received messages for the HackTheSky Drone Swarm.

### Setup
 1. Put keys.txt in /opt/keys.txt
 1. Put acs_socket.py in acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/
   - This will add our HMAC function hooks into the main socket send() and recv() functions for ALL "Messages", regardless of message type.
 1. Put hmacs_keys.py in acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/
   - This is our custom code to implement HMAC and AES128.
