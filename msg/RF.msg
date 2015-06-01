Header header

#Index of the message
uint32 index

#Setup of the RF system (angle min max - points needed - acquisition time)
RFSetup rfSetup

geometry_msgs/Transform RF_CAM_Transform

#Vector of data (one for phi, one for theta)
RFBase[] rfData
