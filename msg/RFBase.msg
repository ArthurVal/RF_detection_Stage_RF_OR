#Identifier of the angle data (phi = true or theta = false)
bool angleID

#Indicate if the data is enable
bool enable

#Size of the data
uint32 sizeData

#Datas : Angle & Intensity detected
float32[] angle
float32[] intensity

#Datas : Specific data (points detected with coordinates in RF & camera space)
uint32 numberPointDetected 

#RF space coordinate
float32[] anglePointRF
float32[] distancePointRF 	 
