#!/bin/bash

self="${0#./}"
base="${self%/*}"
current=$(pwd)

#Get script dir
if [ "$base" = "$self" ] ; then
	cd "$current"
else
	cd "$current/$base"
fi ;

echo "Script dir is:" $(pwd)
	
#Create LUT
	
i=0
j=0
out=""
out2=""

for z in $(cat $(pwd)/Kinectvalues)
do
	
	v=$(echo $z | cut -d"," -f1)
	
	#Depth to step LUT
	for ((; j < v; j++)) ; do
		out="${out} -1,"
	done
	out="${out} $i,"
	j=$[$j+1]

	
	#Step to Depth LUT
	out2="${out2} $v, "
	
	
	i=$[$i+1]
done

#Ending
out="${out} -2"
out2="${out2} -2"


#Write header
echo -e "const static short kinect_depth_to_step_LUT[] = { $out };" >> ../include/kinectStepLUT

echo -e "const static short kinect_depths_count = $j;" >> ../include/kinectStepLUT
	
echo -e "const static short kinect_step_to_depth_LUT[] = { $out2 };" >> ../include/kinectStepLUT

echo -e "const static short kinect_steps_count = $i;" >> ../include/kinectStepLUT


indent ../include/kinectStepLUT 
rm ../include/kinectStepLUT~