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
echo "/** Lookup table to convert Kinect depth value to steps **/" > ../include/kinectStepLUT.h
echo -e "const short kinect_depth_to_step_LUT[] = { $out };" >> ../include/kinectStepLUT.h
echo >> ../include/kinectStepLUT.h

echo "/** Size of lookup table for Kinect depth value to steps **/" >> ../include/kinectStepLUT.h
echo -e "const int kinect_step_to_depth_LUT_size = $j;" >> ../include/kinectStepLUT.h
echo >> ../include/kinectStepLUT.h		
echo >> ../include/kinectStepLUT.h					
	
echo "/** Lookup table to convert Kinect steps to depth **/"  >> ../include/kinectStepLUT.h
echo -e "const short kinect_step_to_depth_LUT[] = { $out2 };" >> ../include/kinectStepLUT.h
echo >> ../include/kinectStepLUT.h		

echo "/** Size of lookup table for Kinect steps to depth value **/" >> ../include/kinectStepLUT.h
echo -e "const int kinect_depth_to_step_LUT_size = $i;" >> ../include/kinectStepLUT.h


indent ../include/kinectStepLUT.h 
rm ../include/kinectStepLUT.h.orig