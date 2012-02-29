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

for z in $(cat $(pwd)/Kinectvalues)
do
	
	v=$(echo $z | cut -d"," -f1)
		
	for ((; j < v; j++)) ; do
		out="${out} -1,"
		if [ "$(($j%10))" == "0" ] ; then
			out="${out} \n"
		fi
	done
	out="${out} $i,"
	j=$[$j+1]
	if [ "$(($j%10))" == "0" ] ; then
		out="${out} \n"
	fi
	i=$[$i+1]
done

#Ending
out="${out} 0"


#Write header
echo -e "short kinect_value_LUT[] = { $out };" > ../includes/kinectStepLUT.h
echo -e "int kinect_value_size = $j;" >> ../includes/kinectStepLUT.h