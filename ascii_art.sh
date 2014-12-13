#!/bin/bash

# files saved to /tmp are automatically removed by the system
tempfile="/tmp/ascii-art-out.txt"

helpstr="\nAscii Art Generator by AVBotz\n\nThis software uses figlet, which is licensed under the new BSD license\nand was created by Glenn Chappell and Ian Chai\n\nThis program does not come with any explicit or implied warranty. AVBotz is not\nresponsible for any damage to your system inflicted by this program\n\n-h|--help\tDisplays this help message\n-a|--all\tTakes all .hpp and .cpp files in src/ as input\n-r|--replace\tReplaces existing ascii art\n-d|--delete\tDeletes existing ascii art\n-s|--smart\tAutomatically detects ascii art and replaces it\n\nNote:--delete and --smart in conjunction deletes detected ascii art\n\nExamples:\n./ascii_art my_source.cpp\tAdds ascii art to src/my_source.cpp\n./ascii_art -a -d\t\tdeletes ascii art in all source files in src/\n"

commentopen="/*"
commentclose="*/"

ALL=false
REMOVE=false
DELETE=false
SMART=false

if [ $# == 0 ]; then
	echo -e $helpstr
	exit 0
fi

while test $# -gt 0; do
        case "$1" in
		-h|--help)
			echo -e $helpstr
			exit 0
			;;
		-a|--all)
			ALL=true
			shift
			;;
		-r|--replace)
			REMOVE=true
			shift
			;;
		-d|--delete)
			REMOVE=true
			DELETE=true
			shift
			;;
		-s|--smart)
			SMART=true
			shift
			;;
		*)
			break
			;;
	esac
done

cd src

FILES="$@"

if $ALL; then
	FILES="$FILES *.cpp *.hpp"
fi

for file in $FILES
do
	touch $tempfile
	if [ $DELETE == false ]; then
		# generate the ascii art and save it
		echo "$commentopen" >> $tempfile
		figlet -w 10000 $file >> $tempfile
		echo "$commentclose" >> $tempfile
	fi
	
	if $SMART; then
		
		# see if there is old ascii art to be removed
		grep "(_)_" $file > /dev/null # identifies the period in figlet's ascii art
		if [ $? == 0 ]; then # if the output of grep is 0, meaning something matched
			printf "Update: "
			# remove the first 8 lines to get rid of the comment
			tail -n+9 $file >> $tempfile
		else
			printf "Add:    "
			cat $file >> $tempfile
		fi
	else
		if $REMOVE; then
			tail -n+9 $file >> $tempfile
		else
			cat $file >> $tempfile
		fi
	fi

	# replace the old source file with the temp file
	cp $tempfile $file
	rm $tempfile
done
