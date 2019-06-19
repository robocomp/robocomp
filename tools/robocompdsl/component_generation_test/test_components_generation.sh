#!/bin/bash
declare -A results

valid=0
generated=0
compiled=0
comp_failed=0
gen_failed=0

previous_dir=$(pwd)
cd ~/robocomp/tools/robocompdsl/component_generation_test
for dir in $(ls -d */)
do
	echo "Entering dir $dir"
	cd $dir
	ls | grep -v ".smdsl" | grep -v ".cdsl" | grep -v ".log" | xargs rm -r > /dev/null 2>&1
	cdsl_file=$(ls *.cdsl) > /dev/null 2>&1
	if [ ! -z "$cdsl_file" ]
	then
		valid=$((valid+1))
		robocompdsl $cdsl_file > /dev/null 2>&1
		echo "Executing robocompdsl for $dir $cdsl_file ... WAIT!"
		robocompdsl $cdsl_file . > generation_output.log 2>&1
		if [ $? -eq 0 ]; then
		    echo "$dir $cdsl_file generation OK"
		    generated=$((generated+1))
		    echo "Executing cmake for $dir ... WAIT!"
            cmake . > cmake_output.log 2>&1
            echo "Executing make for $dir ... WAIT!"
            make > make_output.log 2>&1
            if [ $? -eq 0 ]; then
                echo "$dir compilation OK"
                compiled=$((compiled+1))
                results["$dir"]="True"
            else
                echo "$dir compilation FAILED"
                comp_failed=$((comp_failed+1))
                results["$dir"]="False"
		fi
		else
		    echo "$dir $cdsl_file generation FAILED"
		    gen_failed=$((gen_failed+1))
		fi
	else
		echo "No cdsl file"
	fi
	ls | grep -v ".smdsl" | grep -v ".cdsl" | grep -v ".log" | xargs rm -r > /dev/null 2>&1
	cd ..
done

cd $previous_dir

echo "$valid components found with cdsl"
echo "$generated components generated OK ($gen_failed failed)"
echo "$compiled components compiled OK ($comp_failed failed)"

for key in ${!results[@]}; do echo "    $key have compile? ${results[$key]}"; done
