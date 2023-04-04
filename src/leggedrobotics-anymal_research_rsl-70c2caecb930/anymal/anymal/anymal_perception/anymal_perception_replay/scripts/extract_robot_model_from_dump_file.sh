#! /bin/bash
#
# Credits: Robotic Systems Lab, ETH Zürich
#
# Description: Extracts robot model description from yaml file containing rosparam dump, obtained with 'rosparam dump'.
# Usage: rosrun anymal_perception_replay extract_robot_model_from_dump_file.sh path_dump_file path_output_file string_start(optional)

if [  $# -le 1 ] 
then
	echo "Usage:"
	echo "rosrun anymal_perception_replay extract_robot_model_from_dump_file.sh path_dump_file path_output_file string_start(optional)"
	exit 1
fi

param_dump_file="$1"
output_file="$2"
touch ${output_file}

# Extract robot description
# Note: this command extracts all lines between (and including the lines with)
#       the first occurance of '$string_start description' and '/robot'
#       and thus effectively exporting the compiled quadruped_description robot
#       xacro parameter.
string_start="${3:-anymal_description:}"

# Check if string_start exists in the dump file, exit if not
if ! grep -q $string_start "$param_dump_file"; then
  echo "Did not find start string: '$string_start', exiting."
  exit
fi

string_stop="<\/robot>"
sed -n "/\<${string_start}/, /${string_stop}/ p" ${param_dump_file} > ${output_file}
# Basic way to ensure that robot_description has valid end (i.e. <\robot>\n\n" ), but I could not find a better way.
# A: remove text after string_stop
# B: add final part: \n\n"
sed -i "s|${string_stop}.*|${string_stop}|" ${output_file}
sed -i '$ s/$/\\n\\n"/' ${output_file}

echo "Successfully extracted robot description to file."
