#!/bin/bash



function print_help() {
  echo "Usage: ./generate_models.sh ANYMAL_NAME ANYMAL_SETUP"
  echo "ANYMAL_NAME: can be starleth or anymal"
  echo "ANYMAL_SETUP: can be standard, inspection or minimal"
  
  exit
}



# Check input arguments
if [ "$1" == "" ]; then 
  echo "ERROR: ANYMAL_NAME is emtpy!"
  print_help
fi

if ( [[ "$1" != "anymal" ]] && [[ "$1" != "starleth" ]] ); then 
  echo "ERROR: ANYMAL_NAME must be 'anymal' or 'starleth' !"
  print_help
fi

if [ "$2" == "" ]; then 
  echo "ERROR: ANYMAL_SETUP is emtpy!"
  print_help
fi

# if ( [[ "$2" != "minimal" ]] && [[ "$2" != "inspection" ]] && [[ "$2" != "standard" ]] ); then
#   echo "ERROR: ANYMAL_SETUP must be 'minimal', 'standard' or 'inspection' !"
#   print_help
# fi



# Generate urdf and store it in a file
echo "Generating $1_$2.urdf in $(rospack find anymal_model)/resources ..."
rosrun xacro xacro.py -o $(rospack find anymal_model)/resources/$1_$2.urdf $(rospack find $1_description)/urdf/$1_$2.urdf.xacro simulation:=true fixed:=false perfect_sensors:=true hooks:=false mesh:=true


echo "Done!"