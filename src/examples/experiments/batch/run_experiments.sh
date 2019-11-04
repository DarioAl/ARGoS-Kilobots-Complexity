#!/bin/bash
#!

if [ "$#" -ne 2 ]; then
    echo "Usage: run_experiments.sh <base_config_dir> <base_config_file>"
    exit 11
fi

wdir=`pwd`
base_config=$1$2
if [ ! -e $base_config ]; then
    base_config=$wdir/$1/$2
    if [ ! -e $base_config ]; then
        echo "Error: missing configuration file '$base_config'" 1>&2
        exit 1
    fi
fi

base_dir=`dirname $base_config`

echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

areas="25 50 75 100 125 150 175 200 250"

for par1 in $areas; do
    filename=`printf 'config_complexity_areas%03d.argos' $par1`
    config=`printf 'config_complexity_areas%03d.argos' $par1`
    cp $base_config $config
    sed -i "s|__AREAS__|$par1|g" $config
    sed -i "s|__OUTPUT__|$par1|g" $config
    echo "Running next configuration AREAS $par1"
    echo "argos3 -c $1$config"
    argos3 -c './'$config
done
