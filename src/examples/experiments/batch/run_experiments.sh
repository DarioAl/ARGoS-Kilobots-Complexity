#!/bin/bash

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

areas="25 50 75 100 125 150 175 200 225 250"
kbs="1 50"

RUNS=25

for par1 in $kbs; do
    for par2 in $areas; do
        for it in $(seq 1 $RUNS); do
            filename=`printf 'config_complexity_kbs%02d_areas%03d_seed%03d.argos' $par1 $par2 $it`
            config=`printf 'config_complexity_kbs%02d_areas%03d_seed%03d.argos' $par1 $par2 $it`
            cp $base_config $config
            sed -i "s|__SEED__|$it|g" $config
            sed -i "s|__AREAS__|$par2|g" $config
            sed -i "s|__KBS__|$par1|g" $config
            output_file=$par1"_"$par2"_"$it
            sed -i "s|__OUTPUT__|$output_file|g" $config
            echo "Running next configuration KBS $par1 AREAS $par2"
            echo "argos3 -c $1$config"
            argos3 -c './'$config
        done
    done
done

rm *.argos
