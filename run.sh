#! /usr/bin/env bash

#bagConvert 

dir="./bag_path/"
ls $dir | while read line
do
    file=${dir}${line}
    echo $file
    rm -rf ./*_img/ ./*_pcd/ ./wit
    annotate_bag="lane_change_"${file:0-5:1}
    echo $annotate_bag
    python generate_datasets.py
    mv ./wit/ ../SUSTechPOINTS-dev-auto-annotate/data/$annotate_bag
done