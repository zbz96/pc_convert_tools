## About ##

This code is about **.pcd to .bin**  **.bin to .pcd** **.bag to .bin/.pcd** converting tool.  

## How to use ##
### 0. Environment ###
Python 3.8.2

### 1. Install python libraries ###
`$ pip install numpy==1.23.5`  
`$ pip install argparse`
`$ pip install rosbag`  
`$ pip install rosbags`  
`$ pip install rosnumpy`  

### 2. Launch python file ###
`$ python pcd2bin.py --pcd_path {path of input pcd file directory} --bin_path {path of output bin file directory}`

`$ python bin2pcd.py --pcd_path {path of input pcd file directory} --bin_path {path of output bin file directory}`

`$ python bagConvert.py --bag_path bag_path/  --mode bag2pcd `

`$ python bagConvert.py --bag_path bag_path/   --mode bag2bin `

`$ python bagConvert.py --bag_path bag_path/ --img_path img_path/ --camera_topic /camera0/image_raw/compressed`

