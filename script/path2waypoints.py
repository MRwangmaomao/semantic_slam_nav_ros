from datetime import datetime
import numpy as np
import re
import os
import string
import math
import sys

def main():
    gt_data = open("/home/wpr/code/catkin_ws/src/costmap_lrgbd_ros/config/gt.txt", "r")
    lines = gt_data.readlines()        

    path_file_name = './' + 'waypoints' + '.txt'
    print("output the file:" + path_file_name) 
    
    max_index_num = 100
    index_num = 0
    with open(path_file_name, "wb+") as f:
        for line in lines:
            index_num = index_num + 1
            if index_num == max_index_num:
                index_num = 0
                f.write(line)

if __name__ == "__main__":
    main()