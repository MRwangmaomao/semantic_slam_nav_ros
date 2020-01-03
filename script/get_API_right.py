'''
@Author: your name
@Date: 2020-01-03 13:52:01
@LastEditTime : 2020-01-03 13:55:17
@LastEditors  : Please set LastEditors
@Description: In User Settings Edit
@FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/script/get_API_right.py
'''
# encoding:utf-8
import requests 

host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=EXXFhKHgb8uAGvo2yu9qAf4g&client_secret=KTxGCvBRK6yvSAN1DHOh24Cl1Wk3G0jU'
response = requests.get(host)
if response:
    print(response.json())