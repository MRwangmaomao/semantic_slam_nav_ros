'''
@Author: your name
@Date: 2020-01-03 13:43:59
@LastEditTime : 2020-01-03 14:05:38
@LastEditors  : Please set LastEditors
@Description: In User Settings Edit
@FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/script/get_gesture.py
'''
# encoding:utf-8

import requests
import base64

request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/gesture"

f = open('gesture.jpg', 'rb')
img = base64.b64encode(f.read())

params = {"image":img}
access_token = '24.f63e044db7e28b0edbd46b275f5def4a.2592000.1580623101.282335-18165604'
request_url = request_url + "?access_token=" + access_token
headers = {'content-type': 'application/x-www-form-urlencoded'}
response = requests.post(request_url, data=params, headers=headers)
if response:
    print (response.json())
    