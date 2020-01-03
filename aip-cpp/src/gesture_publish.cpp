/*
 * @Author: 王培荣
 * @Date: 2020-01-03 10:48:03
 * @LastEditTime : 2020-01-03 14:40:05
 * @LastEditors  : Please set LastEditors
 * @Description: 参考 https://ai.baidu.com/ai-doc/BODY/4k3cpywrv
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/aip-cpp/src/gesture_publish.cpp
 */

#include <iostream>
#include <curl/curl.h>
#include <jsoncpp/json/json.h> 
#include <string>

// libcurl库下载链接：https://curl.haxx.se/download.html
// jsoncpp库下载链接：https://github.com/open-source-parsers/jsoncpp/
const static std::string request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/gesture";
static std::string gesture_result;
/**
 * curl发送http请求调用的回调函数，回调函数中对返回的json格式的body进行了解析，解析结果储存在全局的静态变量当中
 * @param 参数定义见libcurl文档
 * @return 返回值定义见libcurl文档
 */
static size_t callback(void *ptr, size_t size, size_t nmemb, void *stream) {
    // 获取到的body存放在ptr中，先将其转换为string格式
    
    gesture_result = std::string((char *) ptr, size * nmemb);
    return size * nmemb;
}
/**
 * 手势识别
 * @return 调用成功返回0，发生错误返回其他错误码
 */
int gesture(std::string &json_result, const std::string &access_token) {
    std::string url = request_url + "?access_token=" + access_token;
    CURL *curl = NULL;
    CURLcode result_code;
    int is_success;
    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.data());
        curl_easy_setopt(curl, CURLOPT_POST, 1);
        curl_httppost *post = NULL;
        curl_httppost *last = NULL;
        curl_formadd(&post, &last, CURLFORM_COPYNAME, "image", CURLFORM_COPYCONTENTS, "【base64_img】", CURLFORM_END);

        curl_easy_setopt(curl, CURLOPT_HTTPPOST, post);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);
        result_code = curl_easy_perform(curl);
        if (result_code != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                    curl_easy_strerror(result_code));
            is_success = 1;
            return is_success;
        }
        json_result = gesture_result;
        curl_easy_cleanup(curl);
        is_success = 0;
    } else {
        fprintf(stderr, "curl_easy_init() failed.");
        is_success = 1;
    }
    return is_success;
}

int main(char argc, char** argv){
    const std::string access_token = "24.f63e044db7e28b0edbd46b275f5def4a.2592000.1580623101.282335-18165604";
 
}