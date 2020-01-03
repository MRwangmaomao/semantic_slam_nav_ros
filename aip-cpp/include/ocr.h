/**
 * Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * @author baidu aip
 */

#ifndef __AIP_OCR_H__
#define __AIP_OCR_H__

#include "base/base.h"

namespace aip {

    class Ocr: public AipBase
    {
    public:

    
        std::string _general_basic =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general_basic";
        
        std::string _accurate_basic =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic";
        
        std::string _general =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general";
        
        std::string _accurate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate";
        
        std::string _general_enhanced =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general_enhanced";
        
        std::string _web_image =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/webimage";
        
        std::string _idcard =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/idcard";
        
        std::string _bankcard =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/bankcard";
        
        std::string _driving_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/driving_license";
        
        std::string _vehicle_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vehicle_license";
        
        std::string _license_plate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate";
        
        std::string _business_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/business_license";
        
        std::string _receipt =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/receipt";
        
        std::string _train_ticket =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/train_ticket";
        
        std::string _taxi_receipt =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/taxi_receipt";
        
        std::string _form =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/form";
        
        std::string _table_recognize =
            "https://aip.baidubce.com/rest/2.0/solution/v1/form_ocr/request";
        
        std::string _table_result_get =
            "https://aip.baidubce.com/rest/2.0/solution/v1/form_ocr/get_request_result";
        
        std::string _vin_code =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vin_code";
        
        std::string _quota_invoice =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/quota_invoice";
        
        std::string _household_register =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/household_register";
        
        std::string _HK_Macau_exitentrypermit =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/HK_Macau_exitentrypermit";
        
        std::string _taiwan_exitentrypermit =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/taiwan_exitentrypermit";
        
        std::string _birth_certificate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/birth_certificate";
        
        std::string _vehicle_invoice =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vehicle_invoice";
        
        std::string _vehicle_certificate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vehicle_certificate";
        
        std::string _invoice =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/invoice";
        
        std::string _air_ticket =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/air_ticket";
        
        std::string _insurance_documents =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/insurance_documents";
        
        std::string _vat_invoice =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vat_invoice";
        
        std::string _qrcode =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/qrcode";
        
        std::string _numbers =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/numbers";
        
        std::string _lottery =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/lottery";
        
        std::string _passport =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/passport";
        
        std::string _business_card =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/business_card";
        
        std::string _handwriting =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/handwriting";
        
        std::string _custom =
            "https://aip.baidubce.com/rest/2.0/solution/v1/iocr/recognise";
        

        Ocr(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * general_basic
         * 用户向服务请求识别某张图中的所有文字
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_basic(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_basic, null, data, null);

            return result;
        }
        
        /**
         * general_basic_url
         * 用户向服务请求识别某张图中的所有文字
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_basic_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_basic, null, data, null);

            return result;
        }
        
        /**
         * accurate_basic
         * 用户向服务请求识别某张图中的所有文字，相对于通用文字识别该产品精度更高，但是识别耗时会稍长。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value accurate_basic(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_accurate_basic, null, data, null);

            return result;
        }
        
        /**
         * general
         * 用户向服务请求识别某张图中的所有文字，并返回文字在图中的位置信息。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general, null, data, null);

            return result;
        }
        
        /**
         * general_url
         * 用户向服务请求识别某张图中的所有文字，并返回文字在图中的位置信息。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general, null, data, null);

            return result;
        }
        
        /**
         * accurate
         * 用户向服务请求识别某张图中的所有文字，并返回文字在图片中的坐标信息，相对于通用文字识别（含位置信息版）该产品精度更高，但是识别耗时会稍长。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value accurate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_accurate, null, data, null);

            return result;
        }
        
        /**
         * general_enhanced
         * 某些场景中，图片中的中文不光有常用字，还包含了生僻字，这时用户需要对该图进行文字识别，应使用通用文字识别（含生僻字版）。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_enhanced(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_enhanced, null, data, null);

            return result;
        }
        
        /**
         * general_enhanced_url
         * 某些场景中，图片中的中文不光有常用字，还包含了生僻字，这时用户需要对该图进行文字识别，应使用通用文字识别（含生僻字版）。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br>- CHN_ENG：中英文混合；<br>- ENG：英文；<br>- POR：葡萄牙语；<br>- FRE：法语；<br>- GER：德语；<br>- ITA：意大利语；<br>- SPA：西班牙语；<br>- RUS：俄语；<br>- JAP：日语；<br>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_enhanced_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_enhanced, null, data, null);

            return result;
        }
        
        /**
         * web_image
         * 用户向服务请求识别一些网络上背景复杂，特殊字体的文字。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         */
        Json::Value web_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_web_image, null, data, null);

            return result;
        }
        
        /**
         * web_image_url
         * 用户向服务请求识别一些网络上背景复杂，特殊字体的文字。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         */
        Json::Value web_image_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_web_image, null, data, null);

            return result;
        }
        
        /**
         * idcard
         * 用户向服务请求识别身份证，身份证识别包括正面和背面。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * @param id_card_side front：身份证含照片的一面；back：身份证带国徽的一面
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * detect_risk 是否开启身份证风险类型(身份证复印件、临时身份证、身份证翻拍、修改过的身份证)功能，默认不开启，即：false。可选值:true-开启；false-不开启
         */
        Json::Value idcard(
            std::string const & image,
            std::string const & id_card_side,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());
            data["id_card_side"] = id_card_side;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_idcard, null, data, null);

            return result;
        }
        
        /**
         * bankcard
         * 识别银行卡并返回卡号和发卡行。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value bankcard(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_bankcard, null, data, null);

            return result;
        }
        
        /**
         * driving_license
         * 对机动车驾驶证所有关键字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         */
        Json::Value driving_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_driving_license, null, data, null);

            return result;
        }
        
        /**
         * vehicle_license
         * 对机动车行驶证正本所有关键字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         * accuracy normal 使用快速服务，1200ms左右时延；缺省或其它值使用高精度服务，1600ms左右时延
         */
        Json::Value vehicle_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_license, null, data, null);

            return result;
        }
        
        /**
         * license_plate
         * 识别机动车车牌，并返回签发地和号牌。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * multi_detect 是否检测多张车牌，默认为false，当置为true的时候可以对一张图片内的多张车牌进行识别
         */
        Json::Value license_plate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_license_plate, null, data, null);

            return result;
        }
        
        /**
         * business_license
         * 识别营业执照，并返回关键字段的值，包括单位名称、法人、地址、有效期、证件编号、社会信用代码等。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value business_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_business_license, null, data, null);

            return result;
        }
        
        /**
         * receipt
         * 用户向服务请求识别医疗票据、发票、的士票、保险保单等票据类图片中的所有文字，并返回文字在图中的位置信息。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * probability 是否返回识别结果中每一行的置信度
         * accuracy normal 使用快速服务，1200ms左右时延；缺省或其它值使用高精度服务，1600ms左右时延
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         */
        Json::Value receipt(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_receipt, null, data, null);

            return result;
        }
        
        /**
         * train_ticket
         * 支持对大陆火车票的车票号、始发站、目的站、车次、日期、票价、席别、姓名进行结构化识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value train_ticket(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_train_ticket, null, data, null);

            return result;
        }
        
        /**
         * taxi_receipt
         * 针对出租车票（现支持北京）的发票号码、发票代码、车号、日期、时间、金额进行结构化识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value taxi_receipt(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_taxi_receipt, null, data, null);

            return result;
        }
        
        /**
         * form
         * 自动识别表格线及表格内容，结构化输出表头、表尾及每个单元格的文字内容。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value form(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_form, null, data, null);

            return result;
        }
        
        /**
         * table_recognize
         * 自动识别表格线及表格内容，结构化输出表头、表尾及每个单元格的文字内容。表格文字识别接口为异步接口，分为两个API：提交请求接口、获取结果接口。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value table_recognize(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_table_recognize, null, data, null);

            return result;
        }
        
        /**
         * table_result_get
         * 获取表格文字识别结果
         * @param request_id 发送表格文字识别请求时返回的request id
         * options 可选参数:
         * result_type 期望获取结果的类型，取值为“excel”时返回xls文件的地址，取值为“json”时返回json格式的字符串,默认为”excel”
         */
        Json::Value table_result_get(
            std::string const & request_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["request_id"] = request_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_table_result_get, null, data, null);

            return result;
        }
        
        /**
         * vin_code
         * 对车辆车架上、挡风玻璃上的VIN码进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value vin_code(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vin_code, null, data, null);

            return result;
        }
        
        /**
         * quota_invoice
         * 对各类定额发票的代码、号码、金额进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value quota_invoice(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_quota_invoice, null, data, null);

            return result;
        }
        
        /**
         * household_register
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对出生地、出生日期、姓名、民族、与户主关系、性别、身份证号码字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value household_register(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_household_register, null, data, null);

            return result;
        }
        
        /**
         * HK_Macau_exitentrypermit
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对港澳通行证证号、姓名、姓名拼音、性别、有效期限、签发地点、出生日期字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value HK_Macau_exitentrypermit(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_HK_Macau_exitentrypermit, null, data, null);

            return result;
        }
        
        /**
         * taiwan_exitentrypermit
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对台湾通行证证号、签发地、出生日期、姓名、姓名拼音、性别、有效期字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value taiwan_exitentrypermit(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_taiwan_exitentrypermit, null, data, null);

            return result;
        }
        
        /**
         * birth_certificate
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对台湾通行证证号、签发地、出生日期、姓名、姓名拼音、性别、有效期字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value birth_certificate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_birth_certificate, null, data, null);

            return result;
        }
        
        /**
         * vehicle_invoice
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】识别机动车销售发票号码、代码、日期、价税合计等14个字段
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value vehicle_invoice(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_invoice, null, data, null);

            return result;
        }
        
        /**
         * vehicle_certificate
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】识别车辆合格证编号、车架号、排放标准、发动机编号等12个字段
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value vehicle_certificate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_certificate, null, data, null);

            return result;
        }
        
        /**
         * invoice
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对国家/地方税务局发行的横/竖版通用机打发票的号码、代码、日期、合计金额、类型、商品名称字段进行结构化识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * location 是否输出位置信息，true：输出位置信息，false：不输出位置信息，默认false
         */
        Json::Value invoice(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_invoice, null, data, null);

            return result;
        }
        
        /**
         * air_ticket
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对飞机行程单中的姓名、始发站、目的站、航班号、日期、票价字段进行结构化识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * location 是否输出位置信息，true：输出位置信息，false：不输出位置信息，默认false
         */
        Json::Value air_ticket(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_air_ticket, null, data, null);

            return result;
        }
        
        /**
         * insurance_documents
         * 【此接口需要您在[申请页面](https://cloud.baidu.com/survey/AICooperativeConsultingApply.html)中提交合作咨询开通权限】对各类保单中投保人、受益人的各项信息、保费、保险名称等字段进行结构化识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * rkv_business 是否进行商业逻辑处理，rue：进行商业逻辑处理，false：不进行商业逻辑处理，默认true
         */
        Json::Value insurance_documents(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_insurance_documents, null, data, null);

            return result;
        }
        
        /**
         * vat_invoice
         * 此接口需要您在页面中提交合作咨询开通权限】 识别并结构化返回增值税发票的各个字段及其对应值，包含了发票基础信息9项，货物相关信息12项，购买方/销售方的名称、识别号、地址电话、开户行及账号，共29项结构化字段。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value vat_invoice(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vat_invoice, null, data, null);

            return result;
        }
        
        /**
         * qrcode
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限识别条形码、二维码中包含的URL或其他信息内容
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value qrcode(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_qrcode, null, data, null);

            return result;
        }
        
        /**
         * numbers
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限】对图像中的阿拉伯数字进行识别提取，适用于快递单号、手机号、充值码提取等场景
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br>- true：检测朝向；<br>- false：不检测朝向。
         */
        Json::Value numbers(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_numbers, null, data, null);

            return result;
        }
        
        /**
         * lottery
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限】对大乐透、双色球彩票进行识别，并返回识别结果
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         */
        Json::Value lottery(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_lottery, null, data, null);

            return result;
        }
        
        /**
         * passport
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限】支持对中国大陆居民护照的资料页进行结构化识别，包含国家码、姓名、性别、护照号、出生日期、签发日期、有效期至、签发地点。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value passport(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_passport, null, data, null);

            return result;
        }
        
        /**
         * business_card
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限】提供对各类名片的结构化识别功能，提取姓名、邮编、邮箱、电话、网址、地址、手机号字段
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value business_card(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_business_card, null, data, null);

            return result;
        }
        
        /**
         * handwriting
         * 【此接口需要您在[页面](http://ai.baidu.com/tech/ocr)中提交合作咨询开通权限】提供对各类名片的结构化识别功能，提取姓名、邮编、邮箱、电话、网址、地址、手机号字段
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         */
        Json::Value handwriting(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_handwriting, null, data, null);

            return result;
        }
        
        /**
         * custom
         * 自定义模板文字识别，是针对百度官方没有推出相应的模板，但是当用户需要对某一类卡证/票据（如房产证、军官证、火车票等）进行结构化的提取内容时，可以使用该产品快速制作模板，进行识别。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * templateSign 您在自定义文字识别平台制作的模板的ID
         * classifierId 分类器Id。这个参数和templateSign至少存在一个，优先使用templateSign。存在templateSign时，表示使用指定模板；如果没有templateSign而有classifierId，表示使用分类器去判断使用哪个模板
         */
        Json::Value custom(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_custom, null, data, null);

            return result;
        }
        

    };
}
#endif