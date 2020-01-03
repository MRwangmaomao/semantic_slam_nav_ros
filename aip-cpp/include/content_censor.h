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

#ifndef __AIP_CONTENTCENSOR_H__
#define __AIP_CONTENTCENSOR_H__

#include "base/base.h"

namespace aip {

    class Contentcensor: public AipBase
    {
    public:

    
        std::string _anti_porn =
            "https://aip.baidubce.com/rest/2.0/antiporn/v1/detect";
        
        std::string _anti_porn_gif =
            "https://aip.baidubce.com/rest/2.0/antiporn/v1/detect_gif";
        
        std::string _anti_spam =
        "https://aip.baidubce.com/rest/2.0/antispam/v2/spam";

        
        std::string _user_defined_image =
            "https://aip.baidubce.com/rest/2.0/solution/v1/img_censor/v2/user_defined";
        
        std::string _user_defined_text =
            "https://aip.baidubce.com/rest/2.0/solution/v1/text_censor/v2/user_defined";
        
        std::string _face_audit =
            "https://aip.baidubce.com/rest/2.0/solution/v1/face_audit";

        std::string _report =
            "https://aip.baidubce.com/rpc/2.0/feedback/v1/report";

        std::string _combo =
        "https://aip.baidubce.com/api/v1/solution/direct/img_censor";


        Contentcensor(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
    
        
        /**
         * anti_porn_gif
         * 该请求用于鉴定GIF图片的色情度，对于非gif接口，请使用色情识别接口。接口会对图片中每一帧进行识别，并返回所有检测结果中色情值最大的为结果。目前支持三个维度：色情、性感、正常。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value anti_porn_gif(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_anti_porn_gif, null, data, null);

            return result;
        }
        
        /**
         * anti_spam
         *
         * @param content 文字内容
         * options 可选参数:
         */
        Json::Value anti_spam(
                                  std::string const & content,
                                  const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["content"] = content;
            
            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));
            
            Json::Value result =
            this->request(_anti_spam, null, data, null);
            
            return result;
        }
        
        Json::Value user_defined_image(
            std::string const & image_or_url,
            std::string const & type,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            if (type == "image") {
                data["image"] = base64_encode(image_or_url.c_str(), (int) image_or_url.size());
            }
            if (type == "imgUrl") {
                data["imgUrl"] = image_or_url;
            }

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_user_defined_image, null, data, null);

            return result;
        }
        
        Json::Value user_defined_text(
                                 std::string const & text,
                                      const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["text"] = text;
            
            Json::Value result =
            this->request(_user_defined_text, null, data, null);
            
            return result;
        }
        
        Json::Value combo(
                                 std::string const & image_or_url,
                                 std::string const & type,
                                 std::vector<std::string> const & scenes,
                                 Json::Value const & scene_conf)
        {
            Json::Value data;
            
            if (type == "image") {
                data["image"] = base64_encode(image_or_url.c_str(), (int) image_or_url.size());
            }
            if (type == "imgUrl") {
                data["imgUrl"] = image_or_url;
            }
            
            data["scenes"] = {};
            
            for (int i = 0; i < (int) scenes.size(); i++)
            {
                data["scenes"][i] = scenes[i];
            }
            
            if (scene_conf != Json::Value::null) {
                data["sceneConf"] = scene_conf;
            }
            
            std::map<std::string, std::string> extern_headers;
            extern_headers["Content-Type"] = "application/json";
            
            Json::Value result =
            this->request(_combo, null, data.toStyledString(), extern_headers);
            
            return result;
        }


        
        Json::Value face_audit(
                                 const std::vector<std::string> image_or_url,
                                 std::string const & type,
                                 const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            if (type == "images") {
                data["images"] = vector_join_base64(image_or_url);
            }
            if (type == "imgUrls") {
                data["imgUrls"] = vector_join_url(image_or_url);
            }
            
            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));
            
            Json::Value result =
            this->request(_face_audit, null, data, null);
            
            return result;
        }
        
        Json::Value report(
                               Json::Value const & report)
        {
            Json::Value param;
            param["feedback"] = report;
            Json::Value result =
            this->request(_report, null, param.toStyledString(), null);
            
            return result;
        }


        

    };
}
#endif
