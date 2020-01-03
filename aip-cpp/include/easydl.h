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

#ifndef __AIP_EASYDL_H__
#define __AIP_EASYDL_H__

#include "base/base.h"

namespace aip {

    class EasyDL: public AipBase
    {
    public:
        
        

        EasyDL(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }

        
        Json::Value easydl_request_image(
            std::string const & url,
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = aip::base64_encode(image.c_str(), (int) image.size());
;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(url, null, data.toStyledString(), null);

            return result;
        }
        
        Json::Value easydl_request_sound(
                                         std::string const & url,
                                         std::string const & sound,
                                         const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["sound"] = aip::base64_encode(sound.c_str(), (int) sound.size());
            ;
            
            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }
            
            Json::Value result =
            this->request(url, null, data.toStyledString(), null);
            
            return result;
        }

    };
}
#endif
