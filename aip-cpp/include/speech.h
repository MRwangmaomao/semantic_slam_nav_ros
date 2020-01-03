#ifndef __AIP_SPEECH_H__
#define __AIP_SPEECH_H__
#include "base/base.h"

#include <json/json.h>

namespace aip {
    
    class Speech: public AipBase
    {
    public:
        
        std::string _asr = "https://vop.baidu.com/server_api";
        
        std::string _tts = "http://tsn.baidu.com/text2audio";
        
        std::string _asr_pro = "https://vop.baidu.com/pro_api";
        
        Speech(const std::string app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        
        Json::Value request_asr(
                                  std::string const & url,
                                  Json::Value & data)
        {
            std::string response;
            Json::Value obj;
            int status_code = this->client.post(url, nullptr, data, nullptr, &response);
            
            if (status_code != CURLcode::CURLE_OK) {
                obj[aip::CURL_ERROR_CODE] = status_code;
                return obj;
            }

            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            
            return obj;
        }
        
        Json::Value request_tts(
                                  const std::string  url,
                                  std::map<std::string, std::string> & data,
                                  std::string & file_content)
        {
            std::string response;
            Json::Value obj;
            Json::Value file_json;
            int status_code = this->client.post(url, nullptr, data, nullptr, &response);
            if (status_code != CURLcode::CURLE_OK) {
                obj[aip::CURL_ERROR_CODE] = status_code;
                return obj;
            }
            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            
            if (error.empty()) {
                // 接口返回错误信息
                obj[aip::CURL_ERROR_CODE] = 0;
            } else {
                // 接口返回语音文件
                file_content = response;
            }
            return obj;
        }
        
        
        Json::Value recognize(const std::string voice_binary, const std::string & format, const int & rate, std::map<std::string, std::string> const & options)
        {
            Json::Value data;
            
            std::map<std::string, std::string>::const_iterator it;
            for(it=options.begin(); it!=options.end(); it++)
            {
                if (it->first == "dev_pid") {
                    try {
                        data[it->first] = std::stoi(it->second);
                    } catch (std::exception &e) {
                    }
                } else {
                    data[it->first] = it->second;
                }
            }
            
            std::string token = this->getAccessToken();
            
            data["speech"] = base64_encode(voice_binary.c_str(), (int) voice_binary.size());
            data["format"] = format;
            data["rate"] = std::to_string(rate);
            data["channel"] = "1";
            data["token"] = token;
            data["cuid"] = this->getAk();
            data["len"] = (int) voice_binary.size();
            
            Json::Value result = this->request_asr(_asr, data);
            return result;
        }
        
        Json::Value recognize_pro(const std::string voice_binary, const std::string & format, const int & rate, std::map<std::string, std::string> const & options)
        {
            Json::Value data;
            
            std::map<std::string, std::string>::const_iterator it;

            data["dev_pid"] = 80001;
            for (it=options.begin(); it!=options.end(); it++)
            {
                if (it->first == "dev_pid") {
                    try {
                        data[it->first] = std::stoi(it->second);
                    } catch (std::exception &e) {
                        
                    }
                } else {
                    data[it->first] = it->second;
                }
            }
            
            std::string token = this->getAccessToken();
            
            data["speech"] = base64_encode(voice_binary.c_str(), (int) voice_binary.size());
            data["format"] = format;
            data["rate"] = std::to_string(rate);
            data["channel"] = "1";
            data["token"] = token;
            data["cuid"] = this->getAk();
            data["len"] = (int) voice_binary.size();
            
            Json::Value result = this->request_asr(_asr_pro, data);
            return result;
        }
        
        
        Json::Value recognize_url(const std::string & url,
                                  const std::string & callback, const std::string & format,
                                  const int & rate,
                                  std::map<std::string, std::string> options)
        {
            Json::Value data;
            std::map<std::string, std::string>::iterator it;
            
            for(it=options.begin(); it!=options.end(); it++)
            {
                data[it->first] = it->second;
            }
            
            std::string token = this->getAccessToken();
            
            data["url"] = url;
            data["callback"] = callback;
            data["format"] = format;
            data["rate"] = std::to_string(rate);
            data["channel"] = 1;
            data["token"] = token;
            data["cuid"] = this->getAk();
            
            Json::Value result = this->request_asr(_asr, data);
            return result;
        }
        
        Json::Value text2audio(const std::string & text, std::map<std::string, std::string> const & options, std::string & file_content)
        {
            std::map<std::string, std::string> data;
            std::map<std::string, std::string>::const_iterator it;
            
            for(it = options.begin(); it !=options.end(); it++)
            {
                data[it->first] = it->second;
            }
            
            std::string token = this->getAccessToken();
            
            data["tex"] = text;
            data["lan"] = "zh";
            data["ctp"] = "1";
            data["tok"] = token;
            data["cuid"] = this->getAk();
            
            Json::Value result = this->request_tts(_tts, data, file_content);
            return result;
        }
        
    };
    
}
#endif
