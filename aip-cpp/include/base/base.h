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
#ifndef __AIP_BASE_H__
#define __AIP_BASE_H__

#include <memory>
#include <cstring>
#include "http.h"
#include "json/json.h"
#include "base64.h"
#include "curl/curl.h"
#include "utils.h"

namespace aip {

    static const char* AIP_SDK_VERSION = "0.8.5";
    static const char* CURL_ERROR_CODE = "curl_error_code";
    static const std::string ACCESS_TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token";
    static const std::map<std::string, std::string> null;
    enum class VECTOR_JOIN_TYPE {BASE64, URL};

    class AipBase
    {
    private:
        std::string _app_id;
        int _expired_time;
        bool _is_bce;
        std::string _scope;

        std::string vector_join(const std::vector<std::string> & v_images, VECTOR_JOIN_TYPE type) {
            std::string images;
            for (size_t i = 0; i < v_images.size(); i++)
            {
                std::string image = v_images[i];
                switch (type) {
                    case VECTOR_JOIN_TYPE::BASE64:
                        images += base64_encode(image.c_str(), (int) image.size());
                        break;
                    case VECTOR_JOIN_TYPE::URL:
                        char* url = curl_escape(image.c_str(), (int) image.size());
                        images += url;
                        curl_free(url);
                        break;
                }
                if (i < v_images.size() - 1) {
                    images += ",";
                }

            }
            return images;
        }


    protected:
        std::string getAccessToken()
        {
            time_t now = time(NULL);

            if (!access_token.empty() && now < this->_expired_time - 60 * 60 * 24)
            {
                return this->access_token;
            }

            std::string response;
            std::map<std::string, std::string> params;

            params["grant_type"] = "client_credentials";
            params["client_id"] = this->ak;
            params["client_secret"] = this->sk;
            int status_code = this->client.get(
                                               ACCESS_TOKEN_URL,
                                               &params,
                                               nullptr,
                                               &response
                                               );

            Json::Value obj;
            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj.toStyledString();
            }

            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            this->access_token = obj["access_token"].asString();
            this->_expired_time = obj["expires_in"].asInt() + (int) now;
            this->_scope = obj["scope"].asString();
            return this->access_token;
        }



    public:
        std::string ak;
        std::string sk;
        HttpClient client;
        Json::CharReaderBuilder crbuilder;
        std::string access_token;
        bool useHttp2;
        AipBase(const std::string & app_id, const std::string & ak, const std::string & sk):
        _app_id(app_id),
        _is_bce(false),
        ak(ak),
        sk(sk),
        useHttp2(false)
        {
            if (_app_id == "")
            {
            }
        }
        void set_use_http2(bool use_http2)
        {
            this->client.setUseHttp2(use_http2);
        }
        void set_is_bce() {
            _is_bce = true;
        }
        void setConnectionTimeoutInMillis(int connect_timeout)
        {
            this->client.setConnectTimeout(connect_timeout);
        }

        void setSocketTimeoutInMillis(int socket_timeout)
        {
            this->client.setSocketTimeout(socket_timeout);
        }

        void setDebug(bool debug)
        {
            this->client.setDebug(debug);
        }

        std::string getAk() {
            return ak;
        }

        std::string vector_join_base64(const std::vector<std::string> & v_images) {
            return vector_join(v_images, VECTOR_JOIN_TYPE::BASE64);
        }

        std::string vector_join_url(const std::vector<std::string> & v_images) {
            return vector_join(v_images, VECTOR_JOIN_TYPE::URL);
        }

        Json::Value request(
                            std::string url,
                            std::map<std::string, std::string> const & params,
                            std::string const & data,
                            std::map<std::string, std::string> const & headers,
                            bool isRetry=false)
        {
            std::string response;
            Json::Value obj;
            std::string body;
            auto headers_for_sign = headers;

            auto temp_params = params;

            temp_params["charset"] = "UTF-8";

            this->prepare_request(url, temp_params, headers_for_sign);

            int status_code = this->client.post(url, &temp_params, data, &headers_for_sign, &response);

            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj;
            }

            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            if ((obj["error_code"].asInt() == 110 || obj["error_code"].asInt() == 111) && !isRetry) {
                this->access_token = "";
                return request(url, params, data, headers, true);

            }
            return obj;
        }

        Json::Value request(
                            std::string url,
                            std::map<std::string, std::string> const & params,
                            std::map<std::string, std::string> const & data,
                            std::map<std::string, std::string> const & headers,
                            bool isRetry=false)
        {
            std::string response;
            Json::Value obj;

            auto headers_for_sign = headers;
            auto temp_params = params;

            this->prepare_request(url, temp_params, headers_for_sign);

            int status_code = this->client.post(url, &temp_params, data, &headers_for_sign, &response);

            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj;
            }

            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            if ((obj["error_code"].asInt() == 110 || obj["error_code"].asInt() == 111) && !isRetry) {
                this->access_token = "";
                return request(url, params, data, headers, true);
            }
            return obj;
        }

        void prepare_request(std::string url,
                             std::map<std::string, std::string> & params,
                             std::map<std::string, std::string> & headers)
        {

            params["aipSdk"] = "C";
            params["aipSdkVersion"] = AIP_SDK_VERSION;

            if (_is_bce) {
                std::string method = "POST";
                sign(method, url, params, headers, ak, sk);
            } else {
                params["access_token"] = this->getAccessToken();
            }

        }


    };

}
#endif
