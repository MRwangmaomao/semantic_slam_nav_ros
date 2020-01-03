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
#ifndef __AIP_UTILS_H__
#define __AIP_UTILS_H__

#include <string>
#include <fstream>
#include <ctype.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <algorithm>
#include <openssl/md5.h>

const int __BCE_VERSION__ = 1;
const int __BCE_EXPIRE__ = 1800;

namespace aip {
    
    template<class CharT, class Traits, class Allocator>
    std::basic_istream<CharT, Traits>& getall(std::basic_istream<CharT, Traits>& input,
                                              std::basic_string<CharT, Traits, Allocator>& str) {
        std::ostringstream oss;
        oss << input.rdbuf();
        str.assign(oss.str());
        return input;
    }
    
    inline int get_file_content(const char *filename, std::string* out) {
        std::ifstream in(filename, std::ios::in | std::ios::binary);
        if (in) {
            getall(in, *out);
            return 0;
        } else {
            return -1;
        }
    }
    
    inline std::string to_upper(std::string src)
    {
        std::transform(src.begin(), src.end(), src.begin(), toupper);
        return src;
    }
    
    
    inline std::string to_lower(std::string src)
    {
        std::transform(src.begin(), src.end(), src.begin(), tolower);
        return src;
    }
    
    inline std::string to_hex(unsigned char c, bool lower = false)
    {
        const std::string hex = "0123456789ABCDEF";
        
        std::stringstream ss;
        ss << hex[c >> 4] << hex[c & 0xf];
        
        return lower ? to_lower(ss.str()) : ss.str();
    }
    
    inline time_t now()
    {
        return time(NULL);
    }
    
    std::string utc_time(time_t timestamp)
    {
        struct tm result_tm;
        char buffer[32];
        
#ifdef _WIN32
        gmtime_s(&result_tm, &timestamp);
#else
        gmtime_r(&timestamp, &result_tm);
#endif
        
        size_t size = strftime(buffer, 32, "%Y-%m-%dT%H:%M:%SZ", &result_tm);
        
        return std::string(buffer, size);
    }
    
    void url_parse(
                   const std::string & url,
                   std::map<std::string, std::string> & params)
    {
        int pos = (int)url.find("?");
        if (pos != -1)
        {
            int key_start = pos + 1,
            key_len = 0,
            val_start = 0;
            for (int i = key_start; i <= (int)url.size(); ++i)
            {
                switch (url[i])
                {
                    case '=':
                        key_len = i - key_start;
                        val_start = i + 1;
                        break;
                    case '\0':
                    case '&':
                        if (key_len != 0)
                        {
                            params[url.substr(key_start, key_len)] = url.substr(val_start, i - val_start);
                            key_start = i + 1;
                            key_len = 0;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }
    
    std::string url_encode(const std::string & input, bool encode_slash=true)
    {
        std::stringstream ss;
        const char *str = input.c_str();
        
        for (uint32_t i = 0; i < input.size(); i++)
        {
            unsigned char c = str[i];
            if (isalnum(c) || c == '_' || c == '-' || c == '~' || c == '.' || (!encode_slash && c == '/'))
            {
                ss << c;
            }
            else
            {
                ss << "%" << to_hex(c);
            }
        }
        
        return ss.str();
    }
    
    std::string canonicalize_params(std::map<std::string, std::string> & params)
    {
        std::vector<std::string> v;
        v.reserve(params.size());
        
        for (auto & it : params) {
            v.push_back(url_encode(it.first) + "=" + url_encode(it.second));
        }
        std::sort(v.begin(), v.end());
        
        std::string result;
        for (auto & it : v)
        {
            result.append((result.empty() ? "" : "&") + it);
        }
        return result;
    }
    
    std::string canonicalize_headers(std::map<std::string, std::string> & headers)
    {
        std::vector<std::string> v;
        v.reserve(headers.size());
        
        for (auto & it : headers) {
            v.push_back(url_encode(to_lower(it.first)) + ":" + url_encode(it.second));
        }
        std::sort(v.begin(), v.end());
        
        std::string result;
        for (auto & it : v)
        {
            result.append((result.empty() ? "" : "\n") + it);
        }
        return result;
    }
    
    std::string get_headers_keys(std::map<std::string, std::string> & headers)
    {
        std::vector<std::string> v;
        v.reserve(headers.size());
        
        for (auto & it : headers) {
            v.push_back(to_lower(it.first));
        }
        
        std::string result;
        for (auto & it : v)
        {
            result.append((result.empty() ? "" : ";") + it);
        }
        return result;
    }
    
    std::string get_host(const std::string & url)
    {
        int pos = (int)url.find("://") + 3;
        return url.substr(
                          pos,
                          url.find('/', pos) - pos
                          );
    }
    
    
    std::string get_path(const std::string & url)
    {
        int path_start = (int)url.find('/', url.find("://") + 3);
        int path_end = (int)url.find('?');
        path_end = path_end == -1 ? (int)url.size() : path_end;
        
        return url.substr(path_start, path_end - path_start);
    }
    
    std::string hmac_sha256(
                            const std::string & src,
                            const std::string & sk)
    {
        const EVP_MD *evp_md = EVP_sha256();
        unsigned char md[EVP_MAX_MD_SIZE];
        unsigned int md_len = 0;
        
        if (HMAC(evp_md,
                 reinterpret_cast<const unsigned char *>(sk.data()), (int)sk.size(),
                 reinterpret_cast<const unsigned char *>(src.data()), src.size(),
                 md, &md_len) == NULL)
        {
            return "";
        }
        
        std::stringstream ss;
        for (int i = 0; i < (int)md_len; ++i)
        {
            ss << to_hex(md[i], true);
        }
        
        return ss.str();
    }
    
    void sign(
              std::string method,
              std::string & url,
              std::map<std::string, std::string> & params,
              std::map<std::string, std::string> & headers,
              std::string & ak,
              std::string & sk)
    {
        url_parse(url, params);
        headers["Host"] = get_host(url);
        std::string timestamp = utc_time(now());
        headers["x-bce-date"] = timestamp;
        
        std::stringstream ss;
        ss << "bce-auth-v" << __BCE_VERSION__ << "/" << ak << "/"
        << timestamp  << "/" << __BCE_EXPIRE__;
        
        std::string val = ss.str();
        std::string sign_key = hmac_sha256(val, sk);
        
        ss.str("");
        ss << to_upper(method) << '\n' << url_encode(get_path(url), false)
        << '\n' << canonicalize_params(params)
        << '\n' << canonicalize_headers(headers);
        
        std::string signature = hmac_sha256(ss.str(), sign_key);
        
        ss.str("");
        ss << "bce-auth-v" << __BCE_VERSION__ << "/" << ak << "/"
        << timestamp  << "/" << __BCE_EXPIRE__ << "/"
        << get_headers_keys(headers) << "/" << signature;
        
        headers["authorization"] = ss.str();
    }
    
}

#endif
