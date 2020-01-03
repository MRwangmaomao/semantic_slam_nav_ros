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

#ifndef __AIP_IMAGESEARCH_H__
#define __AIP_IMAGESEARCH_H__

#include "base/base.h"

namespace aip {

    class Imagesearch: public AipBase
    {
    public:

    
        std::string _same_hq_add =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/add";
        
        std::string _same_hq_search =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/search";
        
        std::string _same_hq_update =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/update";
        
        std::string _same_hq_delete =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/delete";
        
        std::string _similar_add =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/add";
        
        std::string _similar_search =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/search";
        
        std::string _similar_update =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/update";
        
        std::string _similar_delete =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/delete";
        
        std::string _product_add =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/add";
        
        std::string _product_search =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/search";
        
        std::string _product_update =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/update";
        
        std::string _product_delete =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/delete";
        

        Imagesearch(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * same_hq_add
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）；同时可提交分类维度信息（具体变量为tags，最多可传入2个tag），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_add, null, data, null);

            return result;
        }
        
        /**
         * same_hq_add_url
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）；同时可提交分类维度信息（具体变量为tags，最多可传入2个tag），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_add_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_add, null, data, null);

            return result;
        }
        
        /**
         * same_hq_search
         * 完成入库后，可使用该接口实现相同图检索。**支持传入指定分类维度（具体变量tags）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息。**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value same_hq_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_search, null, data, null);

            return result;
        }
        
        /**
         * same_hq_search_url
         * 完成入库后，可使用该接口实现相同图检索。**支持传入指定分类维度（具体变量tags）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息。**
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value same_hq_search_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_search, null, data, null);

            return result;
        }
        
        /**
         * same_hq_update
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_update(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_update, null, data, null);

            return result;
        }
        
        /**
         * same_hq_update_url
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_update_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_update, null, data, null);

            return result;
        }
        
        /**
         * same_hq_update_cont_sign
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param cont_sign 图片签名
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_update_cont_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_update, null, data, null);

            return result;
        }
        
        /**
         * same_hq_delete_by_image
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value same_hq_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_delete, null, data, null);

            return result;
        }
        
        /**
         * same_hq_delete_by_url
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         */
        Json::Value same_hq_delete_by_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_delete, null, data, null);

            return result;
        }
        
        /**
         * same_hq_delete_by_sign
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param cont_sign 图片签名
         * options 可选参数:
         */
        Json::Value same_hq_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_delete, null, data, null);

            return result;
        }
        
        /**
         * similar_add
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）；同时可提交分类维度信息（具体变量为tags，最多可传入2个tag），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_add, null, data, null);

            return result;
        }
        
        /**
         * similar_add_url
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）；同时可提交分类维度信息（具体变量为tags，最多可传入2个tag），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_add_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_add, null, data, null);

            return result;
        }
        
        /**
         * similar_search
         * 完成入库后，可使用该接口实现相似图检索。**支持传入指定分类维度（具体变量tags）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息。**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value similar_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_search, null, data, null);

            return result;
        }
        
        /**
         * similar_search_url
         * 完成入库后，可使用该接口实现相似图检索。**支持传入指定分类维度（具体变量tags）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息。**
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value similar_search_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_search, null, data, null);

            return result;
        }
        
        /**
         * similar_update
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_update(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_update, null, data, null);

            return result;
        }
        
        /**
         * similar_update_url
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_update_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_update, null, data, null);

            return result;
        }
        
        /**
         * similar_update_cont_sign
         * **更新图库中图片的摘要和分类信息（具体变量为brief、tags）**

         * @param cont_sign 图片签名
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_update_cont_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_update, null, data, null);

            return result;
        }
        
        /**
         * similar_delete_by_image
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value similar_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_delete, null, data, null);

            return result;
        }
        
        /**
         * similar_delete_by_url
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         */
        Json::Value similar_delete_by_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_delete, null, data, null);

            return result;
        }
        
        /**
         * similar_delete_by_sign
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param cont_sign 图片签名
         * options 可选参数:
         */
        Json::Value similar_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_delete, null, data, null);

            return result;
        }
        
        /**
         * product_add
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）。同时可提交分类维度信息（具体变量为class_id1、class_id2），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。**请注意，检索接口不返回原图，仅反馈当前填写的brief信息，所以调用该入库接口时，brief信息请尽量填写可关联至本地图库的图片id或者图片url、图片名称等信息**
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         */
        Json::Value product_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_add, null, data, null);

            return result;
        }
        
        /**
         * product_add_url
         * **该接口实现单张图片入库，入库时需要同步提交图片及可关联至本地图库的摘要信息（具体变量为brief，具体可传入图片在本地标记id、图片url、图片名称等）。同时可提交分类维度信息（具体变量为class_id1、class_id2），方便对图库中的图片进行管理、分类检索。****注：重复添加完全相同的图片会返回错误。**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 检索时原样带回,最长256B。**请注意，检索接口不返回原图，仅反馈当前填写的brief信息，所以调用该入库接口时，brief信息请尽量填写可关联至本地图库的图片id或者图片url、图片名称等信息**
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         */
        Json::Value product_add_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_add, null, data, null);

            return result;
        }
        
        /**
         * product_search
         * 完成入库后，可使用该接口实现商品检索。**支持传入指定分类维度（具体变量class_id1、class_id2）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value product_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_search, null, data, null);

            return result;
        }
        
        /**
         * product_search_url
         * 完成入库后，可使用该接口实现商品检索。**支持传入指定分类维度（具体变量class_id1、class_id2）进行检索，返回结果支持翻页（具体变量pn、rn）。****请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息**
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value product_search_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_search, null, data, null);

            return result;
        }
        
        /**
         * product_update
         * **更新图库中图片的摘要和分类信息（具体变量为brief、class_id1/class_id2）**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * class_id1 更新的商品分类1，支持1-60范围内的整数。
         * class_id2 更新的商品分类2，支持1-60范围内的整数。
         */
        Json::Value product_update(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_update, null, data, null);

            return result;
        }
        
        /**
         * product_update_url
         * **更新图库中图片的摘要和分类信息（具体变量为brief、class_id1/class_id2）**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * class_id1 更新的商品分类1，支持1-60范围内的整数。
         * class_id2 更新的商品分类2，支持1-60范围内的整数。
         */
        Json::Value product_update_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_update, null, data, null);

            return result;
        }
        
        /**
         * product_update_cont_sign
         * **更新图库中图片的摘要和分类信息（具体变量为brief、class_id1/class_id2）**

         * @param cont_sign 图片签名
         * options 可选参数:
         * brief 更新的摘要信息，最长256B。样例：{"name":"周杰伦", "id":"666"}
         * class_id1 更新的商品分类1，支持1-60范围内的整数。
         * class_id2 更新的商品分类2，支持1-60范围内的整数。
         */
        Json::Value product_update_cont_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_update, null, data, null);

            return result;
        }
        
        /**
         * product_delete_by_image
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value product_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_delete, null, data, null);

            return result;
        }
        
        /**
         * product_delete_by_url
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效
         * options 可选参数:
         */
        Json::Value product_delete_by_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_delete, null, data, null);

            return result;
        }
        
        /**
         * product_delete_by_sign
         * **删除图库中的图片，支持批量删除，批量删除时请传cont_sign参数，勿传image，最多支持1000个cont_sign**

         * @param cont_sign 图片签名
         * options 可选参数:
         */
        Json::Value product_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_delete, null, data, null);

            return result;
        }
        

    };
}
#endif