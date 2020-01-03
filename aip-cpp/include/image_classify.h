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

#ifndef __AIP_IMAGECLASSIFY_H__
#define __AIP_IMAGECLASSIFY_H__

#include "base/base.h"

namespace aip {

    class Imageclassify: public AipBase
    {
    public:

    
        std::string _advanced_general =
            "https://aip.baidubce.com/rest/2.0/image-classify/v2/advanced_general";
        
        std::string _dish_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v2/dish";
        
        std::string _car_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/car";
        
        std::string _vehicle_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/vehicle_detect";
        
        std::string _vehicle_damage =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/vehicle_damage";
        
        std::string _logo_search =
            "https://aip.baidubce.com/rest/2.0/image-classify/v2/logo";
        
        std::string _logo_add =
            "https://aip.baidubce.com/rest/2.0/realtime_search/v1/logo/add";
        
        std::string _logo_delete =
            "https://aip.baidubce.com/rest/2.0/realtime_search/v1/logo/delete";
        
        std::string _animal_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/animal";
        
        std::string _plant_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/plant";
        
        std::string _object_detect =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/object_detect";
        
        std::string _landmark =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/landmark";
        
        std::string _flower =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/flower";
        
        std::string _ingredient =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/classify/ingredient";
        
        std::string _redwine =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/redwine";
        
        std::string _currency =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/currency";
        

        Imageclassify(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * advanced_general
         * 该请求用于通用物体及场景识别，即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片中的多个物体及场景标签。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value advanced_general(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_advanced_general, null, data, null);

            return result;
        }
        
        /**
         * dish_detect
         * 该请求用于菜品识别。即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片的菜品名称、卡路里信息、置信度。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * top_num 返回预测得分top结果数，默认为5
         * filter_threshold 默认0.95，可以通过该参数调节识别效果，降低非菜识别率.
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value dish_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_dish_detect, null, data, null);

            return result;
        }
        
        /**
         * car_detect
         * 该请求用于检测一张车辆图片的具体车型。即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片的车辆品牌及型号。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * top_num 返回预测得分top结果数，默认为5
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value car_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_car_detect, null, data, null);

            return result;
        }
        
        /**
         * vehicle_detect
         * 传入单帧图像，**检测图片中所有机动车辆，返回每辆车的类型和坐标位置**，可识别小汽车、卡车、巴士、摩托车、三轮车5大类车辆，**并对每类车辆分别计数，可返回含有统计值和检测框的渲染结果图**，支持指定不规则区域的车辆统计。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * show 是否返回结果图（含统计值和跟踪框）。选true时返回渲染后的图片(base64)，其它无效值或为空则默认false。
         * area 只统计该区域内的车辆数，缺省时为全图统计。<br>逗号分隔，如‘x1,y1,x2,y2,x3,y3...xn,yn'，按顺序依次给出每个顶点的x、y坐标（默认尾点和首点相连），形成闭合多边形区域。<br>服务会做范围（顶点左边需在图像范围内）及个数校验（数组长度必须为偶数，且大于3个顶点）。只支持单个多边形区域，建议设置矩形框，即4个顶点。**坐标取值不能超过图像宽度和高度，比如1280的宽度，坐标值最大到1279**。
         */
        Json::Value vehicle_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_detect, null, data, null);

            return result;
        }
        
        /**
         * vehicle_damage
         * 针对常见的小汽车车型，传入单帧图像，识别车辆外观受损部件及损伤类型，支持32种车辆部件、5大类外观损伤。同时可输出损伤的数值化结果（长宽、面积、部件占比），支持单图多种损伤的识别。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value vehicle_damage(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_damage, null, data, null);

            return result;
        }
        
        /**
         * logo_search
         * 该请求用于检测和识别图片中的品牌LOGO信息。即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片中LOGO的名称、位置和置信度。当效果欠佳时，可以建立子库（在[控制台](https://console.bce.baidu.com/ai/#/ai/imagerecognition/overview/index)创建应用并申请建库）并通过调用logo入口接口完成自定义logo入库，提高识别效果。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * custom_lib 是否只使用自定义logo库的结果，默认false：返回自定义库+默认库的识别结果
         */
        Json::Value logo_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_logo_search, null, data, null);

            return result;
        }
        
        /**
         * logo_add
         * 使用入库接口请先在[控制台](https://console.bce.baidu.com/ai/#/ai/imagerecognition/overview/index)创建应用并申请建库，建库成功后方可正常使用。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * @param brief brief，检索时带回。此处要传对应的name与code字段，name长度小于100B，code长度小于150B
         * options 可选参数:
         */
        Json::Value logo_add(
            std::string const & image,
            std::string const & brief,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());
            data["brief"] = brief;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_logo_add, null, data, null);

            return result;
        }
        
        /**
         * logo_delete_by_image
         * 使用删除接口请先在[控制台](https://console.bce.baidu.com/ai/#/ai/imagerecognition/overview/index)创建应用并申请建库，建库成功后先调用入库接口完成logo图片入库，删除接口用户在已入库的logo图片中删除图片。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value logo_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_logo_delete, null, data, null);

            return result;
        }
        
        /**
         * logo_delete_by_sign
         * 使用删除接口请先在[控制台](https://console.bce.baidu.com/ai/#/ai/imagerecognition/overview/index)创建应用并申请建库，建库成功后先调用入库接口完成logo图片入库，删除接口用户在已入库的logo图片中删除图片。
         * @param cont_sign 图片签名（和image二选一，image优先级更高）
         * options 可选参数:
         */
        Json::Value logo_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_logo_delete, null, data, null);

            return result;
        }
        
        /**
         * animal_detect
         * 该请求用于识别一张图片。即对于输入的一张图片（可正常解码，且长宽比适宜），输出动物识别结果
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * top_num 返回预测得分top结果数，默认为6
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value animal_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_animal_detect, null, data, null);

            return result;
        }
        
        /**
         * plant_detect
         * 该请求用于识别一张图片。即对于输入的一张图片（可正常解码，且长宽比适宜），输出植物识别结果。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value plant_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_plant_detect, null, data, null);

            return result;
        }
        
        /**
         * object_detect
         * 用户向服务请求检测图像中的主体位置。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * with_face 如果检测主体是人，主体区域是否带上人脸部分，0-不带人脸区域，其他-带人脸区域，裁剪类需求推荐带人脸，检索/识别类需求推荐不带人脸。默认取1，带人脸。
         */
        Json::Value object_detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_object_detect, null, data, null);

            return result;
        }
        
        /**
         * landmark
         * 该请求用于识别地标，即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片中的地标识别结果。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value landmark(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_landmark, null, data, null);

            return result;
        }
        
        /**
         * flower
         * 检测用户上传的花卉图片，输出图片的花卉识别结果名称及对应的概率打分。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * top_num 返回预测得分top结果数，默认为5
         * baike_num 返回百科信息的结果数，默认不返回
         */
        Json::Value flower(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_flower, null, data, null);

            return result;
        }
        
        /**
         * ingredient
         * 该请求用于识别果蔬类食材，即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片中的果蔬食材结果。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * top_num 返回预测得分top结果数，如果为空或小于等于0默认为5；如果大于20默认20
         */
        Json::Value ingredient(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_ingredient, null, data, null);

            return result;
        }
        
        /**
         * redwine
         * 该服务用于识别红酒标签，即对于输入的一张图片（可正常解码，长宽比适宜，且酒标清晰可见），输出图片中的红酒名称、国家、产区、酒庄、类型、糖分、葡萄品种、酒品描述等信息。可识别数十万中外常见红酒。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value redwine(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_redwine, null, data, null);

            return result;
        }
        
        /**
         * currency
         * 识别图像中的货币类型，以纸币为主，正反面均可准确识别，接口返回货币的名称、代码、面值、年份信息；可识别各类近代常见货币，如美元、欧元、英镑、法郎、澳大利亚元、俄罗斯卢布、日元、韩元、泰铢、印尼卢比等。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value currency(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_currency, null, data, null);

            return result;
        }
        

    };
}
#endif