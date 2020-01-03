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

#ifndef __AIP_KG_H__
#define __AIP_KG_H__

#include "base/base.h"

namespace aip {

    class Kg: public AipBase
    {
    public:

    
        std::string _create_task =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_create";
        
        std::string _update_task =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_update";
        
        std::string _task_info =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_info";
        
        std::string _task_query =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_query";
        
        std::string _task_start =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_start";
        
        std::string _task_status =
            "https://aip.baidubce.com/rest/2.0/kg/v1/pie/task_status";
        

        Kg(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * create_task
         * 创建一个新的信息抽取任务
         * @param name 任务名字
         * @param template_content json string 解析模板内容
         * @param input_mapping_file 抓取结果映射文件的路径
         * @param output_file 输出文件名字
         * @param url_pattern url pattern
         * options 可选参数:
         * limit_count 限制解析数量limit_count为0时进行全量任务，limit_count>0时只解析limit_count数量的页面
         */
        Json::Value create_task(
            std::string const & name,
            std::string const & template_content,
            std::string const & input_mapping_file,
            std::string const & output_file,
            std::string const & url_pattern,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["name"] = name;
            data["template_content"] = template_content;
            data["input_mapping_file"] = input_mapping_file;
            data["output_file"] = output_file;
            data["url_pattern"] = url_pattern;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_create_task, null, data, null);

            return result;
        }
        
        /**
         * update_task
         * 更新任务配置，在任务重新启动后生效
         * @param id 任务ID
         * options 可选参数:
         * name 任务名字
         * template_content json string 解析模板内容
         * input_mapping_file 抓取结果映射文件的路径
         * url_pattern url pattern
         * output_file 输出文件名字
         */
        Json::Value update_task(
            const int & id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["id"] =  std::to_string(id);

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_update_task, null, data, null);

            return result;
        }
        
        /**
         * task_info
         * 根据任务id获取单个任务的详细信息
         * @param id 任务ID
         * options 可选参数:
         */
        Json::Value task_info(
            const int & id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["id"] =  std::to_string(id);

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_task_info, null, data, null);

            return result;
        }
        
        /**
         * task_query
         * 该请求用于菜品识别。即对于输入的一张图片（可正常解码，且长宽比适宜），输出图片的菜品名称、卡路里信息、置信度。
         * options 可选参数:
         * id 任务ID，精确匹配
         * name 中缀模糊匹配,abc可以匹配abc,aaabc,abcde等
         * status 要筛选的任务状态
         * page 页码
         * per_page 页码
         */
        Json::Value task_query(
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_task_query, null, data, null);

            return result;
        }
        
        /**
         * task_start
         * 启动一个已经创建的信息抽取任务
         * @param id 任务ID
         * options 可选参数:
         */
        Json::Value task_start(
            const int & id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["id"] =  std::to_string(id);

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_task_start, null, data, null);

            return result;
        }
        
        /**
         * task_status
         * 查询指定的任务的最新执行状态
         * @param id 任务ID
         * options 可选参数:
         */
        Json::Value task_status(
            const int & id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["id"] =  std::to_string(id);

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_task_status, null, data, null);

            return result;
        }
        

    };
}
#endif