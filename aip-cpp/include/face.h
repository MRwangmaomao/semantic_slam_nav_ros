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

#ifndef __AIP_FACE_H__
#define __AIP_FACE_H__

#include "base/base.h"

namespace aip {

    class Face: public AipBase
    {
    public:
        
        std::string _detect =
            "https://aip.baidubce.com/rest/2.0/face/v3/detect";
        
        std::string _search =
            "https://aip.baidubce.com/rest/2.0/face/v3/search";
        
        std::string _multi_search =
            "https://aip.baidubce.com/rest/2.0/face/v3/multi-search";
        
        std::string _user_add =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/add";
        
        std::string _user_update =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/update";
        
        std::string _face_delete =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/face/delete";
        
        std::string _user_get =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/get";
        
        std::string _face_getlist =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/face/getlist";
        
        std::string _group_getusers =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/group/getusers";
        
        std::string _user_copy =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/copy";
        
        std::string _user_delete =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/user/delete";
        
        std::string _group_add =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/group/add";
        
        std::string _group_delete =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/group/delete";
        
        std::string _group_getlist =
            "https://aip.baidubce.com/rest/2.0/face/v3/faceset/group/getlist";
        
        std::string _person_verify =
            "https://aip.baidubce.com/rest/2.0/face/v3/person/verify";
        
        std::string _video_sessioncode =
            "https://aip.baidubce.com/rest/2.0/face/v1/faceliveness/sessioncode";
        

        Face(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }

        
        /**
         * detect
         * **人脸检测**：检测图片中的人脸并标记出位置信息;

         * @param image 图片信息(**总数据大小应小于10M**)，图片上传方式根据image_type来判断
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * options 可选参数:
         * face_field 包括**age,beauty,expression,face_shape,gender,glasses,landmark,landmark72，landmark150，race,quality,eye_status,emotion,face_type信息**  <br> 逗号分隔. 默认只返回face_token、人脸框、概率和旋转角度
         * max_face_num 最多处理人脸的数目，默认值为1，仅检测图片中面积最大的那个人脸；**最大值10**，检测图片中面积最大的几张人脸。
         * face_type 人脸的类型 **LIVE**表示生活照：通常为手机、相机拍摄的人像图片、或从网络获取的人像图片等**IDCARD**表示身份证芯片照：二代身份证内置芯片中的人像照片 **WATERMARK**表示带水印证件照：一般为带水印的小图，如公安网小图 **CERT**表示证件照片：如拍摄的身份证、工卡、护照、学生证等证件图片 默认**LIVE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         */
        Json::Value detect(
            std::string const & image,
            std::string const & image_type,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_detect, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * search
         * * **1：N人脸搜索**：也称为1：N识别，在指定人脸集合中，找到最相似的人脸；
* **1：N人脸认证**：基于uid维度的1：N识别，由于uid已经锁定固定数量的人脸，所以检索范围更聚焦；

> **1：N人脸识别**与**1：N人脸认证**的差别在于：人脸搜索是在指定人脸集合中进行直接地人脸检索操作，而人脸认证是基于uid，先调取这个uid对应的人脸，再在这个uid对应的人脸集合中进行检索（因为每个uid通常对应的只有一张人脸，所以通常也就变为了1：1对比）；实际应用中，人脸认证需要用户或系统先输入id，这增加了验证安全度，但也增加了复杂度，具体使用哪个接口需要视您的业务场景判断。

         * @param image 图片信息(**总数据大小应小于10M**)，图片上传方式根据image_type来判断
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * @param group_id_list 从指定的group中进行查找 用逗号分隔，**上限20个**
         * options 可选参数:
         * max_face_num 最多处理人脸的数目<br>**默认值为1(仅检测图片中面积最大的那个人脸)** **最大值10**
         * match_threshold 匹配阈值（设置阈值后，score低于此阈值的用户信息将不会返回） 最大100 最小0 默认80 <br>**此阈值设置得越高，检索速度将会越快，推荐使用默认阈值`80`**
         * quality_control 图片质量控制  **NONE**: 不进行控制 **LOW**:较低的质量要求 **NORMAL**: 一般的质量要求 **HIGH**: 较高的质量要求 **默认 NONE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         * user_id 当需要对特定用户进行比对时，指定user_id进行比对。即人脸认证功能。
         * max_user_num 查找后返回的用户数量。返回相似度最高的几个用户，默认为1，最多返回50个。
         */
        Json::Value search(
            std::string const & image,
            std::string const & image_type,
            std::string const & group_id_list,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;
            data["group_id_list"] = group_id_list;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_search, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * multi_search
         * 待识别的图片中，存在多张人脸的情况下，支持在一个人脸库中，一次请求，同时返回图片中所有人脸的识别结果。
         * @param image 图片信息(**总数据大小应小于10M**)，图片上传方式根据image_type来判断
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * @param group_id_list 从指定的group中进行查找 用逗号分隔，**上限20个**
         * options 可选参数:
         * max_face_num 最多处理人脸的数目<br>**默认值为1(仅检测图片中面积最大的那个人脸)** **最大值10**
         * match_threshold 匹配阈值（设置阈值后，score低于此阈值的用户信息将不会返回） 最大100 最小0 默认80 <br>**此阈值设置得越高，检索速度将会越快，推荐使用默认阈值`80`**
         * quality_control 图片质量控制  **NONE**: 不进行控制 **LOW**:较低的质量要求 **NORMAL**: 一般的质量要求 **HIGH**: 较高的质量要求 **默认 NONE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         * max_user_num 查找后返回的用户数量。返回相似度最高的几个用户，默认为1，最多返回50个。
         */
        Json::Value multi_search(
            std::string const & image,
            std::string const & image_type,
            std::string const & group_id_list,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;
            data["group_id_list"] = group_id_list;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_multi_search, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * user_add
         * 用于从人脸库中新增用户，可以设定多个用户所在组，及组内用户的人脸图片，

典型应用场景：构建您的人脸库，如**会员人脸注册**，**已有用户补全人脸信息**等。

人脸库、用户组、用户、用户下的人脸**层级关系**如下所示：

```json
|- 人脸库
   |- 用户组一
      |- 用户01
         |- 人脸
      |- 用户02
         |- 人脸
         |- 人脸
         ....
       ....
   |- 用户组二
   |- 用户组三
   |- 用户组四
   ....
```

**关于人脸库的设置限制**

* 每个开发者账号可以创建100个appid；
* **每个appid对应一个人脸库，且不同appid之间，人脸库互不相通**；
* 每个人脸库下，可以创建多个用户组，用户组（group）数量**没有限制**；
* 每个用户组（group）下，可添加最多**无限**张人脸，**无限**个uid；
* 每个用户（uid）所能注册的最大人脸数量**没有限制**；

为了保证识别效果，请控制注册人脸的质量（通过`/detect`人脸检测接口判断），具体参数可详见下表所示：

**质量判断**

可通过人脸检测接口，基于以下字段和对应阈值，进行质量检测的判断，以保证人脸质量符合后续业务操作要求。

|   指标   |   字段与解释   |  推荐数值界限   |
| -------- | ------------- | ------------- |
| **遮挡范围**  |  **occlusion**（0~1），0为无遮挡，1是完全遮挡<br/>含有多个具体子字段，表示脸部多个部位<br/>通常用作判断头发、墨镜、口罩等遮挡 | left\_eye : 0.6, #左眼被遮挡的阈值<br/>right\_eye : 0.6, #右眼被遮挡的阈值<br/>nose : 0.7, #鼻子被遮挡的阈值<br/>mouth : 0.7, #嘴巴被遮挡的阈值<br/>left\_check : 0.8, #左脸颊被遮挡的阈值<br/>right\_check : 0.8, #右脸颊被遮挡的阈值<br/>chin\_contour : 0.6, #下巴被遮挡阈值  |
| **模糊度范围** | **Blur**（0~1），0是最清晰，1是最模糊 | 小于0.7 |
| **光照范围** | **illumination**（0~255）<br/>脸部光照的灰度值，0表示光照不好<br/>以及对应客户端SDK中，YUV的Y分量| 大于40 |
| **姿态角度** | **Pitch**：三维旋转之俯仰角度[-90(上), 90(下)]<br/>**Roll**：平面内旋转角[-180(逆时针), 180(顺时针)]<br/>**Yaw**：三维旋转之左右旋转角[-90(左), 90(右)] | 分别小于20度 |
| **人脸完整度** | **completeness**（0或1），0为人脸溢出图像边界，1为人脸都在图像边界内 | 视业务逻辑判断 |
| **人脸大小** | 人脸部分的大小<br/>建议长宽像素值范围：80\*80~200\*200 | 人脸部分不小于**100\*100**像素 |

         * @param image 图片信息(总数据大小应小于10M)，图片上传方式根据image_type来判断。注：组内每个uid下的人脸图片数目上限为20张
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * user_info 用户资料，长度限制256B
         * quality_control 图片质量控制  **NONE**: 不进行控制 **LOW**:较低的质量要求 **NORMAL**: 一般的质量要求 **HIGH**: 较高的质量要求 **默认 NONE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         * action_type 操作方式  APPEND: 当user_id在库中已经存在时，对此user_id重复注册时，新注册的图片默认会追加到该user_id下,REPLACE : 当对此user_id重复注册时,则会用新图替换库中该user_id下所有图片,默认使用APPEND
         */
        Json::Value user_add(
            std::string const & image,
            std::string const & image_type,
            std::string const & group_id,
            std::string const & user_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;
            data["group_id"] = group_id;
            data["user_id"] = user_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_user_add, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * user_update
         * 用于对人脸库中指定用户，更新其下的人脸图像。

> **说明：**针对一个uid执行更新操作，新上传的人脸图像将覆盖此uid原有所有图像。

         * @param image 图片信息(**总数据大小应小于10M**)，图片上传方式根据image_type来判断
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * @param group_id 更新指定groupid下uid对应的信息
         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * user_info 用户资料，长度限制256B
         * quality_control 图片质量控制  **NONE**: 不进行控制 **LOW**:较低的质量要求 **NORMAL**: 一般的质量要求 **HIGH**: 较高的质量要求 **默认 NONE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         * action_type 操作方式  APPEND: 当user_id在库中已经存在时，对此user_id重复注册时，新注册的图片默认会追加到该user_id下,REPLACE : 当对此user_id重复注册时,则会用新图替换库中该user_id下所有图片,默认使用APPEND
         */
        Json::Value user_update(
            std::string const & image,
            std::string const & image_type,
            std::string const & group_id,
            std::string const & user_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;
            data["group_id"] = group_id;
            data["user_id"] = user_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_user_update, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * face_delete
         * 用于从人脸库中删除一个用户。

**人脸删除注意事项：**

* 删除的内容，包括用户所有图像和身份信息；
* 如果一个uid存在于多个用户组内，将会同时将从各个组中把用户删除
* 如果指定了group_id，则只删除此group下的uid相关信息

         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * @param face_token 需要删除的人脸图片token，（由数字、字母、下划线组成）长度限制64B
         * options 可选参数:
         */
        Json::Value face_delete(
            std::string const & user_id,
            std::string const & group_id,
            std::string const & face_token,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["user_id"] = user_id;
            data["group_id"] = group_id;
            data["face_token"] = face_token;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_face_delete, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * user_get
         * 获取人脸库中某个用户的信息(user_info信息和用户所属的组)。

         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value user_get(
            std::string const & user_id,
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["user_id"] = user_id;
            data["group_id"] = group_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_user_get, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * face_getlist
         * 用于获取一个用户的全部人脸列表。

         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value face_getlist(
            std::string const & user_id,
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["user_id"] = user_id;
            data["group_id"] = group_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_face_getlist, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * group_getusers
         * 用于查询指定用户组中的用户列表。

         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * start 默认值0，起始序号
         * length 返回数量，默认值100，最大值1000
         */
        Json::Value group_getusers(
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["group_id"] = group_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_group_getusers, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * user_copy
         * 用于将已经存在于人脸库中的用户**复制到一个新的组**。

         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * src_group_id 从指定组里复制信息
         * dst_group_id 需要添加用户的组id
         */
        Json::Value user_copy(
            std::string const & user_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["user_id"] = user_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_user_copy, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * user_delete
         * 用于将用户从某个组中删除。

         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * @param user_id 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value user_delete(
            std::string const & group_id,
            std::string const & user_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["group_id"] = group_id;
            data["user_id"] = user_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_user_delete, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * group_add
         * 用于创建一个空的用户组，如果用户组已存在 则返回错误。

         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value group_add(
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["group_id"] = group_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_group_add, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * group_delete
         * 删除用户组下所有的用户及人脸，如果组不存在 则返回错误。

         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value group_delete(
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["group_id"] = group_id;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_group_delete, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * group_getlist
         * 用于查询用户组的列表。

         * options 可选参数:
         * start 默认值0，起始序号
         * length 返回数量，默认值100，最大值1000
         */
        Json::Value group_getlist(
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_group_getlist, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * person_verify
         * 质量检测（可选）活体检测（可选）公安验证（必选）
         * @param image 图片信息(**总数据大小应小于10M**)，图片上传方式根据image_type来判断
         * @param image_type 图片类型     <br> **BASE64**:图片的base64值，base64编码后的图片数据，编码后的图片大小不超过2M； <br>**URL**:图片的 URL地址( 可能由于网络等原因导致下载图片时间过长)； <br>**FACE_TOKEN**: 人脸图片的唯一标识，调用人脸检测接口时，会为每个人脸图片赋予一个唯一的FACE_TOKEN，同一张图片多次检测得到的FACE_TOKEN是同一个。
         * @param id_card_number 身份证号（真实身份证号号码）
         * @param name utf8，姓名（真实姓名，和身份证号匹配）
         * options 可选参数:
         * quality_control 图片质量控制  **NONE**: 不进行控制 **LOW**:较低的质量要求 **NORMAL**: 一般的质量要求 **HIGH**: 较高的质量要求 **默认 NONE**
         * liveness_control 活体检测控制  **NONE**: 不进行控制 **LOW**:较低的活体要求(高通过率 低攻击拒绝率) **NORMAL**: 一般的活体要求(平衡的攻击拒绝率, 通过率) **HIGH**: 较高的活体要求(高攻击拒绝率 低通过率) **默认NONE**
         */
        Json::Value person_verify(
            std::string const & image,
            std::string const & image_type,
            std::string const & id_card_number,
            std::string const & name,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["image"] = image;
            data["image_type"] = image_type;
            data["id_card_number"] = id_card_number;
            data["name"] = name;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_person_verify, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * video_sessioncode
         * 此接口主要用于生成随机码，用于视频的语音识别校验使用，以判断视频的即时性，而非事先录制的，提升作弊的难度。
         * options 可选参数:
         * appid 百度云创建应用时的唯一标识ID
         */
        Json::Value video_sessioncode(
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_video_sessioncode, null, data.toStyledString(), null);

            return result;
        }
        /**
         * faceverify
         * @param data 参数对象数组
         *
         */
        Json::Value faceverify(
            Json::Value const & data)
        {
            std::string _faceverify =
                "https://aip.baidubce.com/rest/2.0/face/v3/faceverify";
            Json::Value result =
                this->request(_faceverify, null, data.toStyledString(), null);

            return result;
        }

        /**
         * match
         * @param data 参数对象数组
         *
         */
        Json::Value match(
            Json::Value const & data)
        {
            std::string _match =
                "https://aip.baidubce.com/rest/2.0/face/v3/match";
            Json::Value result =
                this->request(_match, null, data.toStyledString(), null);

            return result;
        }

    };
}
#endif