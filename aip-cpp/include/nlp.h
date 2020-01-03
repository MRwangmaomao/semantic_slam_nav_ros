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

#ifndef __AIP_NLP_H__
#define __AIP_NLP_H__

#include "base/base.h"

namespace aip {

    class Nlp: public AipBase
    {
    public:
        
        std::string _lexer =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/lexer";
        
        std::string _lexer_custom =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/lexer_custom";
        
        std::string _dep_parser =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/depparser";
        
        std::string _word_embedding =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/word_emb_vec";
        
        std::string _dnnlm_cn =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/dnnlm_cn";
        
        std::string _word_sim_embedding =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/word_emb_sim";
        
        std::string _simnet =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/simnet";
        
        std::string _comment_tag =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/comment_tag";
        
        std::string _sentiment_classify =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/sentiment_classify";
        
        std::string _keyword =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/keyword";
        
        std::string _topic =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/topic";
        
        std::string _ecnet =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/ecnet";
        
        std::string _emotion =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/emotion";
        
        std::string _news_summary =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/news_summary";
        
        std::string _address =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/address";
        

        Nlp(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }

        
        /**
         * lexer
         * 词法分析接口向用户提供分词、词性标注、专名识别三大功能；能够识别出文本串中的基本词汇（分词），对这些词汇进行重组、标注组合后词汇的词性，并进一步识别出命名实体。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过65536字节
         * options 可选参数:
         */
        Json::Value lexer(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_lexer, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * lexer_custom
         * 词法分析接口向用户提供分词、词性标注、专名识别三大功能；能够识别出文本串中的基本词汇（分词），对这些词汇进行重组、标注组合后词汇的词性，并进一步识别出命名实体。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过65536字节
         * options 可选参数:
         */
        Json::Value lexer_custom(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_lexer_custom, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * dep_parser
         * 依存句法分析接口可自动分析文本中的依存句法结构信息，利用句子中词与词之间的依存关系来表示词语的句法结构信息（如“主谓”、“动宾”、“定中”等结构关系），并用树状结构来表示整句的结构（如“主谓宾”、“定状补”等）。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过256字节
         * options 可选参数:
         * mode 模型选择。默认值为0，可选值mode=0（对应web模型）；mode=1（对应query模型）
         */
        Json::Value dep_parser(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_dep_parser, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * word_embedding
         * 词向量表示接口提供中文词向量的查询功能。
         * @param word 文本内容（UTF8编码），最大64字节
         * options 可选参数:
         */
        Json::Value word_embedding(
            std::string const & word,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["word"] = word;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_word_embedding, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * dnnlm_cn
         * 中文DNN语言模型接口用于输出切词结果并给出每个词在句子中的概率值,判断一句话是否符合语言表达习惯。
         * @param text 文本内容（UTF8编码），最大512字节，不需要切词
         * options 可选参数:
         */
        Json::Value dnnlm_cn(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_dnnlm_cn, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * word_sim_embedding
         * 输入两个词，得到两个词的相似度结果。
         * @param word_1 词1（UTF8编码），最大64字节
         * @param word_2 词1（UTF8编码），最大64字节
         * options 可选参数:
         * mode 预留字段，可选择不同的词义相似度模型。默认值为0，目前仅支持mode=0
         */
        Json::Value word_sim_embedding(
            std::string const & word_1,
            std::string const & word_2,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["word_1"] = word_1;
            data["word_2"] = word_2;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_word_sim_embedding, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * simnet
         * 短文本相似度接口用来判断两个文本的相似度得分。
         * @param text_1 待比较文本1（UTF8编码），最大512字节
         * @param text_2 待比较文本2（UTF8编码），最大512字节
         * options 可选参数:
         * model 默认为"BOW"，可选"BOW"、"CNN"与"GRNN"
         */
        Json::Value simnet(
            std::string const & text_1,
            std::string const & text_2,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text_1"] = text_1;
            data["text_2"] = text_2;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_simnet, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * comment_tag
         * 评论观点抽取接口用来提取一条评论句子的关注点和评论观点，并输出评论观点标签及评论观点极性。
         * @param text 评论内容（UTF8编码），最大10240字节
         * options 可选参数:
         * type 评论行业类型，默认为4（餐饮美食）
         */
        Json::Value comment_tag(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_comment_tag, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * sentiment_classify
         * 对包含主观观点信息的文本进行情感极性类别（积极、消极、中性）的判断，并给出相应的置信度。
         * @param text 文本内容（UTF8编码），最大102400字节
         * options 可选参数:
         */
        Json::Value sentiment_classify(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_sentiment_classify, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * keyword
         * 文章标签服务能够针对网络各类媒体文章进行快速的内容理解，根据输入含有标题的文章，输出多个内容标签以及对应的置信度，用于个性化推荐、相似文章聚合、文本内容分析等场景。
         * @param title 篇章的标题，最大80字节
         * @param content 篇章的正文，最大65535字节
         * options 可选参数:
         */
        Json::Value keyword(
            std::string const & title,
            std::string const & content,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["title"] = title;
            data["content"] = content;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_keyword, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * topic
         * 对文章按照内容类型进行自动分类，首批支持娱乐、体育、科技等26个主流内容类型，为文章聚类、文本内容分析等应用提供基础技术支持。
         * @param title 篇章的标题，最大80字节
         * @param content 篇章的正文，最大65535字节
         * options 可选参数:
         */
        Json::Value topic(
            std::string const & title,
            std::string const & content,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["title"] = title;
            data["content"] = content;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_topic, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * ecnet
         * 识别输入文本中有错误的片段，提示错误并给出正确的文本结果。支持短文本、长文本、语音等内容的错误识别，纠错是搜索引擎、语音识别、内容审查等功能更好运行的基础模块之一。
         * @param text 待纠错文本，输入限制511字节
         * options 可选参数:
         */
        Json::Value ecnet(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_ecnet, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * emotion
         * 针对用户日常沟通文本背后所蕴含情绪的一种直观检测，可自动识别出当前会话者所表现出的情绪类别及其置信度，可以帮助企业更全面地把握产品服务质量、监控客户服务质量
         * @param text 待识别情感文本，输入限制512字节
         * options 可选参数:
         * scene default（默认项-不区分场景），talk（闲聊对话-如度秘聊天等），task（任务型对话-如导航对话等），customer_service（客服对话-如电信/银行客服等）
         */
        Json::Value emotion(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_emotion, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * news_summary
         * 自动抽取新闻文本中的关键信息，进而生成指定长度的新闻摘要
         * @param content 字符串（限3000字符数以内）字符串仅支持UTF8编码，长度需小于3000字符数（即6000字节），请输入前确认字符数没有超限，若字符数超长会返回错误。正文中如果包含段落信息，请使用"\n"分隔，段落信息算法中有重要的作用，请尽量保留
         * @param max_summary_len 此数值将作为摘要结果的最大长度。例如：原文长度1000字，本参数设置为150，则摘要结果的最大长度是150字；推荐最优区间：200-500字
         * options 可选参数:
         * title 字符串（限200字符数）字符串仅支持UTF8编码，长度需小于200字符数（即400字节），请输入前确认字符数没有超限，若字符数超长会返回错误。标题在算法中具有重要的作用，若文章确无标题，输入参数的“标题”字段为空即可
         */
        Json::Value news_summary(
            std::string const & content,
            const int & max_summary_len,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["content"] = content;
            data["max_summary_len"] = max_summary_len;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_news_summary, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * address
         * 针对快递、电商行业中客户在线提交的大量非结构化地址单据，该接口可以帮助精准提取快递填单文本中的姓名、电话、地址信息，通过自然语言处理辅助地址识别做自动补充和纠正，生成标准规范的结构化信息，大幅提升企业处理单据的效率。
         * @param text 待识别的文本内容，不超过1000字节
         * options 可选参数:
         */
        Json::Value address(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_address, null, data.toStyledString(), null);

            return result;
        }
        
    };
}
#endif