// Serialization - 序列化支持，把对象转字符串或者二进制

#pragma once

#include <string>

namespace AstraAlgebra {

namespace Serialization {
    
    // 序列化格式
    enum class Format {
        String,  // 字符串格式
        Binary   // 二进制格式
    };
    
    // 序列化
    template <typename T>
    std::string serialize(const T& value, Format format = Format::String);
    
    // 反序列化
    template <typename T>
    T deserialize(const std::string& data, Format format = Format::String);
    
    // 二进制序列化辅助
    template <typename T>
    void appendBinary(std::string& data, const T& value);  // 追加到二进制数据
    
    // 从二进制数据读
    template <typename T>
    T readBinary(const std::string& data, size_t& offset);  // offset是偏移量
    
} // namespace Serialization

} // namespace AstraAlgebra
