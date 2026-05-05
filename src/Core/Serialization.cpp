// Serialization.cpp - 序列化实现
// 把对象转字符串或者二进制

#include <AstraAlgebra/Core/Serialization.h>
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix2x2.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <array>
#include <sstream>
#include <iomanip>

namespace AstraAlgebra {

namespace Serialization {
    
    // 二进制序列化辅助函数实现
    template <typename T>
    void appendBinary(std::string& data, const T& value) {
        const char* bytes = reinterpret_cast<const char*>(&value);
        data.append(bytes, sizeof(T));
    }
    
    template <typename T>
    T readBinary(const std::string& data, size_t& offset) {
        T value = *reinterpret_cast<const T*>(data.data() + offset);
        offset += sizeof(T);
        return value;
    }
    
    // 显式实例化常用类型的二进制序列化辅助函数
    template void appendBinary(std::string&, const float&);
    template void appendBinary(std::string&, const double&);
    template void appendBinary(std::string&, const int&);
    template void appendBinary(std::string&, const unsigned int&);
    template void appendBinary(std::string&, const bool&);
    
    template float readBinary(const std::string&, size_t&);
    template double readBinary(const std::string&, size_t&);
    template int readBinary(const std::string&, size_t&);
    template unsigned int readBinary(const std::string&, size_t&);
    template bool readBinary(const std::string&, size_t&);
    
    // Vector2 序列化
    template <>
    std::string serialize(const Vector2& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.x);
            appendBinary(data, value.y);
            return data;
        } else {
            std::stringstream ss;
            ss << "Vector2(" << std::fixed << std::setprecision(6) << value.x << ", " << value.y << ")";
            return ss.str();
        }
    }
    
    template <>
    Vector2 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            float x = readBinary<float>(data, offset);
            float y = readBinary<float>(data, offset);
            return Vector2(x, y);
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma = data.find(',', start);
            size_t end = data.find(')', comma);
            
            float x = std::stof(data.substr(start + 1, comma - start - 1));
            float y = std::stof(data.substr(comma + 2, end - comma - 2));
            
            return Vector2(x, y);
        }
    }
    
    // Vector3 序列化
    template <>
    std::string serialize(const Vector3& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.x);
            appendBinary(data, value.y);
            appendBinary(data, value.z);
            return data;
        } else {
            std::stringstream ss;
            ss << "Vector3(" << std::fixed << std::setprecision(6) << value.x << ", " << value.y << ", " << value.z << ")";
            return ss.str();
        }
    }
    
    template <>
    Vector3 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            float x = readBinary<float>(data, offset);
            float y = readBinary<float>(data, offset);
            float z = readBinary<float>(data, offset);
            return Vector3(x, y, z);
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma1 = data.find(',', start);
            size_t comma2 = data.find(',', comma1 + 1);
            size_t end = data.find(')', comma2);
            
            float x = std::stof(data.substr(start + 1, comma1 - start - 1));
            float y = std::stof(data.substr(comma1 + 2, comma2 - comma1 - 2));
            float z = std::stof(data.substr(comma2 + 2, end - comma2 - 2));
            
            return Vector3(x, y, z);
        }
    }
    
    // Vector4 序列化
    template <>
    std::string serialize(const Vector4& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.x);
            appendBinary(data, value.y);
            appendBinary(data, value.z);
            appendBinary(data, value.w);
            return data;
        } else {
            std::stringstream ss;
            ss << "Vector4(" << std::fixed << std::setprecision(6) << value.x << ", " << value.y << ", " << value.z << ", " << value.w << ")";
            return ss.str();
        }
    }
    
    template <>
    Vector4 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            float x = readBinary<float>(data, offset);
            float y = readBinary<float>(data, offset);
            float z = readBinary<float>(data, offset);
            float w = readBinary<float>(data, offset);
            return Vector4(x, y, z, w);
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma1 = data.find(',', start);
            size_t comma2 = data.find(',', comma1 + 1);
            size_t comma3 = data.find(',', comma2 + 1);
            size_t end = data.find(')', comma3);
            
            float x = std::stof(data.substr(start + 1, comma1 - start - 1));
            float y = std::stof(data.substr(comma1 + 2, comma2 - comma1 - 2));
            float z = std::stof(data.substr(comma2 + 2, comma3 - comma2 - 2));
            float w = std::stof(data.substr(comma3 + 2, end - comma3 - 2));
            
            return Vector4(x, y, z, w);
        }
    }
    
    // Matrix2x2 序列化
    template <>
    std::string serialize(const Matrix2x2& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    appendBinary(data, value[i][j]);
                }
            }
            return data;
        } else {
            std::stringstream ss;
            ss << "Matrix2x2(" << std::fixed << std::setprecision(6) 
               << value[0][0] << ", " << value[0][1] << ", " 
               << value[1][0] << ", " << value[1][1] << ")";
            return ss.str();
        }
    }
    
    template <>
    Matrix2x2 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Matrix2x2 matrix;
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    matrix[i][j] = readBinary<float>(data, offset);
                }
            }
            return matrix;
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma1 = data.find(',', start);
            size_t comma2 = data.find(',', comma1 + 1);
            size_t comma3 = data.find(',', comma2 + 1);
            size_t end = data.find(')', comma3);
            
            std::array<float, 4> values = {
                std::stof(data.substr(start + 1, comma1 - start - 1)),
                std::stof(data.substr(comma1 + 2, comma2 - comma1 - 2)),
                std::stof(data.substr(comma2 + 2, comma3 - comma2 - 2)),
                std::stof(data.substr(comma3 + 2, end - comma3 - 2))
            };
            
            return Matrix2x2(values[0], values[1], values[2], values[3]);
        }
    }
    
    // Matrix3x3 序列化
    template <>
    std::string serialize(const Matrix3x3& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    appendBinary(data, value[i][j]);
                }
            }
            return data;
        } else {
            std::stringstream ss;
            ss << "Matrix3x3(" << std::fixed << std::setprecision(6);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    if (i > 0 || j > 0) {
                        ss << ", ";
                    }
                    ss << value[i][j];
                }
            }
            ss << ")";
            return ss.str();
        }
    }
    
    template <>
    Matrix3x3 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Matrix3x3 matrix;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    matrix[i][j] = readBinary<float>(data, offset);
                }
            }
            return matrix;
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t end = data.find(')');
            std::string valuesStr = data.substr(start + 1, end - start - 1);
            
            std::array<float, 9> values;
            size_t pos = 0;
            for (int i = 0; i < 9; ++i) {
                size_t next = valuesStr.find(',', pos);
                if (next == std::string::npos) {
                    next = valuesStr.size();
                }
                values[i] = std::stof(valuesStr.substr(pos, next - pos));
                pos = next + 2;
            }
            
            return Matrix3x3(values[0], values[1], values[2],
                           values[3], values[4], values[5],
                           values[6], values[7], values[8]);
        }
    }
    
    // Matrix4x4 序列化
    template <>
    std::string serialize(const Matrix4x4& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    appendBinary(data, value[i][j]);
                }
            }
            return data;
        } else {
            std::stringstream ss;
            ss << "Matrix4x4(" << std::fixed << std::setprecision(6);
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (i > 0 || j > 0) {
                        ss << ", ";
                    }
                    ss << value[i][j];
                }
            }
            ss << ")";
            return ss.str();
        }
    }
    
    template <>
    Matrix4x4 deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Matrix4x4 matrix;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    matrix[i][j] = readBinary<float>(data, offset);
                }
            }
            return matrix;
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t end = data.find(')');
            std::string valuesStr = data.substr(start + 1, end - start - 1);
            
            std::array<float, 16> values;
            size_t pos = 0;
            for (int i = 0; i < 16; ++i) {
                size_t next = valuesStr.find(',', pos);
                if (next == std::string::npos) {
                    next = valuesStr.size();
                }
                values[i] = std::stof(valuesStr.substr(pos, next - pos));
                pos = next + 2;
            }
            
            return Matrix4x4(values[0], values[1], values[2], values[3],
                           values[4], values[5], values[6], values[7],
                           values[8], values[9], values[10], values[11],
                           values[12], values[13], values[14], values[15]);
        }
    }
    
    // Quaternion 序列化
    template <>
    std::string serialize(const Quaternion& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.x);
            appendBinary(data, value.y);
            appendBinary(data, value.z);
            appendBinary(data, value.w);
            return data;
        } else {
            std::stringstream ss;
            ss << "Quaternion(" << std::fixed << std::setprecision(6) << value.x << ", " << value.y << ", " << value.z << ", " << value.w << ")";
            return ss.str();
        }
    }
    
    template <>
    Quaternion deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            float x = readBinary<float>(data, offset);
            float y = readBinary<float>(data, offset);
            float z = readBinary<float>(data, offset);
            float w = readBinary<float>(data, offset);
            return Quaternion(x, y, z, w);
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma1 = data.find(',', start);
            size_t comma2 = data.find(',', comma1 + 1);
            size_t comma3 = data.find(',', comma2 + 1);
            size_t end = data.find(')', comma3);
            
            float x = std::stof(data.substr(start + 1, comma1 - start - 1));
            float y = std::stof(data.substr(comma1 + 2, comma2 - comma1 - 2));
            float z = std::stof(data.substr(comma2 + 2, comma3 - comma2 - 2));
            float w = std::stof(data.substr(comma3 + 2, end - comma3 - 2));
            
            return Quaternion(x, y, z, w);
        }
    }
    
    // Ray 序列化
    template <>
    std::string serialize(const Ray& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.origin.x);
            appendBinary(data, value.origin.y);
            appendBinary(data, value.origin.z);
            appendBinary(data, value.direction.x);
            appendBinary(data, value.direction.y);
            appendBinary(data, value.direction.z);
            return data;
        } else {
            std::stringstream ss;
            ss << "Ray(Origin(" << std::fixed << std::setprecision(6) << value.origin.x << ", " << value.origin.y << ", " << value.origin.z << "), Direction(" << value.direction.x << ", " << value.direction.y << ", " << value.direction.z << "))";
            return ss.str();
        }
    }
    
    template <>
    Ray deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Vector3 origin(readBinary<float>(data, offset), readBinary<float>(data, offset), readBinary<float>(data, offset));
            Vector3 direction(readBinary<float>(data, offset), readBinary<float>(data, offset), readBinary<float>(data, offset));
            return Ray(origin, direction);
        } else {
            // 简单的字符串解析
            size_t originStart = data.find("Origin(") + 7;
            size_t originComma1 = data.find(',', originStart);
            size_t originComma2 = data.find(',', originComma1 + 1);
            size_t originEnd = data.find(')', originComma2);
            
            Vector3 origin(
                std::stof(data.substr(originStart, originComma1 - originStart)),
                std::stof(data.substr(originComma1 + 2, originComma2 - originComma1 - 2)),
                std::stof(data.substr(originComma2 + 2, originEnd - originComma2 - 2))
            );
            
            size_t dirStart = data.find("Direction(") + 10;
            size_t dirComma1 = data.find(',', dirStart);
            size_t dirComma2 = data.find(',', dirComma1 + 1);
            size_t dirEnd = data.find(')', dirComma2);
            
            Vector3 direction(
                std::stof(data.substr(dirStart, dirComma1 - dirStart)),
                std::stof(data.substr(dirComma1 + 2, dirComma2 - dirComma1 - 2)),
                std::stof(data.substr(dirComma2 + 2, dirEnd - dirComma2 - 2))
            );
            
            return Ray(origin, direction);
        }
    }
    
    // Plane 序列化
    template <>
    std::string serialize(const Plane& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.a);
            appendBinary(data, value.b);
            appendBinary(data, value.c);
            appendBinary(data, value.d);
            return data;
        } else {
            std::stringstream ss;
            ss << "Plane(" << std::fixed << std::setprecision(6) << value.a << ", " << value.b << ", " << value.c << ", " << value.d << ")";
            return ss.str();
        }
    }
    
    template <>
    Plane deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            float a = readBinary<float>(data, offset);
            float b = readBinary<float>(data, offset);
            float c = readBinary<float>(data, offset);
            float d = readBinary<float>(data, offset);
            return Plane(a, b, c, d);
        } else {
            // 简单的字符串解析
            size_t start = data.find('(');
            size_t comma1 = data.find(',', start);
            size_t comma2 = data.find(',', comma1 + 1);
            size_t comma3 = data.find(',', comma2 + 1);
            size_t end = data.find(')', comma3);
            
            float a = std::stof(data.substr(start + 1, comma1 - start - 1));
            float b = std::stof(data.substr(comma1 + 2, comma2 - comma1 - 2));
            float c = std::stof(data.substr(comma2 + 2, comma3 - comma2 - 2));
            float d = std::stof(data.substr(comma3 + 2, end - comma3 - 2));
            
            return Plane(a, b, c, d);
        }
    }
    
    // Sphere 序列化
    template <>
    std::string serialize(const Sphere& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.center.x);
            appendBinary(data, value.center.y);
            appendBinary(data, value.center.z);
            appendBinary(data, value.radius);
            return data;
        } else {
            std::stringstream ss;
            ss << "Sphere(Center(" << std::fixed << std::setprecision(6) << value.center.x << ", " << value.center.y << ", " << value.center.z << "), Radius(" << value.radius << "))";
            return ss.str();
        }
    }
    
    template <>
    Sphere deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Vector3 center(readBinary<float>(data, offset), readBinary<float>(data, offset), readBinary<float>(data, offset));
            float radius = readBinary<float>(data, offset);
            return Sphere(center, radius);
        } else {
            // 简单的字符串解析
            size_t centerStart = data.find("Center(") + 7;
            size_t centerComma1 = data.find(',', centerStart);
            size_t centerComma2 = data.find(',', centerComma1 + 1);
            size_t centerEnd = data.find(')', centerComma2);
            
            Vector3 center(
                std::stof(data.substr(centerStart, centerComma1 - centerStart)),
                std::stof(data.substr(centerComma1 + 2, centerComma2 - centerComma1 - 2)),
                std::stof(data.substr(centerComma2 + 2, centerEnd - centerComma2 - 2))
            );
            
            size_t radiusStart = data.find("Radius(") + 7;
            size_t radiusEnd = data.find(')', radiusStart);
            float radius = std::stof(data.substr(radiusStart, radiusEnd - radiusStart));
            
            return Sphere(center, radius);
        }
    }
    
    // Bounds 序列化
    template <>
    std::string serialize(const Bounds& value, Format format) {
        if (format == Format::Binary) {
            std::string data;
            appendBinary(data, value.min.x);
            appendBinary(data, value.min.y);
            appendBinary(data, value.min.z);
            appendBinary(data, value.max.x);
            appendBinary(data, value.max.y);
            appendBinary(data, value.max.z);
            return data;
        } else {
            std::stringstream ss;
            ss << "Bounds(Min(" << std::fixed << std::setprecision(6) << value.min.x << ", " << value.min.y << ", " << value.min.z << "), Max(" << value.max.x << ", " << value.max.y << ", " << value.max.z << "))";
            return ss.str();
        }
    }
    
    template <>
    Bounds deserialize(const std::string& data, Format format) {
        if (format == Format::Binary) {
            size_t offset = 0;
            Vector3 min(readBinary<float>(data, offset), readBinary<float>(data, offset), readBinary<float>(data, offset));
            Vector3 max(readBinary<float>(data, offset), readBinary<float>(data, offset), readBinary<float>(data, offset));
            return Bounds(min, max);
        } else {
            // 简单的字符串解析
            size_t minStart = data.find("Min(") + 4;
            size_t minComma1 = data.find(',', minStart);
            size_t minComma2 = data.find(',', minComma1 + 1);
            size_t minEnd = data.find(')', minComma2);
            
            Vector3 min(
                std::stof(data.substr(minStart, minComma1 - minStart)),
                std::stof(data.substr(minComma1 + 2, minComma2 - minComma1 - 2)),
                std::stof(data.substr(minComma2 + 2, minEnd - minComma2 - 2))
            );
            
            size_t maxStart = data.find("Max(") + 4;
            size_t maxComma1 = data.find(',', maxStart);
            size_t maxComma2 = data.find(',', maxComma1 + 1);
            size_t maxEnd = data.find(')', maxComma2);
            
            Vector3 max(
                std::stof(data.substr(maxStart, maxComma1 - maxStart)),
                std::stof(data.substr(maxComma1 + 2, maxComma2 - maxComma1 - 2)),
                std::stof(data.substr(maxComma2 + 2, maxEnd - maxComma2 - 2))
            );
            
            return Bounds(min, max);
        }
    }
    
} // namespace Serialization

} // namespace AstraAlgebra
