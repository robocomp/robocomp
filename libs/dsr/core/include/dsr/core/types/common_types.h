//
// Created by juancarlos on 1/6/21.
//

#ifndef DSR_COMMON_TYPES_H
#define DSR_COMMON_TYPES_H

#include "../topics/IDLGraph.h"
#include "../utils.h"

#include <variant>
#include <unordered_map>
#include <iostream>
#include <cassert>

namespace DSR {
    static constexpr std::array<std::string_view, 14> TYPENAMES_UNION =
            { "STRING", "INT", "FLOAT",
          "FLOAT_VEC", "BOOL", "BYTE_VEC",
          "UINT","UINT64", "DOUBLE",
          "UINT64_VEC", "FLOAT_VEC2", "FLOAT_VEC3",
        "FLOAT_VEC4", "FLOAT_VEC6"};

    using ValType = std::variant<std::string, int32_t, float,
            std::vector<float>, bool, std::vector<uint8_t>,
            uint32_t, uint64_t, double, std::vector<uint64_t>,
            std::array<float, 2>, std::array<float, 3>,
            std::array<float, 4>, std::array<float, 6>>;


    enum Types : uint32_t {
        STRING = 0,
        INT = 1,
        FLOAT = 2,
        FLOAT_VEC = 3,
        BOOL = 4,
        BYTE_VEC = 5,
        UINT = 6,
        UINT64 = 7,
        DOUBLE = 8,
        U64_VEC = 9,
        VEC2 = 10,
        VEC3 = 11,
        VEC4 = 12,
        VEC6 = 13
    };


    class Attribute
    {
    public:

        Attribute() = default;
        ~Attribute() = default;

        Attribute(const ValType &value_, uint64_t timestamp_, uint32_t agent_id_)
                : m_value(ValType(value_)), m_timestamp(timestamp_), m_agent_id(agent_id_)
        {}

        Attribute (const Attribute& attr)
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = attr.m_value;
        }

        Attribute (Attribute&& attr) noexcept
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = std::move(attr.m_value);
        }

        explicit Attribute (IDL::Attrib &&x) noexcept
        {
            m_timestamp = x.timestamp();
            m_value = std::move(get_valtype(std::move(x.value())));
            m_agent_id = x.agent_id();
        }

        Attribute& operator= (const Attribute& attr )
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = attr.m_value;
            return *this;
        }

        Attribute& operator= (Attribute&& attr) noexcept
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = std::move(attr.m_value);
            return *this;
        }

        Attribute &operator=(IDL::Attrib &&x)
        {
            m_timestamp = x.timestamp();
            m_value = std::move(get_valtype(std::move(x.value())));
            m_agent_id = x.agent_id();
            return *this;
        }

        [[nodiscard]] IDL::Val toIDLVal();

        [[nodiscard]] IDL::Attrib toIDLAttrib();

        ///////////////////////
        // Members
        //////////////////////

        [[nodiscard]] uint64_t timestamp() const;

        void timestamp(uint64_t t);

        [[nodiscard]] uint32_t agent_id() const;

        void agent_id(uint32_t mAgentId);

        [[nodiscard]] std::size_t selected() const;

        ///////////////////////
        // Variant
        //////////////////////
        void value(const ValType &mValue);

        void value(ValType &&mValue);

        [[nodiscard]] const ValType &value() const;

        [[nodiscard]] ValType& value();
        ///////////////////////
        // String
        //////////////////////

        [[nodiscard]] std::string &str();

        [[nodiscard]] const std::string &str() const;

        void str(const std::string &_str);

        void str(std::string &&_str);

        ///////////////////////
        // int32
        //////////////////////
        void dec(int32_t _dec);

        [[nodiscard]] int32_t dec() const;

        ///////////////////////
        // uint32
        //////////////////////
        void uint(uint32_t _uint);

        [[nodiscard]] uint32_t uint() const;

        ///////////////////////
        // uint64
        //////////////////////
        void uint64(uint64_t _uint);

        [[nodiscard]] uint64_t uint64() const;

        ///////////////////////
        // float
        //////////////////////
        void fl(float _fl);

        [[nodiscard]] float fl() const;

        ///////////////////////
        // double
        //////////////////////
        void dob(double _dob);

        [[nodiscard]] double dob() const;

        ///////////////////////
        // vector<float>
        //////////////////////
        void float_vec(const std::vector<float> &_float_vec);

        void float_vec(std::vector<float> &&_float_vec);

        [[nodiscard]] const std::vector<float> &float_vec() const;

        std::vector<float> &float_vec();

        ///////////////////////
        // bool
        //////////////////////
        void bl(bool _bl);

        [[nodiscard]] bool bl() const;

        ///////////////////////
        // vector<uint8>
        //////////////////////
        void byte_vec(const std::vector<uint8_t> &_float_vec);

        void byte_vec(std::vector<uint8_t> &&_float_vec);

        [[nodiscard]] const std::vector<uint8_t> &byte_vec() const;

        [[nodiscard]] std::vector<uint8_t> &byte_vec();

        ///////////////////////
        // vector<uint64>
        //////////////////////
        void u64_vec(const std::vector<uint64_t> &_uint64_vec);

        void u64_vec(std::vector<uint64_t> &&_uint64_vec);

        [[nodiscard]] const std::vector<uint64_t> &u64_vec() const;

        [[nodiscard]] std::vector<uint64_t> &u64_vec();

        ///////////////////////
        // array<flaot, 2>
        //////////////////////
        void vec2(const std::array<float, 2> &_vec_float2);

        [[nodiscard]] const std::array<float, 2> &vec2() const;

        [[nodiscard]] std::array<float, 2> &vec2();

        ///////////////////////
        // array<flaot, 3>
        //////////////////////
        void vec3(const std::array<float, 3> &_vec_float3);

        [[nodiscard]] const std::array<float, 3> &vec3() const;

        [[nodiscard]] std::array<float, 3> &vec3();

        ///////////////////////
        // array<flaot, 4>
        //////////////////////
        void vec4(const std::array<float, 4> &_vec_float4);

        [[nodiscard]] const std::array<float, 4> &vec4() const;

        [[nodiscard]] std::array<float, 4> &vec4();

        ///////////////////////
        // array<flaot, 6>
        //////////////////////
        void vec6(const std::array<float, 6> &_vec_float6);

        [[nodiscard]] const std::array<float, 6> &vec6() const;

        [[nodiscard]] std::array<float, 6> &vec6();

        ///////////////////////
        // Operators
        //////////////////////
        friend std::ostream &operator<<(std::ostream &os, const Attribute &type)
        {

            switch (type.m_value.index()) {
                case 0:
                    os << " str: " << std::get<std::string>(type.m_value);
                    break;
                case 1:
                    os << " dec: " << std::get<int32_t>(type.m_value);
                    break;
                case 2:
                    os << " float: " << std::get<float>(type.m_value);
                    break;
                case 3:
                    os << " float_vec: [ ";
                    for (const auto &k: std::get<std::vector<float>>(type.m_value))
                        os << k << ", ";
                    os << "] ";
                    break;
                case 4:
                    os << "bool: " << (std::get<bool>(type.m_value) ? " TRUE" : " FALSE");
                    break;
                case 5:
                    os << " byte_vec: [ ";
                    for (const uint8_t k: std::get<std::vector<uint8_t>>(type.m_value))
                        os << std::to_string(k) << ", ";
                    os << "] ";
                    break;
                case 6:
                    os << " uint: " << std::get<uint32_t>(type.m_value);
                    break;
                case 7:
                    os << " uint64: " << std::get<uint64_t>(type.m_value);
                    break;
                case 8:
                    os << " double: " << std::get<double>(type.m_value);
                    break;
                case 9:
                    os << " u64_vec: [ ";
                    for (const auto &k: type.u64_vec())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 10:
                    os << " vec2: [ ";
                    for (const auto &k: type.vec2())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 11:
                    os << " vec3: [ ";
                    for (const auto &k: type.vec3())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 12:
                    os << " vec4: [ ";
                    for (const auto &k: type.vec4())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 13:
                    os << " vec6: [ ";
                    for (const auto &k: type.vec6())
                        os << k << ", ";
                    os << "] ";
                    break;
                default:
                    os << "INVALID TYPE";
                    assert(false);
            }
            return os;
        }

        bool operator==(const Attribute &rhs) const
        {
            return m_value == rhs.m_value;
        }

        bool operator!=(const Attribute &rhs) const
        {
            return !(rhs == *this);
        }

        bool operator<(const Attribute &rhs) const
        {
            return m_value < rhs.m_value;
        }

        bool operator>(const Attribute &rhs) const
        {
            return rhs < *this;
        }

        bool operator<=(const Attribute &rhs) const
        {
            return !(rhs < *this);
        }

        bool operator>=(const Attribute &rhs) const
        {
            return !(*this < rhs);
        }

        [[nodiscard]] static ValType get_valtype(IDL::Val &&x)
        {
            ValType val;
            switch (x._d()) {
                case 0:
                    val = std::move(x.str());
                    break;
                case 1:
                    val = x.dec();
                    break;
                case 2:
                    val = x.fl();
                    break;
                case 3:
                    val = std::move(x.float_vec());
                    break;
                case 4:
                    val = x.bl();
                    break;
                case 5: {
                    val = std::move(x.byte_vec());
                    break;
                }
                case 6: {
                    val = x.uint();
                    break;
                }
                case 7: {
                    val = x.u64();
                    break;
                }
                case 8: {
                    val = x.dob();
                    break;
                }
                case 9: {
                    val = std::move(x.uint64_vec());
                    break;
                }
                case 10: {
                    val = x.vec_float2();
                    break;
                }
                case 11: {
                    val = x.vec_float3();
                    break;
                }
                case 12: {
                    val = x.vec_float4();
                    break;
                }
                case 13: {
                    val = x.vec_float6();
                    break;
                }
                default:
                    assert(false);
            }
            return val;
        }
    private:

        ValType m_value;
        uint64_t m_timestamp = 0;
        uint32_t m_agent_id = 0;
    };

}





#endif //DSR_COMMON_TYPES_H
