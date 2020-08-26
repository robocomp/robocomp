//
// Created by juancarlos on 8/6/20.
//

#ifndef DSR_CRDT_TYPES_H
#define DSR_CRDT_TYPES_H

#include "../crdt/delta-crdts.cc"
#include <iostream>
#include "unordered_map"
#include "variant"
#include "map"

namespace DSR {


    static constexpr std::array<std::string_view, 8> TYPENAMES_UNION = { "STRING", "INT", "FLOAT",
                                                                        "FLOAT_VEC", "BOOL", "BYTE_VEC", "UINT",};

    using ValType = std::variant<std::string, int32_t, float, std::vector<float>, bool, std::vector<uint8_t>, uint32_t>;

    enum Types : uint32_t {
        STRING,
        INT,
        FLOAT,
        FLOAT_VEC,
        BOOL,
        BYTE_VEC,
        UINT,
    };


    class CRDTValue
    {
    public:

        CRDTValue() = default;

        ~CRDTValue() = default;


        explicit CRDTValue(IDL::Val &&x)
        {

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
                    val = x.byte_vec();
                    break;
                }
                case 6: {
                    val = x.uint();
                    break;
                }
                default:
                    break;
            }
        }

        [[nodiscard]] const ValType& variant() const
        {
            return val;
        }

        void variant(ValType&& v)
        {
            val = std::move(v);
        }

        void variant(const ValType& v)
        {
            val = v;
        }

        explicit CRDTValue(std::vector<float> &&float_vec)
        {
            val = std::move(float_vec);
        }

        explicit CRDTValue(const std::vector<float> &float_vec)
        {
            val = float_vec;
        }

        CRDTValue &operator=(const CRDTValue &x)
        {

            if (this == &x) return *this;
            val = x.val;
            return *this;
        }

        CRDTValue &operator=(CRDTValue &&x)  noexcept
        {
            if (this == &x) return *this;
            val = std::move(x.val);
            return *this;
        }

        [[nodiscard]] int32_t selected() const;

        std::string &str();

        [[nodiscard]] const std::string &str() const;

        void str(const std::string &_str);

        void str(std::string &&_str);

        void dec(int32_t _dec);

        [[nodiscard]] int32_t dec() const;

        void uint(uint32_t _udec);

        [[nodiscard]] uint32_t uint() const;

        void fl(float _fl);

        [[nodiscard]] float fl() const;

        void float_vec(const std::vector<float> &_float_vec);

        void float_vec(std::vector<float> &&_float_vec);

        [[nodiscard]] const std::vector<float> &float_vec() const;

        std::vector<float> &float_vec();

        void bl(bool _bl);

        [[nodiscard]] bool bl() const;

        void byte_vec(const std::vector<uint8_t> &_float_vec);

        void byte_vec(std::vector<uint8_t> &&_float_vec);

        [[nodiscard]] const std::vector<uint8_t> &byte_vec() const;

        std::vector<uint8_t> &byte_vec();

        [[nodiscard]] IDL::Val toIDLVal();



        bool operator<(const CRDTValue &rhs) const
        {

            if (static_cast<int32_t>(val.index()) != rhs.selected()) return false;

            switch (val.index()) {
                case 0:
                    return str() < rhs.str();
                case 1:
                    return dec() < rhs.dec();
                case 2:
                    return fl() < rhs.fl();
                case 3:
                    return float_vec() < rhs.float_vec();
                case 4:
                    return bl() < rhs.bl();
                case 5:
                    return byte_vec() < rhs.byte_vec();
                case 6: {
                    return uint() < rhs.uint();
                }
                default:
                    return false;
            }
        }

        bool operator>(const CRDTValue &rhs) const
        {
            return rhs < *this;
        }

        bool operator<=(const CRDTValue &rhs) const
        {
            return !(rhs < *this);
        }

        bool operator>=(const CRDTValue &rhs) const
        {
            return !(*this < rhs);
        }

        bool operator==(const CRDTValue &rhs) const
        {

            if (static_cast<int32_t>(val.index()) != rhs.selected()) return false;
            return val == rhs.val;
        }

        bool operator!=(const CRDTValue &rhs) const
        {
            return !(rhs == *this);
        }


        friend std::ostream &operator<<(std::ostream &os, const CRDTValue &type)
        {

            switch (type.selected()) {
                case 0:
                    os << " str: " << type.str();
                    break;
                case 1:
                    os << " dec: " << type.dec();
                    break;
                case 2:
                    os << " float: " << type.fl();
                    break;
                case 3:
                    os << " float_vec: [ ";
                    for (const auto &k: type.float_vec())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 4:
                    os << "bool: " << (type.bl() ? " TRUE" : " FALSE");
                    break;
                case 5:
                    os << " byte_vec: [ ";
                    for (const auto &k: type.byte_vec())
                        os << static_cast<uint8_t >(k) << ", ";
                    os << "] ";
                    break;
                case 6:
                    os << " uint: " << type.uint();
                    break;
                default:
                    os << "INVALID TYPE";
                    break;
            }
            return os;
        }

    private:
        ValType val;
    };


    class CRDTAttribute
    {
    public:

        CRDTAttribute() : m_type(0), m_timestamp(0), m_agent_id(0) {}

        ~CRDTAttribute() = default;

        CRDTAttribute(const CRDTAttribute &x)
        {

            m_type = x.type();
            m_Value = x.val();
            m_timestamp = x.timestamp();
            m_agent_id = x.agent_id();

        }

        CRDTAttribute &operator=(IDL::Attrib &&x)
                {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = CRDTValue(std::move(x.value()));
            m_agent_id = x.agent_id();
            return *this;
        }

        CRDTAttribute &operator=(CRDTAttribute &&x)  noexcept
                {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = std::move(x.val());
            m_agent_id = x.agent_id();
            return *this;
        }

        CRDTAttribute &operator=(const CRDTAttribute &x)
        {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = x.val();
            m_agent_id = x.agent_id();
            return *this;
        }

        void type(uint32_t _type);


        [[nodiscard]] uint32_t type() const;

        void timestamp(uint64_t _time);

        [[nodiscard]] uint64_t timestamp() const;


        void val(IDL::Val &&Value_);

        void val(CRDTValue &&Value_);

        [[nodiscard]] const CRDTValue &val() const;

        CRDTValue &val();

        void agent_id(uint32_t _agent_id);

        [[nodiscard]] uint32_t agent_id() const;

        [[nodiscard]] IDL::Attrib toIDLAttrib();


        bool operator==(const CRDTAttribute &av_) const
        {
            if (this == &av_) {
                return true;
            }
            if (type() != av_.type() || val() != av_.val() || timestamp() != av_.timestamp()) {
                return false;
            }
            return true;
        }

        bool operator<(const CRDTAttribute &av_) const
        {
            if (this == &av_) {
                return false;
            }
            if (CRDTValue() < av_.val()) {
                return true;
            } else if (av_.val() < CRDTValue()) {
                return false;
            }
            return false;
        }

        bool operator!=(const CRDTAttribute &av_) const
        {
            return !operator==(av_);
        }

        bool operator<=(const CRDTAttribute &av_) const
        {
            return operator<(av_) || operator==(av_);
        }

        bool operator>(const CRDTAttribute &av_) const
        {
            return !operator<(av_) && !operator==(av_);
        }

        bool operator>=(const CRDTAttribute &av_) const
        {
            return !operator<(av_);
        }

        friend std::ostream &operator<<(std::ostream &output, const CRDTAttribute &av_)
        {
            output << "Type: " << av_.type() << ", Value[" << av_.val() << "]: " << av_.val() << ", ";
            return output;
        };

    private:
        uint32_t m_type;
        CRDTValue m_Value;
        uint64_t m_timestamp;
        uint32_t m_agent_id;
    };


    class CRDTEdge
    {
    public:


        CRDTEdge() : m_to(0), m_from(0), m_agent_id(0) {}

        ~CRDTEdge() = default;

        CRDTEdge &operator=(const CRDTEdge &x) = default;

        CRDTEdge &operator=(IDL::IDLEdge &&x);

        void to(uint32_t _to);

        [[nodiscard]] uint32_t to() const;


        void type(const std::string &_type);

        void type(std::string &&_type);

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        void from(uint32_t _from);

        [[nodiscard]] uint32_t from() const;

        void attrs(const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &_attrs);

        void attrs(std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &&_attrs);

        [[nodiscard]] const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &attrs() const;

        [[nodiscard]] std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &attrs();

        void agent_id(uint32_t _agent_id);

        [[nodiscard]] uint32_t agent_id() const;

        [[nodiscard]] IDL::IDLEdge toIDLEdge(uint32_t id);


        bool operator==(const CRDTEdge &eA_) const
        {
            if (this == &eA_) {
                return true;
            }
            if (m_type != eA_.m_type || from() != eA_.from() || to() != eA_.to() || attrs() != eA_.attrs()) {
                return false;
            }
            return true;
        }

        bool operator<(const CRDTEdge &eA_) const
        {
            if (this == &eA_) {
                return false;
            }
            if (m_type < eA_.m_type) {
                return true;
            } else if (eA_.m_type < m_type) {
                return false;
            }
            return false;
        }

        bool operator!=(const CRDTEdge &eA_) const
        {
            return !operator==(eA_);
        }

        bool operator<=(const CRDTEdge &eA_) const
        {
            return operator<(eA_) || operator==(eA_);
        }

        bool operator>(const CRDTEdge &eA_) const
        {
            return !operator<(eA_) && !operator==(eA_);
        }

        bool operator>=(const CRDTEdge &eA_) const
        {
            return !operator<(eA_);
        }

        friend std::ostream &operator<<(std::ostream &output, const CRDTEdge &ea_)
        {
            output << "IDL::EdgeAttribs[" << ea_.m_type << ", from:" << ea_.from() << "-> to:" << ea_.to()
                   << " Attribs:[";
            for (const auto &v : ea_.attrs())
                output << v.first << ":" << v.second << " - ";
            output << "]]";
            return output;
        };

    private:
        uint32_t m_to;
        std::string m_type;
        uint32_t m_from;
        std::map<std::string, mvreg<CRDTAttribute, uint32_t>> m_attrs;
        uint32_t m_agent_id{};
    };

    class CRDTNode {

    public:

        CRDTNode() : m_id(0), m_agent_id(0) {}

        ~CRDTNode() = default;

        CRDTNode(const CRDTNode &x)
        {
            m_type = x.m_type;
            m_name = x.m_name;
            m_id = x.m_id;
            m_agent_id = x.m_agent_id;
            m_attrs = x.m_attrs;
            m_fano = x.m_fano;
        }

        explicit CRDTNode(IDL::IDLNode &&x);

        explicit CRDTNode(IDL::IDLNode &x);


        void type(const std::string &_type);

        void type(std::string &&_type);

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        void name(const std::string &_name);

        void name(std::string &&_name);

        [[nodiscard]] const std::string &name() const;

        [[nodiscard]] std::string &name();

        void id(uint32_t _id);

        [[nodiscard]] uint32_t id() const;

        void agent_id(uint32_t _agent_id);

        [[nodiscard]] uint32_t agent_id() const;

        void attrs(const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &_attrs);

        void attrs(std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &&_attrs);

        [[nodiscard]] std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &attrs() &;

        [[nodiscard]] const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &attrs() const &;

        void fano(const std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &_fano);

        void fano(std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &&_fano);

        [[nodiscard]] std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &fano();

        [[nodiscard]] const std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &fano() const;

        [[nodiscard]] IDL::IDLNode toIDLNode(uint32_t id);

        bool operator==(const CRDTNode &n_) const
        {
            if (this == &n_) {
                return true;
            }
            if (id() != n_.id() || type() != n_.type() || attrs() != n_.attrs() || fano() != n_.fano()) {
                return false;
            }
            return true;
        }

        bool operator<(const CRDTNode &n_) const
        {
            if (this == &n_) {
                return false;
            }
            if (id() < n_.id()) {
                return true;
            } else if (n_.id() < id()) {
                return false;
            }
            return false;
        }

        bool operator!=(const CRDTNode &n_) const
        {
            return !operator==(n_);
        }

        bool operator<=(const CRDTNode &n_) const
        {
            return operator<(n_) || operator==(n_);
        }

        bool operator>(const CRDTNode &n_) const
        {
            return !operator<(n_) && !operator==(n_);
        }

        bool operator>=(const CRDTNode &n_) const
        {
            return !operator<(n_);
        }

        friend std::ostream &operator<<(std::ostream &output, CRDTNode &n_)
        {
            output << "IDL::Node:[" << n_.id() << "," << n_.name() << "," << n_.type() << "], Attribs:[";
            for (const auto &v : n_.attrs())
                output << v.first << ":(" << v.second << ");";
            output << "], FanOut:[";
            for (auto &v : n_.fano())
                output << "[ " << v.first.first << " " << v.first.second << "] " << ":(" << v.second << ");";
            output << "]";
            return output;
        }
    private:
        std::string m_type;
        std::string m_name;
        uint32_t m_id{};
        uint32_t m_agent_id{};
        std::map<std::string, mvreg<CRDTAttribute, uint32_t>> m_attrs;
        std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> m_fano;
    };


}

#endif //DSR_CRDT_TYPES_H
