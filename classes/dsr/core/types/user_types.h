//
// Created by juancarlos on 1/7/20.
//

#ifndef USER_TYPES_H
#define USER_TYPES_H

#include <cstdint>
#include <utility>
#include "crdt_types.h"
#include "type_checking/type_checker.h"

#define TYPE_ASSERT_ERROR(x, y) "Error, " #x "is not a valid" #y "type"

namespace DSR {


    class Attribute
    {
    public:

        Attribute() = default;

        Attribute(const ValType &value_, uint64_t timestamp_, uint32_t agent_id_)
                : m_value(ValType(value_)), m_timestamp(timestamp_), m_agent_id(agent_id_) {}

        explicit Attribute (const CRDTAttribute& attr )
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = ValType(attr.val().variant());
        }
        ~Attribute() = default;

        Attribute& operator= (const CRDTAttribute& attr )
        {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = ValType(attr.val().variant());

            return *this;
        }

        [[nodiscard]] const ValType &value() const;

        ValType& value();

        [[nodiscard]] uint64_t timestamp() const;

        void timestamp(uint64_t t);

        [[nodiscard]] uint32_t agent_id() const;

        void value(const ValType &mValue);

        void agent_id(uint32_t mAgentId);

        [[nodiscard]] int32_t selected() const;

        std::string &str();

        [[nodiscard]] const std::string &str() const;

        void str(const std::string &_str);

        void str(std::string &&_str);

        void dec(int32_t _dec);

        [[nodiscard]] int32_t dec() const;

        void uint(uint32_t _uint);

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

        friend std::ostream &operator<<(std::ostream &os, const Attribute &type)
        {

            switch (type.m_value.index()) {
                case 0:
                    os << " str: " << get<string>(type.m_value);
                    break;
                case 1:
                    os << " dec: " << get<int32_t>(type.m_value);
                    break;
                case 2:
                    os << " float: " << get<float>(type.m_value);
                    break;
                case 3:
                    os << " float_vec: [ ";
                    for (const auto &k: get<vector<float>>(type.m_value))
                        os << k << ", ";
                    os << "] ";
                    break;
                case 4:
                    os << "bool: " << (get<bool>(type.m_value) ? " TRUE" : " FALSE");
                    break;
                case 5:
                    os << " byte_vec: [ ";
                    for (const auto &k: get<vector<uint8_t>>(type.m_value))
                        os << k << ", ";
                    os << "] ";
                    break;
                case 6:
                    os << " uint: " << get<uint32_t>(type.m_value);
                    break;
                default:
                    os << "INVALID TYPE";
                    break;
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

    private:
        ValType m_value;
        uint64_t m_timestamp = 0;
        uint32_t m_agent_id = 0;
    };

    class Edge
    {
    public:

        Edge() = default;

        /* template constructor doesnt work
        template <std::string_view& type>
        Edge(uint32_t mTo, uint32_t mFrom,  uint32_t mAgentId) :  m_to(mTo), m_from(mFrom), m_type(type), m_attrs{},
                                  m_agent_id(mAgentId)
        {

            static_assert(EDGE_TYPES::find(type), TYPE_ASSERT_ERROR(type, edge));
        }*/

        Edge(uint32_t mTo, uint32_t mFrom, string mType, uint32_t mAgentId) : m_to(mTo), m_from(mFrom), m_type(std::move(mType)), m_attrs{},
                                  m_agent_id(mAgentId)
        {
            if(!EDGE_TYPES::find(m_type)) {
                throw std::runtime_error("Error, " + m_type + " is not a valid edge type");
            }
        }

        Edge(uint32_t mTo, uint32_t mFrom, string mType,
                   const  map<std::string, Attribute> &mAttrs,
                   uint32_t mAgentId) : m_to(mTo), m_from(mFrom), m_type(std::move(mType)), m_attrs{mAttrs},
                                        m_agent_id(mAgentId)
        {
            if(!EDGE_TYPES::find(m_type)) {
                throw std::runtime_error("Error, " + m_type + " is not a valid edge type");
            }
        }


        explicit Edge (const CRDTEdge& edge )
        {
            m_agent_id = edge.agent_id();
            m_from = edge.from();
            m_to = edge.to();
            m_type = edge.type();
            for (const auto &[k,v] : edge.attrs()) {
                m_attrs[k] = *v.read().begin();
            }

        }
        ~Edge() = default;


        Edge& operator= (const CRDTEdge& attr )
        {
            m_agent_id = attr.agent_id();
            m_from = attr.from();
            m_to = attr.to();
            m_type = attr.type();
            for (const auto &[k,v] : attr.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            return *this;
        }

        [[nodiscard]] uint32_t to() const;

        [[nodiscard]] uint32_t from() const;

        [[nodiscard]] const string &type() const;

        [[nodiscard]] string &type();

        [[nodiscard]] const map<std::string, Attribute> &attrs() const;

        [[nodiscard]] map<std::string, Attribute> &attrs();

        [[nodiscard]] uint32_t agent_id() const;

        void to(uint32_t mTo);

        void from(uint32_t mFrom);

        void type(const string &mType);

        void attrs(const map<std::string, Attribute> &mAttrs);

        void agent_id(uint32_t mAgentId);

        bool operator==(const Edge &rhs) const
        {
            return m_to == rhs.m_to &&
                   m_from == rhs.m_from &&
                   m_type == rhs.m_type &&
                   m_attrs == rhs.m_attrs;
        }

        bool operator!=(const Edge &rhs) const
        {
            return !(rhs == *this);
        }

        bool operator<(const Edge &rhs) const
        {
            if (m_to < rhs.m_to)
                return true;
            if (rhs.m_to < m_to)
                return false;
            if (m_from < rhs.m_from)
                return true;
            if (rhs.m_from < m_from)
                return false;
            if (m_type < rhs.m_type)
                return true;
            if (rhs.m_type < m_type)
                return false;
            return true;
        }

        bool operator>(const Edge &rhs) const
        {
            return rhs < *this;
        }

        bool operator<=(const Edge &rhs) const
        {
            return !(rhs < *this);
        }

        bool operator>=(const Edge &rhs) const
        {
            return !(*this < rhs);
        }

    private:
        uint32_t m_to = 0;
        uint32_t m_from = 0;
        std::string m_type;
        std::map<std::string, Attribute> m_attrs;
        uint32_t m_agent_id = 0;
    };

    class Node {
    public:

        Node() = default;

        Node(uint32_t mAgentId, string mType) : m_id(0), m_type(std::move(mType)), m_attrs{}, m_fano{}, m_agent_id(mAgentId)
        {
            if (!NODE_TYPES::find(m_type)) {
                throw std::runtime_error("Error, " + m_type + " is not a valid node type");
            }
        }

        Node(string mType, uint32_t mAgentId,
                   const  map<std::string, Attribute> &mAttrs,
                   const  map<std::pair<uint32_t, std::string>, Edge > &mFano)
                : m_id(0), m_type(std::move(mType)), m_attrs{mAttrs}, m_fano{mFano}, m_agent_id(mAgentId)
        {
            if (!NODE_TYPES::find(m_type)) {
                throw std::runtime_error("Error, " + m_type + " is not a valid node type");
            }
        }

        /*
        template <std::string_view& type>
        static Node make_node(uint32_t mAgentId)
        {
            static_assert(NODE_TYPES::find(type), TYPE_ASSERT_ERROR(type, node));
            Node n;
            n.m_type = type;
            n.m_agent_id = mAgentId;
            return n;
        }
         */

        explicit Node (const CRDTNode& node)
        {
            m_agent_id = node.agent_id();
            m_id = node.id();
            m_name = node.name();
            m_type = node.type();
            for (const auto &[k,v] : node.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            for (const auto &[k,v] : node.fano()) {
                m_fano[k] = *v.read().begin();
            }
        }


        ~Node() = default;

        Node& operator= (const CRDTNode& node )
        {
            m_agent_id = node.agent_id();
            m_id = node.id();
            m_name = node.name();
            m_type = node.type();
            for (const auto &[k,v] : node.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            for (const auto &[k,v] : node.fano()) {
                m_fano[k] = *v.read().begin();
            }

            return *this;
        }

        [[nodiscard]] uint32_t id() const;

        [[nodiscard]] const string &type() const;

        [[nodiscard]] string &type();

        [[nodiscard]] const string &name() const;

        [[nodiscard]] string &name();

        [[nodiscard]] const map<std::string, Attribute> &attrs() const;

        [[nodiscard]] map<std::string, Attribute> &attrs();

        [[nodiscard]] const map<std::pair<uint32_t, std::string>, Edge > &fano() const;

        [[nodiscard]] map<std::pair<uint32_t, std::string>, Edge > &fano();

        [[nodiscard]] uint32_t agent_id() const;

        void id(uint32_t mId);

        void type(const string &mType);

        void name(const string &mName);

        void attrs(const map<std::string, Attribute> &mAttrs);

        void fano(const map<std::pair<uint32_t, std::string>, Edge > &mFano);

        void agent_id(uint32_t mAgentId);

        bool operator==(const Node &rhs) const
        {
            return m_id == rhs.m_id &&
                   m_type == rhs.m_type &&
                   m_name == rhs.m_name &&
                   m_attrs == rhs.m_attrs &&
                   m_fano == rhs.m_fano;
        }

        bool operator!=(const Node &rhs) const
        {
            return !(rhs == *this);
        }

        bool operator<(const Node &rhs) const
        {
            if (m_id < rhs.m_id)
                return true;
            if (rhs.m_id < m_id)
                return false;
            if (m_type < rhs.m_type)
                return true;
            if (rhs.m_type < m_type)
                return false;
            if (m_name < rhs.m_name)
                return true;
            if (rhs.m_name < m_name)
                return false;
            return true;
        }

        bool operator>(const Node &rhs) const
        {
            return rhs < *this;
        }

        bool operator<=(const Node &rhs) const
        {
            return !(rhs < *this);
        }

        bool operator>=(const Node &rhs) const
        {
            return !(*this < rhs);
        }

    private:
        uint32_t m_id = 0;
        std::string m_type;
        std::string m_name;
        std::map<std::string, Attribute> m_attrs;
        std::map<std::pair<uint32_t, std::string>, Edge > m_fano;
        uint32_t m_agent_id = 0;
    };

}

#endif //USER_TYPES_H
