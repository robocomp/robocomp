//
// Created by juancarlos on 1/7/20.
//

#ifndef USER_TYPES_H
#define USER_TYPES_H

#include <cstdint>
#include <utility>
#include "crdt_types.h"
#include "type_checking/type_checker.h"
#include <dsr/core/utils.h>


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

        void uint64(uint64_t _uint);

        [[nodiscard]] uint64_t uint64() const;

        void fl(float _fl);

        [[nodiscard]] float fl() const;

        void dob(double _dob);

        [[nodiscard]] double dob() const;

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

    private:

        Edge(uint64_t from, uint64_t to, std::string  type, uint32_t agent_id, std::map<std::string, Attribute> attrs)
            : m_to(to),  m_from(from), m_type(std::move(type)),  m_attrs(std::move(attrs)), m_agent_id(agent_id)
        {

        }
    public:

        Edge() = default;

        [[deprecated("Use Edge::create<example_edge_type>(...)")]] Edge(uint64_t mTo, uint64_t mFrom, std::string mType, uint32_t mAgentId) : m_to(mTo), m_from(mFrom), m_type(std::move(mType)), m_attrs{},
                                  m_agent_id(mAgentId)
        {
            if(!edge_types::check_type(m_type)) {
                throw std::runtime_error("Error, \"" + m_type + "\" is not a valid edge type");
            }
        }

        [[deprecated("Use Edge::create<example_edge_type>(...)")]] Edge(uint64_t mTo, uint64_t mFrom, std::string mType,
                   std::map<std::string, Attribute> mAttrs,
                   uint32_t mAgentId) : m_to(mTo), m_from(mFrom), m_type(std::move(mType)), m_attrs{std::move(mAttrs)},
                                        m_agent_id(mAgentId)
        {
            if(!edge_types::check_type(m_type)) {
                throw std::runtime_error("Error, \"" + m_type + "\" is not a valid edge type");
            }
        }

        template <typename edge_type>
        static Edge create(uint64_t from, uint64_t to)
        {
            static_assert(edge_type::edge_type, "Invalid Edge type.");
            return Edge(from, to,  std::string(edge_type::attr_name.data()), 0, {});
        }

        template <typename edge_type>
        static Edge create(uint64_t from, uint64_t to,  const  std::map<std::string, Attribute> &attrs)
        {
            static_assert(edge_type::edge_type, "Invalid Edge type.");
            return Edge(from, to, std::string(edge_type::attr_name.data()), 0, attrs);
        }


        explicit Edge (const CRDTEdge& edge )
        {
            m_agent_id = edge.agent_id();
            m_from = edge.from();
            m_to = edge.to();
            m_type = edge.type();
            for (const auto &[k,v] : edge.attrs()) {
                assert(v.dk.ds.size() > 0);
                m_attrs[k] = v.dk.ds.begin()->second;
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
                assert(v.dk.ds.size() > 0);
                m_attrs[k] = v.dk.ds.begin()->second;
            }
            return *this;
        }

        [[nodiscard]] uint64_t  to() const;

        [[nodiscard]] uint64_t  from() const;

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        [[nodiscard]] const std::map<std::string, Attribute> &attrs() const;

        [[nodiscard]] std::map<std::string, Attribute> &attrs();

        [[nodiscard]] uint32_t agent_id() const;

        void to(uint64_t  mTo);

        void from(uint64_t  mFrom);

        void type(const std::string &mType);

        void attrs(const std::map<std::string, Attribute> &mAttrs);

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
        uint64_t m_to = 0;
        uint64_t m_from = 0;
        std::string m_type;
        std::map<std::string, Attribute> m_attrs;
        uint32_t m_agent_id = 0;
    };

    class Node {
    private:

        Node(std::string  type, uint32_t agent_id,
             std::map<std::string, Attribute> attrs,
             std::map<std::pair<uint64_t, std::string>, Edge > fano, std::string  name = "")
            : m_id(0), m_type(std::move(type)), m_name(std::move(name)), m_attrs{std::move(attrs)}, m_fano{std::move(fano)},  m_agent_id(agent_id)
        {

        }
    public:

        Node() = default;

        [[deprecated("Use Node::create<example_node_type>(...)")]] Node(uint64_t mAgentId, std::string mType) : m_id(0), m_type(std::move(mType)), m_attrs{}, m_fano{}, m_agent_id(mAgentId)
        {
            if (!node_types::check_type(m_type)) {
                throw std::runtime_error("Error, \"" + m_type + "\" is not a valid node type");
            }
        }

        [[deprecated("Use Node::create<example_node_type>(...)")]] Node(std::string mType, uint32_t mAgentId,
                   std::map<std::string, Attribute> mAttrs,
                   std::map<std::pair<uint64_t, std::string>, Edge > mFano)
                : m_id(0), m_type(std::move(mType)), m_attrs{std::move(mAttrs)}, m_fano{std::move(mFano)}, m_agent_id(mAgentId)
        {
            if (!node_types::check_type(m_type)) {
                throw std::runtime_error("Error, \"" + m_type + "\" is not a valid node type");
            }
        }

        template <typename node_type>
        static Node create(/*uint32_t agent_id,*/ const std::string& name = "")
        {
            static_assert(node_type::node_type, "Invalid Node type.");
            return Node( std::string(node_type::attr_name.data()), 0, {}, {}, name);
        }


        template <typename node_type>
        static Node create(/*uint32_t agent_id,*/
                           const  std::map<std::string, Attribute> &attrs,
                           const  std::map<std::pair<uint64_t, std::string>, Edge > &fano,
                           const  std::string& name = "")
        {
            static_assert(node_type::node_type, "Invalid Node type.");
            return Node( std::string(node_type::attr_name.data()), 0, attrs, fano, name);
        }



        explicit Node (const CRDTNode& node)
        {
            m_agent_id = node.agent_id();
            m_id = node.id();
            m_name = node.name();
            m_type = node.type();
            for (const auto &[k,v] : node.attrs()) {
                assert(v.dk.ds.size() > 0);
                m_attrs[k] = v.dk.ds.begin()->second;
            }
            for (const auto &[k,v] : node.fano()) {
                assert(v.dk.ds.size() > 0);
                m_fano[k] = v.dk.ds.begin()->second;
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
                assert(v.dk.ds.size() > 0);
                m_attrs[k] = v.dk.ds.begin()->second;
            }
            for (const auto &[k,v] : node.fano()) {
                assert(v.dk.ds.size() > 0);
                m_fano[k] = v.dk.ds.begin()->second;
            }

            return *this;
        }

        [[nodiscard]] uint64_t id() const;

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        [[nodiscard]] const std::string &name() const;

        [[nodiscard]] std::string &name();

        [[nodiscard]] const std::map<std::string, Attribute> &attrs() const;

        [[nodiscard]] std::map<std::string, Attribute> &attrs();

        [[nodiscard]] const std::map<std::pair<uint64_t, std::string>, Edge > &fano() const;

        [[nodiscard]] std::map<std::pair<uint64_t, std::string>, Edge > &fano();

        [[nodiscard]] uint32_t agent_id() const;

        void id(uint64_t mId);

        void type(const std::string &mType);

        void name(const std::string &mName);

        void attrs(const std::map<std::string, Attribute> &mAttrs);

        void fano(const std::map<std::pair<uint64_t, std::string>, Edge > &mFano);

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
        uint64_t m_id = 0;
        std::string m_type;
        std::string m_name;
        std::map<std::string, Attribute> m_attrs;
        std::map<std::pair<uint64_t, std::string>, Edge > m_fano;
        uint32_t m_agent_id = 0;
    };

}

#endif //USER_TYPES_H
