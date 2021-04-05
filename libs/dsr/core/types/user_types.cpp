//
// Created by juancarlos on 1/7/20.
//

#include <dsr/core/types/user_types.h>

namespace DSR {

    /////////////////////////////////////////////////////
    /// Attribute
    ////////////////////////////////////////////////////

    const ValType &Attribute::value() const
    {
        return m_value;
    }

    ValType& Attribute::value()
    {
        return m_value;
    }

    uint64_t Attribute::timestamp() const
    {
        return m_timestamp;
    }

    uint32_t Attribute::agent_id() const
    {
        return m_agent_id;
    }

    void Attribute::timestamp(uint64_t t)
    {
        m_timestamp = t;
    }

    void Attribute::value(const ValType &mValue)
    {
        m_value = mValue;
    }


    void Attribute::agent_id(uint32_t mAgentId)
    {
        m_agent_id = mAgentId;
    }


    [[nodiscard]] int32_t Attribute::selected() const
    {
        return m_value.index();
    }

    std::string &Attribute::str()
    {
        if (auto pval = std::get_if<std::string>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("STRING is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());

    }

    [[nodiscard]] const std::string &Attribute::str() const
    {
        if (auto pval = std::get_if<std::string>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("STRING is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());

    }

    void Attribute::str(const std::string &_str)
    {
        m_value = _str;
    }

    void Attribute::str(std::string &&_str)
    {
        m_value = std::move(_str);
    }

    void Attribute::dec(int32_t _dec)
    {
        m_value = _dec;
    }

    [[nodiscard]] int32_t Attribute::dec() const
    {
        if (auto pval = std::get_if<int32_t>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("INT is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());

    }

    void Attribute::uint(uint32_t _uint)
    {
        m_value = _uint;
    }

    [[nodiscard]] uint32_t Attribute::uint() const
    {
        if (auto pval = std::get_if<uint32_t>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("UINT is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());

    }

    void Attribute::uint64(uint64_t _uint)
    {
        m_value = _uint;
    }

    [[nodiscard]] uint64_t Attribute::uint64() const
    {
        if (auto pval = std::get_if<uint64_t>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("UINT64 is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());

    }

    void Attribute::fl(float _fl)
    {
        m_value = _fl;
    }

    [[nodiscard]] float Attribute::fl() const
    {
        if (auto pval = std::get_if<float>(&m_value)) {
            return *pval;
        }

        throw std::runtime_error(
                ("FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    void Attribute::float_vec(const std::vector<float> &_float_vec)
    {
        m_value = _float_vec;
    }

    void Attribute::float_vec(std::vector<float> &&_float_vec)
    {
        m_value = std::move(_float_vec);
    }

    const std::vector<float> &Attribute::float_vec() const
    {
        if (auto pval = std::get_if<std::vector<float>>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    std::vector<float> &Attribute::float_vec()
    {

        if (auto pval = std::get_if<std::vector<float>>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    void Attribute::bl(bool _bl)
    {
        m_value = _bl;
    }

    [[nodiscard]] bool Attribute::bl() const
    {

        if (auto pval = std::get_if<bool>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("BOOL is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    void Attribute::byte_vec(const std::vector<uint8_t> &_float_vec)
    {
        m_value = _float_vec;
    }

    void Attribute::byte_vec(std::vector<uint8_t> &&_float_vec)
    {
        m_value = std::move(_float_vec);
    }

    [[nodiscard]] const std::vector<uint8_t> &Attribute::byte_vec() const
    {
        if (auto pval = std::get_if<std::vector<uint8_t>>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_BYTE is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    std::vector<uint8_t> &Attribute::byte_vec()
    {

        if (auto pval = std::get_if<std::vector<uint8_t >>(&m_value)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_BYTE is not selected, selected is " + std::string(TYPENAMES_UNION[m_value.index()])).data());
    }

    /////////////////////////////////////////////////////
    /// Edge
    ////////////////////////////////////////////////////

    uint64_t Edge::to() const
    {
        return m_to;
    }

    uint64_t Edge::from() const
    {
        return m_from;
    }

    const std::string &Edge::type() const
    {
        return m_type;
    }

    std::string &Edge::type()
    {
        return m_type;
    }

    const  std::map<std::string, Attribute> &Edge::attrs() const
    {
        return m_attrs;
    }

    std::map<std::string, Attribute> &Edge::attrs()
     {
        return m_attrs;
    }

    uint32_t Edge::agent_id() const
    {
        return m_agent_id;
    }

    void Edge::to(uint64_t mTo)
    {
        m_to = mTo;
    }

    void Edge::from(uint64_t mFrom)
    {
        m_from = mFrom;
    }

    void Edge::type(const std::string &mType)
    {
        if(!edge_types::check_type(mType)) {
            throw std::runtime_error("Error, " + mType + " is not a valid edge type");
        }
        m_type = mType;
    }

    void Edge::attrs(const  std::map<std::string, Attribute> &mAttrs)
    {
        m_attrs = mAttrs;
    }

    void Edge::agent_id(uint32_t mAgentId)
    {
        m_agent_id = mAgentId;
    }

    /////////////////////////////////////////////////////
    /// Node
    ////////////////////////////////////////////////////

    uint64_t Node::id() const
    {
        return m_id;
    }

    const std::string &Node::type() const
    {
        return m_type;
    }

    std::string &Node::type()
    {
        return m_type;
    }

    const std::string &Node::name() const
    {
        return m_name;
    }

    std::string &Node::name()
    {
        return m_name;
    }

    const  std::map<std::string, Attribute> &Node::attrs() const
    {
        return m_attrs;
    }

    std::map<std::string, Attribute>& Node::attrs()
     {
        return m_attrs;
    }

    const  std::map<std::pair<uint64_t, std::string>, Edge > &Node::fano() const
    {
        return m_fano;
    }

    std::map<std::pair<uint64_t, std::string>, Edge > &Node::fano()
    {
        return m_fano;
    }

    uint32_t Node::agent_id() const
    {
        return m_agent_id;
    }

    void Node::id(uint64_t mId)
    {
        m_id = mId;
    }

    void Node::type (const std::string &mType)
    {
        if(!node_types::check_type(mType)) {
            throw std::runtime_error("Error, " + mType + " is not a valid node type");
        }
        m_type = mType;
    }

    void Node::name(const std::string &mName)
    {
        m_name = mName;
    }

    void Node::attrs(const  std::map<std::string, Attribute> &mAttrs)
    {
        m_attrs = mAttrs;
    }

    void Node::fano(const  std::map<std::pair<uint64_t, std::string>, Edge > &mFano)
    {
        m_fano = mFano;
    }

    void Node::agent_id(uint32_t mAgentId)
    {
        m_agent_id = mAgentId;
    }
}