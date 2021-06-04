//
// Created by juancarlos on 1/7/20.
//

#include <dsr/core/types/user_types.h>

namespace DSR {

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

    const std::map<std::string, Attribute> &Edge::attrs() const
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
            throw std::runtime_error("Error, \"" + mType + "\" is not a valid edge type");
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
            throw std::runtime_error("Error, \"" + mType + "\" is not a valid node type");
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