//
// Created by juancarlos on 8/6/20.
//


#include <dsr/core/types/crdt_types.h>
#include <dsr/core/types/translator.h>

namespace DSR {

    CRDTEdge::CRDTEdge (IDL::IDLEdge &&x) noexcept
    {

        m_to = x.to();
        m_type = std::move(x.type());
        m_from = x.from();
        if (!x.attrs().empty()) {
            for (auto&[k, v] : x.attrs()) {
                m_attrs.emplace(k , IDLEdgeAttr_to_CRDT(std::move(v)));
            }
        }
        m_agent_id = x.agent_id();

    }

    CRDTEdge &CRDTEdge::operator=(IDL::IDLEdge &&x)
    {

        m_to = x.to();
        m_type = std::move(x.type());
        m_from = x.from();
        if (!x.attrs().empty()) {
            for (auto&[k, v] : x.attrs()) {
                m_attrs.emplace(k , IDLEdgeAttr_to_CRDT(std::move(v)));
            }
        }
        m_agent_id = x.agent_id();

        return *this;
    }

    void CRDTEdge::to(uint64_t  _to)
    {
        m_to = _to;
    }

    uint64_t  CRDTEdge::to() const
    {
        return m_to;
    }

    void CRDTEdge::type(const std::string &_type) 
    {
        m_type = _type;
    }

    void CRDTEdge::type(std::string &&_type) 
    {
        m_type = std::move(_type);
    }

    const std::string &CRDTEdge::type() const 
    {
        return m_type;
    }

    std::string &CRDTEdge::type()
    {
        return m_type;
    }

    void CRDTEdge::from(uint64_t  _from)
    {
        m_from = _from;
    }

    uint64_t  CRDTEdge::from() const
    {
        return m_from;
    }

    void CRDTEdge::attrs(const std::map<std::string, mvreg<CRDTAttribute>> &_attrs)
    {
        m_attrs = _attrs;
    }

    void CRDTEdge::attrs(std::map<std::string, mvreg<CRDTAttribute>> &&_attrs)
    {
        m_attrs = std::move(_attrs);
    }

    const std::map<std::string, mvreg<CRDTAttribute>> &CRDTEdge::attrs() const
    {
        return m_attrs;
    }

    std::map<std::string, mvreg<CRDTAttribute>> &CRDTEdge::attrs()
    {
        return m_attrs;
    }

    void CRDTEdge::agent_id(uint32_t _agent_id)
    {
        m_agent_id = _agent_id;
    }

    uint32_t CRDTEdge::agent_id() const
    {
        return m_agent_id;
    }

    IDL::IDLEdge CRDTEdge::toIDLEdge(uint64_t id)
    {
        IDL::IDLEdge edge;
        edge.from(m_from);
        edge.to(m_to);
        edge.type(m_type);
        edge.agent_id(m_agent_id);
        for (auto &[k, v] : m_attrs) {

            IDL::MvregEdgeAttr edgeAttr;
            for (auto &kv_dots : v.dk.ds) {
                IDL::PairInt pi;
                pi.first(kv_dots.first.first);
                pi.second(kv_dots.first.second);

                edgeAttr.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLAttrib()));
                edgeAttr.dk().cbase().cc().emplace(kv_dots.first);

            }

            edgeAttr.from(m_from);
            edgeAttr.to(m_to);
            edgeAttr.type(m_type);
            edgeAttr.agent_id(v.read_reg().agent_id());
            edgeAttr.id(id);

            edge.attrs().emplace(k, std::move(edgeAttr));
        }
        return edge;
    }


    CRDTNode::CRDTNode(IDL::IDLNode &&x)
    {
        m_type = std::move(x.type());
        m_name = std::move(x.name());
        m_id = x.id();
        m_agent_id = x.agent_id();
        for (auto&[k, v] : x.attrs()) {
            m_attrs.emplace(k, IDLNodeAttr_to_CRDT(std::move(v)));
        }
        for (auto&[k, v] : x.fano()) {
            m_fano.emplace(std::pair<uint64_t, std::string>{k.to(), k.type()},
                           IDLEdge_to_CRDT(std::move(v)));
        }
    }

    void CRDTNode::type(const std::string &_type)
    {
        m_type = _type;
    }

    void CRDTNode::type(std::string &&_type)
    {
        m_type = std::move(_type);
    }

    const std::string &CRDTNode::type() const
    {
        return m_type;
    }

    std::string &CRDTNode::type()
    {
        return m_type;
    }

    void CRDTNode::name(const std::string &_name)
    {
        m_name = _name;
    }

    void CRDTNode::name(std::string &&_name)
    {
        m_name = std::move(_name);
    }

    const std::string &CRDTNode::name() const
    {
        return m_name;
    }

    std::string &CRDTNode::name()
    {
        return m_name;
    }

    void CRDTNode::id(uint64_t _id)
    {
        m_id = _id;
    }

    uint64_t CRDTNode::id() const
    {
        return m_id;
    }

    void CRDTNode::agent_id(uint32_t _agent_id)
    {
        m_agent_id = _agent_id;
    }

    uint32_t CRDTNode::agent_id() const
    {
        return m_agent_id;
    }

    void CRDTNode::attrs(const std::map<std::string, mvreg<CRDTAttribute>> &_attrs)
    {
        m_attrs = _attrs;
    }

    void CRDTNode::attrs(std::map<std::string, mvreg<CRDTAttribute>> &&_attrs)
    {
        m_attrs = std::move(_attrs);
    }

    std::map<std::string, mvreg<CRDTAttribute>> &CRDTNode::attrs() &
    {
        return m_attrs;
    }

    const std::map<std::string, mvreg<CRDTAttribute>> &CRDTNode::attrs() const &
    {
        return m_attrs;
    }

    void CRDTNode::fano(const std::map<std::pair<uint64_t , std::string>, mvreg<CRDTEdge>> &_fano)
    {
        m_fano = _fano;
    }

    void CRDTNode::fano(std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &&_fano)
    {
        m_fano = std::move(_fano);
    }

    std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &CRDTNode::fano()
    {
        return m_fano;
    }

    const std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &CRDTNode::fano() const
    {
        return m_fano;
    }


    IDL::IDLNode CRDTNode::toIDLNode(uint64_t id)
    {
        IDL::IDLNode node;
        node.id(m_id);
        node.name(m_name);
        node.type(m_type);
        node.agent_id(m_agent_id);
        for (auto &[k, v] : m_attrs) {
            IDL::MvregNodeAttr nodeAttr;
            for (auto &kv_dots : v.dk.ds) {
                IDL::PairInt pi;
                pi.first(kv_dots.first.first);
                pi.second(kv_dots.first.second);

                nodeAttr.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLAttrib()));
                nodeAttr.dk().cbase().cc().emplace(kv_dots.first);
            }

            nodeAttr.id(id);
            nodeAttr.attr_name(k);
            nodeAttr.agent_id(v.read_reg().agent_id());
            node.attrs().emplace(k, std::move(nodeAttr));
        }

        for (auto &[k, v] : m_fano) {
            IDL::MvregEdge mvregCRDTEdge;
            for (auto &kv_dots : v.dk.ds) {
                IDL::PairInt pi;
                pi.first(kv_dots.first.first);
                pi.second(kv_dots.first.second);

                mvregCRDTEdge.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLEdge(id)));
                mvregCRDTEdge.dk().cbase().cc().emplace(kv_dots.first);

            }

            mvregCRDTEdge.id(id);
            mvregCRDTEdge.agent_id(v.read_reg().agent_id());
            mvregCRDTEdge.to(k.first);
            mvregCRDTEdge.from(v.read_reg().from());
            mvregCRDTEdge.type(k.second);
            IDL::EdgeKey ek;
            ek.to(k.first);
            ek.type(k.second);
            node.fano().emplace(ek, std::move(mvregCRDTEdge));
        }
        return node;
    }

}

