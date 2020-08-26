//
// Created by juancarlos on 8/6/20.
//


#include "crdt_types.h"
#include "translator.h"

namespace DSR {
    
    int32_t CRDTValue::selected() const 
    {
        return val.index();
    }

    std::string &CRDTValue::str() 
    {
        if (auto pval = std::get_if<std::string>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("STRING is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());

    }

    const std::string &CRDTValue::str() const 
    {
        if (auto pval = std::get_if<std::string>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("STRING is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());

    }

    void CRDTValue::str(const std::string &_str) 
    {
        val = _str;
    }

    void CRDTValue::str(std::string &&_str) 
    {
        val = std::move(_str);
    }

    void CRDTValue::dec(int32_t _dec) 
    {
        val = _dec;
    }

    int32_t CRDTValue::dec() const 
    {
        if (auto pval = std::get_if<int32_t>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("INT is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());

    }

    void CRDTValue::uint(uint32_t _uint) 
    {
        val = _uint;
    }

    uint32_t CRDTValue::uint() const 
    {
        if (auto pval = std::get_if<uint32_t>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("UINT is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());

    }

    void CRDTValue::fl(float _fl) 
    {
        val = _fl;
    }

    float CRDTValue::fl() const 
    {
        if (auto pval = std::get_if<float>(&val)) {
            return *pval;
        }

        throw std::runtime_error(
                ("FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    void CRDTValue::float_vec(const std::vector<float> &_float_vec) 
    {
        val = _float_vec;
    }

    void CRDTValue::float_vec(std::vector<float> &&_float_vec) 
    {
        val = std::move(_float_vec);
    }

    const std::vector<float> &CRDTValue::float_vec() const 
    {
        if (auto pval = std::get_if<vector<float>>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    std::vector<float> &CRDTValue::float_vec() 
    {

        if (auto pval = std::get_if<vector<float>>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_FLOAT is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    void CRDTValue::bl(bool _bl) 
    {
        val = _bl;
    }

    bool CRDTValue::bl() const 
    {

        if (auto pval = std::get_if<bool>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("BOOL is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    void CRDTValue::byte_vec(const std::vector<uint8_t> &_float_vec) 
    {
        val = _float_vec;
    }

    void CRDTValue::byte_vec(std::vector<uint8_t> &&_float_vec) 
    {
        val = std::move(_float_vec);
    }

    const std::vector<uint8_t> &CRDTValue::byte_vec() const 
    {
        if (auto pval = std::get_if<vector<uint8_t>>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_BYTE is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    std::vector<uint8_t> &CRDTValue::byte_vec() 
    {

        if (auto pval = std::get_if<vector<uint8_t >>(&val)) {
            return *pval;
        }
        throw std::runtime_error(
                ("VECTOR_BYTE is not selected, selected is " + std::string(TYPENAMES_UNION[val.index()])).data());
    }

    IDL::Val CRDTValue::toIDLVal() 
    {
        IDL::Val value;

        switch (val.index()) {
            case 0:
                value.str(std::get<std::string>(val));
                break;
            case 1:
                value.dec(std::get<int32_t>(val));
                break;
            case 2:
                value.fl(std::get<float>(val));
                break;
            case 3:
                value.float_vec(std::get<std::vector<float>>(val));
                break;
            case 4:
                value.bl(std::get<bool>(val));
                break;
            case 5:
                value.byte_vec(std::get<std::vector<uint8_t>>(val));
                break;
            case 6:
                value.uint(std::get<std::uint32_t>(val));
                break;
            default:
                throw std::runtime_error(
                        ("Error converting CRDT::CRDTAttribute to IDL::Attrib. The CRDTAttribute is uninitialized. " +
                         std::to_string(__LINE__) + " " + __FILE__).data());
        }

        return value;
    }
    
    void CRDTAttribute::type(uint32_t _type) 
    {
        m_type = _type;
    }
    
    uint32_t CRDTAttribute::type() const 
    {
        return m_type;
    }

    void CRDTAttribute::timestamp(uint64_t _time) 
    {
        m_timestamp = _time;
    }


    uint64_t CRDTAttribute::timestamp() const 
    {
        return m_timestamp;
    }


    void CRDTAttribute::val(IDL::Val &&CRDTValue_) 
    {
        m_Value = CRDTValue(std::move(CRDTValue_));
    }

    void CRDTAttribute::val(CRDTValue &&CRDTValue_) 
    {
        m_Value = std::move(CRDTValue_);
    }

    const CRDTValue &CRDTAttribute::val() const 
    {
        return m_Value;
    }

    CRDTValue &CRDTAttribute::val() 
    {
        return m_Value;
    }

    void CRDTAttribute::agent_id(uint32_t _agent_id) 
    {
        m_agent_id = _agent_id;
    }

    uint32_t CRDTAttribute::agent_id() const 
    {
        return m_agent_id;
    }

    IDL::Attrib CRDTAttribute::toIDLAttrib() 
    {
        IDL::Attrib att;
        att.timestamp(m_timestamp);
        att.type(m_type);
        att.value(m_Value.toIDLVal());
        att.agent_id(m_agent_id);
        return att;
    }


    CRDTEdge &CRDTEdge::operator=(IDL::IDLEdge &&x) 
    {

        m_to = x.to();
        m_type = std::move(x.type());
        m_from = x.from();
        if (!x.attrs().empty()) {
            for (auto&[k, v] : x.attrs()) {
                m_attrs[k] = translate_edge_attr_mvIDL_to_CRDT(v);
            }
        }
        m_agent_id = x.agent_id();

        return *this;
    }


    void CRDTEdge::to(uint32_t _to) 
    {
        m_to = _to;
    }
    
    uint32_t CRDTEdge::to() const 
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

    void CRDTEdge::from(uint32_t _from)
    {
        m_from = _from;
    }

    uint32_t CRDTEdge::from() const
    {
        return m_from;
    }

    void CRDTEdge::attrs(const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &_attrs)
    {
        m_attrs = _attrs;
    }

    void CRDTEdge::attrs(std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &&_attrs)
    {
        m_attrs = std::move(_attrs);
    }

    const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &CRDTEdge::attrs() const
    {
        return m_attrs;
    }

    std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &CRDTEdge::attrs()
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

    IDL::IDLEdge CRDTEdge::toIDLEdge(uint32_t id)
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

                edgeAttr.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
                edgeAttr.dk().cbase().cc().emplace(kv_dots.first);

            }

            edgeAttr.from(m_from);
            edgeAttr.to(m_to);
            edgeAttr.type(m_type);
            edgeAttr.agent_id(v.read_reg().agent_id());
            edgeAttr.id(id);

            edge.attrs()[k] = edgeAttr;
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
            m_attrs[k] = translate_node_attr_mvIDL_to_CRDT(v);
        }
        for (auto&[k, v] : x.fano()) {
            m_fano[make_pair(k.to(), k.type())] = translate_edge_mvIDL_to_CRDT(v);
        }
    }

    CRDTNode::CRDTNode(IDL::IDLNode& x)
    {
        m_type = x.type();
        m_name = x.name();
        m_id = x.id();
        m_agent_id = x.agent_id();
        for (auto&[k, v] : x.attrs()) {
            m_attrs[k] = translate_node_attr_mvIDL_to_CRDT(v);
        }
        for (auto&[k, v] : x.fano()) {
            m_fano[make_pair(k.to(), k.type())] = translate_edge_mvIDL_to_CRDT(v);
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

    void CRDTNode::id(uint32_t _id)
    {
        m_id = _id;
    }

    uint32_t CRDTNode::id() const
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

    void CRDTNode::attrs(const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &_attrs)
    {
        m_attrs = _attrs;
    }

    void CRDTNode::attrs(std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &&_attrs)
    {
        m_attrs = std::move(_attrs);
    }

    std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &CRDTNode::attrs() &
    {
        return m_attrs;
    }

    const std::map<std::string, mvreg<CRDTAttribute, uint32_t>> &CRDTNode::attrs() const &
    {
        return m_attrs;
    }

    void CRDTNode::fano(const std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &_fano)
    {
        m_fano = _fano;
    }

    void CRDTNode::fano(std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &&_fano)
    {
        m_fano = std::move(_fano);
    }

    std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &CRDTNode::fano()
    {
        return m_fano;
    }

    const std::map<std::pair<uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>> &CRDTNode::fano() const
    {
        return m_fano;
    }


    IDL::IDLNode CRDTNode::toIDLNode(uint32_t id)
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

                nodeAttr.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
                nodeAttr.dk().cbase().cc().emplace(kv_dots.first);
            }

            nodeAttr.id(id);
            nodeAttr.attr_name(k);
            nodeAttr.agent_id(v.read().begin()->agent_id());
            node.attrs()[k] = nodeAttr;
        }

        for (auto &[k, v] : m_fano) {
            IDL::MvregEdge mvregCRDTEdge;
            for (auto &kv_dots : v.dk.ds) {
                IDL::PairInt pi;
                pi.first(kv_dots.first.first);
                pi.second(kv_dots.first.second);

                mvregCRDTEdge.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLEdge(id)));
                mvregCRDTEdge.dk().cbase().cc().emplace(kv_dots.first);

            }

            mvregCRDTEdge.id(id);
            mvregCRDTEdge.agent_id(v.read().begin()->agent_id());
            mvregCRDTEdge.to(k.first);
            mvregCRDTEdge.from(v.read().begin()->from());
            mvregCRDTEdge.type(k.second);
            IDL::EdgeKey ek;
            ek.to(k.first);
            ek.type(k.second);
            node.fano()[ek] = mvregCRDTEdge;
        }
        return node;
    }

}

