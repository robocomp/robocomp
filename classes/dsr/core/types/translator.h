//
// Created by juancarlos on 1/7/20.
//

#ifndef CONVERTER_H
#define CONVERTER_H

#include "user_types.h"

namespace DSR {

    // Translators
    inline static IDL::Mvreg translate_node_mvCRDT_to_IDL(uint32_t agent_id, uint32_t id, mvreg<CRDTNode, uint32_t> &data)
    {
        IDL::Mvreg delta_crdt;
        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLNode(id)));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.context().getCcDc().second) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.id(id);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

    inline static mvreg<CRDTNode, uint32_t> translate_node_mvIDL_to_CRDT(IDL::Mvreg &data)
    {
        // Context
        dotcontext<uint32_t> dotcontext_aux;
        std::map<uint32_t, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<uint32_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<uint32_t, int>, CRDTNode> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<uint32_t, int>(k.first(), k.second())] = CRDTNode(v);
        // Join
        mvreg<CRDTNode, uint32_t> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdgeAttr
    translate_edge_attr_mvCRDT_to_IDL(uint32_t agent_id, uint32_t id, uint32_t from, uint32_t to, const std::string &type,
                                      const std::string &attr, mvreg<CRDTAttribute, uint32_t> &data)
    {
        IDL::MvregEdgeAttr delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.type(type);
        delta_crdt.id(id);
        delta_crdt.attr_name(attr);
        delta_crdt.from(from);
        delta_crdt.to(to);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;

    }

    inline static mvreg<CRDTAttribute, uint32_t> translate_edge_attr_mvIDL_to_CRDT(IDL::MvregEdgeAttr &data)
    {
        // Context
        dotcontext<uint32_t> dotcontext_aux;
        std::map<uint32_t, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<uint32_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<uint32_t, int>, CRDTAttribute> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<uint32_t, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTAttribute, uint32_t> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregNodeAttr
    translate_node_attr_mvCRDT_to_IDL(uint32_t agent_id, uint32_t id, uint32_t node, const std::string &attr, mvreg<CRDTAttribute, uint32_t> &data)
    {
        IDL::MvregNodeAttr delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }

        delta_crdt.id(id);
        delta_crdt.attr_name(attr);
        delta_crdt.node(node);
        delta_crdt.agent_id(agent_id);

        return delta_crdt;
    }

    inline static mvreg<CRDTAttribute, uint32_t> translate_node_attr_mvIDL_to_CRDT(IDL::MvregNodeAttr &data)
    {
        // Context
        dotcontext<uint32_t> dotcontext_aux;
        std::map<uint32_t, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<uint32_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<uint32_t, int>, CRDTAttribute> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<uint32_t, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTAttribute, uint32_t> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static mvreg<CRDTEdge, uint32_t> translate_edge_mvIDL_to_CRDT(IDL::MvregEdge &data)
    {
        // Context
        dotcontext<uint32_t> dotcontext_aux;
        std::map<uint32_t, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<uint32_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<uint32_t, int>, CRDTEdge> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<uint32_t, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTEdge, uint32_t> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdge translate_edge_mvCRDT_to_IDL(uint32_t agent_id, uint32_t from, uint32_t to, const std::string& type , mvreg<CRDTEdge, uint32_t> &data)
    {
        IDL::MvregEdge delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            auto edge = kv_dots.second.toIDLEdge(from);
            delta_crdt.dk().ds().emplace(make_pair(pi, edge));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);

        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.from(from);
        delta_crdt.to(to);
        delta_crdt.type(type);
        delta_crdt.id(from);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

    inline static CRDTAttribute user_attribute_to_crdt(Attribute&& attr)
    {

        CRDTAttribute attribute;
        CRDTValue val;
        attribute.type(attr.value().index());
        val.variant(std::move(attr.value()));
        attribute.val(std::move(val));
        attribute.agent_id(attr.agent_id());
        attribute.timestamp(attr.timestamp());

        return attribute;
    }

    inline static CRDTAttribute user_attribute_to_crdt(const Attribute& attr)
    {

        CRDTAttribute attribute;
        CRDTValue val;
        attribute.type(attr.value().index());
        val.variant(attr.value());
        attribute.val(std::move(val));
        attribute.agent_id(attr.agent_id());
        attribute.timestamp(attr.timestamp());

        return attribute;
    }

    inline static CRDTEdge user_edge_to_crdt(Edge&& edge)
    {
        CRDTEdge crdt_edge;

        crdt_edge.agent_id(edge.agent_id());
        crdt_edge.from(edge.from());
        crdt_edge.to(edge.to());
        crdt_edge.type(std::move(edge.type()));
        for (auto &&[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute, uint32_t> mv;
            mv.write(user_attribute_to_crdt(std::move(v)));
            crdt_edge.attrs().emplace(k, std::move(mv));
        }

        return crdt_edge;
    }

    inline static CRDTEdge user_edge_to_crdt(const Edge& edge)
    {
        CRDTEdge crdt_edge;

        crdt_edge.agent_id(edge.agent_id());
        crdt_edge.from(edge.from());
        crdt_edge.to(edge.to());
        crdt_edge.type(edge.type());
        for (auto &&[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute, uint32_t> mv;
            mv.write(user_attribute_to_crdt(v));
            crdt_edge.attrs().emplace(k, mv);
        }

        return crdt_edge;
    }


    inline static CRDTNode user_node_to_crdt(Node&& node)
    {
        CRDTNode crdt_node;

        crdt_node.agent_id(node.agent_id());
        crdt_node.id(node.id());
        crdt_node.type(std::move(node.type()));
        crdt_node.name(std::move(node.name()));
        for (auto &&[k,v] : node.attrs()) {
            mvreg<CRDTAttribute, uint32_t> mv;
            mv.write(user_attribute_to_crdt(std::move(v)));
            crdt_node.attrs().emplace(k, std::move(mv));
        }

        for (auto &&[k,v] : node.fano()) {
            mvreg<CRDTEdge, uint32_t> mv;
            mv.write(user_edge_to_crdt(std::move(v)));
            crdt_node.fano().emplace(k, std::move(mv));
        }
        return crdt_node;
    }


    inline static CRDTNode user_node_to_crdt(const Node& node)
    {
        CRDTNode crdt_node;

        crdt_node.agent_id(node.agent_id());
        crdt_node.id(node.id());
        crdt_node.type(node.type());
        crdt_node.name(node.name());
        for (auto &&[k,v] : node.attrs()) {
            mvreg<CRDTAttribute, uint32_t> mv;
            mv.write(user_attribute_to_crdt(v));
            crdt_node.attrs().emplace(k, mv);
        }

        for (auto &&[k,v] : node.fano()) {
            mvreg<CRDTEdge, uint32_t> mv;
            mv.write(user_edge_to_crdt(v));
            crdt_node.fano().emplace(k, mv);
        }
        return crdt_node;
    }
}


#endif //CONVERTER_H
