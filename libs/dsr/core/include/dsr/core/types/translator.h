//
// Created by juancarlos on 1/7/20.
//

#ifndef CONVERTER_H
#define CONVERTER_H

#include "user_types.h"
#include "crdt_types.h"
#include <cassert>

namespace DSR {

    // Translators
    inline static IDL::MvregNode CRDTNode_to_IDL(uint32_t agent_id, uint64_t id, mvreg<DSR::CRDTNode> &data)
    {
        IDL::MvregNode delta_crdt;
        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLNode(id)));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.dk.c.dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().emplace_back(std::move(pi));
        }

        delta_crdt.dk().cbase().cc(data.dk.c.cc);

        delta_crdt.id(id);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

    inline static mvreg<CRDTNode> IDLNode_to_CRDT(IDL::MvregNode &&data)
    {
        // Context
        dot_context dotcontext_aux;
        std::map<uint64_t , int> m = std::move(data.dk().cbase().cc());
        std::set<std::pair<uint64_t , int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(std::move(m), std::move(s));
        // Dots
        std::map<std::pair<uint64_t , int>, CRDTNode> ds_aux;
        for (auto &&val : std::move(data.dk().ds()))
            ds_aux.emplace(std::pair<uint64_t , int>(val.first.first(), val.first.second()), CRDTNode(std::move(val.second)));

        // Join
        mvreg<CRDTNode> aw;
        aw.dk.c = std::move(dotcontext_aux);
        aw.dk.dot_map(std::move(ds_aux));
        return aw;
    }

    inline static IDL::MvregEdgeAttr
    CRDTEdgeAttr_to_IDL(uint32_t agent_id, uint64_t id, uint64_t from, uint64_t to, const std::string &type,
                                      const std::string &attr, mvreg<CRDTAttribute> &data)
    {
        IDL::MvregEdgeAttr delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().emplace_back(std::move(pi));
        }

        delta_crdt.dk().cbase().cc(data.dk.c.cc);

        delta_crdt.type(type);
        delta_crdt.id(id);
        delta_crdt.attr_name(attr);
        delta_crdt.from(from);
        delta_crdt.to(to);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;

    }

    inline static mvreg<CRDTAttribute> IDLEdgeAttr_to_CRDT(IDL::MvregEdgeAttr &&data)
    {
        // Context
        dot_context dotcontext_aux;
        std::map<uint64_t, int> m = std::move(data.dk().cbase().cc());

        std::set<std::pair<uint64_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(std::move(m), std::move(s));
        // Dots
        std::map<std::pair<uint64_t, int>, CRDTAttribute> ds_aux;
        for (auto &val : data.dk().ds()) {
            auto&& attrib  = std::move(val.second);
            ds_aux.emplace(std::pair<uint64_t, int>(val.first.first(), val.first.second()),
                           CRDTAttribute(std::move(attrib)));
        }
        // Join
        mvreg<CRDTAttribute> aw;
        aw.dk.c = std::move(dotcontext_aux);
        aw.dk.dot_map(std::move(ds_aux));
        return aw;
    }

    inline static IDL::MvregNodeAttr
    CRDTNodeAttr_to_IDL(uint32_t agent_id, uint64_t id, uint64_t node, const std::string &attr, mvreg<CRDTAttribute> &data)
    {
        IDL::MvregNodeAttr delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(std::make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().emplace_back(std::move(pi));
        }

        delta_crdt.dk().cbase().cc(data.dk.c.cc);


        delta_crdt.id(id);
        delta_crdt.attr_name(attr);
        delta_crdt.node(node);
        delta_crdt.agent_id(agent_id);

        return delta_crdt;
    }

    inline static mvreg<CRDTAttribute> IDLNodeAttr_to_CRDT(IDL::MvregNodeAttr &&data)
    {
        // Context
        dot_context dotcontext_aux;
        std::map<uint64_t, int> m = std::move(*(std::map<uint64_t, int32_t>*)&data.dk().cbase().cc());

        std::set<std::pair<uint64_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(std::move(m), std::move(s));
        // Dots
        std::map<std::pair<uint64_t, int>, CRDTAttribute> ds_aux;
        for (auto &val : data.dk().ds()) {
            auto&& attrib  = std::move(val.second);
            ds_aux.emplace(std::pair<uint64_t, int>(val.first.first(), val.first.second()),
                           CRDTAttribute(std::move(attrib)));
        }
        // Join
        mvreg<CRDTAttribute> aw;
        aw.dk.c = std::move(dotcontext_aux);
        aw.dk.dot_map(std::move(ds_aux));
        return aw;
    }

    inline static mvreg<CRDTEdge> IDLEdge_to_CRDT(IDL::MvregEdge &&data)
    {
        // Context
        dot_context dotcontext_aux;
        std::map<uint64_t, int> m = std::move(*(std::map<uint64_t, int32_t>*)&data.dk().cbase().cc());
        std::set<std::pair<uint64_t, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(std::move(m), std::move(s));
        // Dots
        std::map<std::pair<uint64_t, int>, CRDTEdge> ds_aux;
        for (auto &val : data.dk().ds()) {
            auto&& attrib  = std::move(val.second);
            ds_aux.emplace(std::pair<uint64_t, int>(val.first.first(), val.first.second()),
                           CRDTEdge(std::move(attrib)));
        }
        mvreg<CRDTEdge> aw;
        aw.dk.c = std::move(dotcontext_aux);
        aw.dk.dot_map(std::move(ds_aux));
        return aw;
    }

    inline static IDL::MvregEdge CRDTEdge_to_IDL(uint32_t agent_id, uint64_t from, uint64_t to, const std::string& type , mvreg<CRDTEdge> &data)
    {
        IDL::MvregEdge delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            auto edge = kv_dots.second.toIDLEdge(from);
            delta_crdt.dk().ds().emplace(std::make_pair(pi, edge));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);

        }

        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().emplace_back(std::move(pi));
        }

        delta_crdt.dk().cbase().cc(data.dk.c.cc);

        delta_crdt.from(from);
        delta_crdt.to(to);
        delta_crdt.type(type);
        delta_crdt.id(from);
        delta_crdt.agent_id(agent_id);

        return delta_crdt;
    }

    inline static CRDTEdge user_edge_to_crdt(Edge&& edge)
    {
        CRDTEdge crdt_edge;

        crdt_edge.agent_id(edge.agent_id());
        crdt_edge.from(edge.from());
        crdt_edge.to(edge.to());
        crdt_edge.type(std::move(edge.type()));
        for (auto &&[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute> mv;
            mv.write(std::move(v));
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
        for (auto &[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute> mv;
            mv.write(v);
            crdt_edge.attrs().emplace(k, std::move(mv));
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


        for (auto &&[k, val] : node.attrs()) {
            mvreg<CRDTAttribute> mv;
            mv.write(std::move(val));
            crdt_node.attrs().emplace(k, std::move(mv));
        }

        for (auto &[k,v] : node.fano()) {
            mvreg<CRDTEdge> mv;
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
        for (auto &[k,v] : node.attrs()) {
            mvreg<CRDTAttribute> mv;
            mv.write(v);
            crdt_node.attrs().emplace(k, std::move(mv));
        }

        for (auto &[k,v] : node.fano()) {
            mvreg<CRDTEdge> mv;
            mv.write(user_edge_to_crdt(v));
            crdt_node.fano().emplace(k, std::move(mv));
        }

        return crdt_node;
    }
}


#endif //CONVERTER_H
