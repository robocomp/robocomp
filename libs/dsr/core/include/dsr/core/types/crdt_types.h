//
// Created by juancarlos on 8/6/20.
//

#ifndef DSR_CRDT_TYPES_H
#define DSR_CRDT_TYPES_H


#include <iostream>
#include <unordered_map>
#include <variant>
#include <map>

#include "../crdt/delta_crdt.h"
#include "../topics/IDLGraph.h"
#include "../utils.h"
#include "common_types.h"

namespace DSR {

    //using CRDTAttribute = DSR::Attribute;
    typedef DSR::Attribute CRDTAttribute;

    class CRDTEdge
    {
    public:

        CRDTEdge() : m_to(0), m_from(0), m_agent_id(0) {}

        ~CRDTEdge() = default;

        explicit CRDTEdge (IDL::IDLEdge &&x) noexcept;

        CRDTEdge &operator=(IDL::IDLEdge &&x);

        void to(uint64_t  _to);

        [[nodiscard]] uint64_t  to() const;

        void type(const std::string &_type);

        void type(std::string &&_type);

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        void from(uint64_t  _from);

        [[nodiscard]] uint64_t from() const;

        void attrs(const std::map<std::string, mvreg<CRDTAttribute>> &_attrs);

        void attrs(std::map<std::string, mvreg<CRDTAttribute>> &&_attrs);

        [[nodiscard]] const std::map<std::string, mvreg<CRDTAttribute>> &attrs() const;

        [[nodiscard]] std::map<std::string, mvreg<CRDTAttribute>> &attrs();

        void agent_id(uint32_t _agent_id);

        [[nodiscard]] uint32_t agent_id() const;

        [[nodiscard]] IDL::IDLEdge toIDLEdge(uint64_t id);


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
            output << "IDL::EdgeAttribs[" << ea_.m_type << ", from:" << std::to_string(ea_.from()) << "-> to:" << std::to_string(ea_.to())
                   << " Attribs:[";
            for (const auto &v : ea_.attrs())
                output << v.first << ":" << v.second << " - ";
            output << "]]";
            return output;
        };

    private:
        uint64_t m_to;
        std::string m_type;
        uint64_t  m_from;
        std::map<std::string, mvreg<CRDTAttribute>> m_attrs;
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

        void type(const std::string &_type);

        void type(std::string &&_type);

        [[nodiscard]] const std::string &type() const;

        [[nodiscard]] std::string &type();

        void name(const std::string &_name);

        void name(std::string &&_name);

        [[nodiscard]] const std::string &name() const;

        [[nodiscard]] std::string &name();

        void id(uint64_t _id);

        [[nodiscard]] uint64_t id() const;

        void agent_id(uint32_t _agent_id);

        [[nodiscard]] uint32_t agent_id() const;

        void attrs(const std::map<std::string, mvreg<CRDTAttribute>> &_attrs);

        void attrs(std::map<std::string, mvreg<CRDTAttribute>> &&_attrs);

        [[nodiscard]] std::map<std::string, mvreg<CRDTAttribute>> &attrs() &;

        [[nodiscard]] const std::map<std::string, mvreg<CRDTAttribute>> &attrs() const &;

        void fano(const std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &_fano);

        void fano(std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &&_fano);

        [[nodiscard]] std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &fano();

        [[nodiscard]] const std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> &fano() const;

        [[nodiscard]] IDL::IDLNode toIDLNode(uint64_t id);

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
            output << "IDL::Node:[" << std::to_string(n_.id()) << "," << n_.name() << "," << n_.type() << "], Attribs:[";
            for (const auto &v : n_.attrs())
                output << v.first << ":(" << v.second << ");";
            output << "], FanOut:[";
            for (auto &v : n_.fano())
                output << "[ " << std::to_string(v.first.first) << " " << v.first.second << "] " << ":(" << v.second << ");";
            output << "]";
            return output;
        }
    private:
        std::string m_type;
        std::string m_name;
        uint64_t m_id{};
        uint32_t m_agent_id{};
        std::map<std::string, mvreg<CRDTAttribute>> m_attrs;
        std::map<std::pair<uint64_t, std::string>, mvreg<CRDTEdge>> m_fano;
    };


}

#endif //DSR_CRDT_TYPES_H
