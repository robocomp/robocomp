/*
Reimplementation from https://github.com/CBaquero/delta-enabled-crdts
*/

#ifndef DELTA_CRDT
#define DELTA_CRDT

#include <set>
#include <unordered_set>
#include <map>
#include <list>
#include <tuple>
#include <vector>
#include <string>
#include <iostream>
#include <type_traits>
#include <cassert>

using key_type = uint64_t;


// Autonomous causal context, for context sharing in maps
class dot_context {
public:
    
    std::map<key_type, int> cc; // Compact causal context
    std::set<std::pair<key_type, int> > dc; // Dot cloud



    dot_context &operator=(const dot_context &o) {
        if (&o == this) return *this;
        cc = o.cc;
        dc = o.dc;
        return *this;
    }

    dot_context &operator=(dot_context &&o) {
        if (&o == this) return *this;
        cc = std::move(o.cc);
        dc = std::move(o.dc);
        return *this;
    }

    void setContext(std::map<key_type, int> &cc_, std::set<std::pair<key_type, int> > &dc_) {
        cc = cc_;
        dc = dc_;
    }

    void setContext(std::map<key_type, int> &&cc_, std::set<std::pair<key_type, int> > &&dc_) {
        cc = std::move(cc_);
        dc = std::move(dc_);
    }

    const std::map<key_type, int>& compact_causal_context() {
        return cc;
    }

    const std::set<std::pair<key_type, int> >& dot_cloud() {
        return dc;
    }

    bool dotin(const std::pair<key_type, int> &d) const {
        const auto itm = cc.find(d.first);
        if (itm != cc.end() && d.second <= itm->second) return true;
        if (not dc.empty() and d.second < dc.rbegin()->second) return true;
        if (dc.count(d) != 0) return true;
        return false;
    }

    //TODO: debug this
    void compact() {
        // Compact DC to CC if possible
        //typename map<K,int>::iterator mit;
        //typename set<pair<K,int> >::iterator sit;
        bool flag; // may need to compact several times if ordering not best
        do {
            flag = false;
            for (auto sit = dc.begin(); sit != dc.end();) {

                auto mit = cc.find(sit->first);
                if (mit == cc.end()) // No CC entry
                    if (sit->second == 1) // Can compact
                    {
                        cc.insert(*sit);
                        dc.erase(sit++);
                        flag = true;
                    } else ++sit;
                else // there is a CC entry already
                if (sit->second == cc.at(sit->first) + 1) // Contiguous, can compact
                {
                    cc.at(sit->first)++;
                    dc.erase(sit++);
                    flag = true;
                } else if (sit->second <= cc.at(sit->first)) // dominated, so prune
                {
                    dc.erase(sit++);
                    // no extra compaction oportunities so flag untouched
                } else ++sit;
            }
        } while (flag == true);
    }

    std::pair<key_type, int> makedot(const key_type &id) {
        // On a valid dot generator, all dots should be compact on the used id
        // Making the new dot, updates the dot generator and returns the dot
        if (auto [it, res] = cc.insert(std::make_pair(id, 1)) ; !res) {
            it->second+=1;
            return *it;
        } else {
            return *it;
        }
    }

    void insertdot(const std::pair<key_type, int> &d, bool compactnow = true) {
        // Set
        dc.insert(d);
        if (compactnow) {
            compact();
        }
    }

    void join(const dot_context &o) {
        if (this == &o) return; // Join is idempotent, but just dont do it.
        // CC
        auto mit = cc.begin();
        auto mito = o.cc.begin();
        do {
            if (mit != cc.end() && (mito == o.cc.end() || mit->first < mito->first)) {
                // entry only at here
                ++mit;
            } else if (mito != o.cc.end() && (mit == cc.end() || mito->first < mit->first)) {
                // entry only at other
                cc.insert(*mito);
                ++mito;
            } else if (mit != cc.end() && mito != o.cc.end()) {
                // in both
                cc.at(mit->first) = std::max(mit->second, mito->second);
                ++mit;
                ++mito;
            }
        } while (mit != cc.end() || mito != o.cc.end());

        // DC
        // Set
        for (const auto &e : o.dc)
            insertdot(e, false); 

        compact();
    }

    friend std::ostream &operator<<(std::ostream &output, const dot_context &o) {
        output << "Context:";
        output << " CC ( ";
        for (const auto &ki : o.cc)
            output << ki.first << ":" << ki.second << " ";
        output << ")";
        output << " DC ( ";
        for (const auto &ki : o.dc)
            output << ki.first << ":" << ki.second << " ";
        output << ")";
        return output;
    }



    bool operator==(const dot_context &rhs) const {
        return cc == rhs.cc &&
               dc == rhs.dc;
    }

    bool operator!=(const dot_context &rhs) const {
        return !(rhs == *this);
    }

    bool operator<(const dot_context &rhs) const {
        if (cc < rhs.cc)
            return true;
        if (rhs.cc < cc)
            return false;
        return dc < rhs.dc;
    }

    bool operator>(const dot_context &rhs) const {
        return rhs < *this;
    }

    bool operator<=(const dot_context &rhs) const {
        return !(rhs < *this);
    }

    bool operator>=(const dot_context &rhs) const {
        return !(*this < rhs);
    }
};



template<typename T>
class dot_kernel {
public:
    
    std::map<std::pair<key_type, int>, T> ds;  // Map of dots to vals
    dot_context c;

    // if no causal context supplied, used base one
    dot_kernel() {}

    // if supplied, use a shared causal context
    dot_kernel(const dot_context &jointc)
    {
        c = jointc;
    }

    dot_kernel(const dot_kernel &o)
    {
        ds = o.ds;
        c = o.c;
    }

    dot_kernel(dot_kernel &&o)
    {
        ds = std::move(o.ds);
        c = std::move(o.c);
    }

    dot_kernel(dot_context &&jointc)
    {
        c = std::move(jointc);
    }

    dot_kernel<T> &operator=(const dot_kernel<T> &o)
    {
        if (&o == this) return *this;
        ds = o.ds;
        c = o.c;
        return *this;
    }

    dot_kernel<T> &operator=(dot_kernel<T> &&o)
    {
        if (&o == this) return *this;
        ds = std::move(o.ds);
        c = std::move(o.c);
        return *this;
    }


    void dot_map(std::map<std::pair<key_type, int>, T> &ds_)
    { 
        ds = ds_; 
    }

    void dot_map(std::map<std::pair<key_type, int>, T> &&ds_)
    { 
        ds = std::move(ds_); 
    }

    void join_replace_conflict(dot_kernel<T> &&o) {

        if (this == &o) return; // Join is idempotent, but just dont do it.

        // DS
        // will iterate over the two sorted sets to compute join
        auto it = ds.begin();
        auto ito = o.ds.begin();
        do {
            if (it != ds.end() && (ito == o.ds.end() || it->first < ito->first)) {
                // dot only at this
                if (o.c.dotin(it->first)) { // other knows dot, must delete here
                    ds.erase(it++);
                } else {// keep it
                    ++it;
                }
            } else if (ito != o.ds.end() && (it == ds.end() || ito->first < it->first)) {
                // dot only at other
                if (!c.dotin(ito->first) || ds.empty()) { // If I dont know, import
                    ds.insert(std::move(*ito));
                }
                ++ito;
            } else if (it != ds.end() && ito != o.ds.end()) {
                // dot in both
                //replace in case of conflict if the agent id has a lower value
                if (it->second.agent_id() > ito->second.agent_id() && *it != *ito) {
                    it = ds.erase(it);
                    ds.insert(std::move(*ito));
                } else {
                    ++it;
                }
                ++ito;
            }
        } while (it != ds.end() || ito != o.ds.end());
        // CC
        c.join(std::move(o.c));
    }

    dot_kernel<T> add(key_type &id, const T &val) {

        dot_kernel<T> res;
        // get new dot
        std::pair<key_type , int> dot = c.makedot(id);
        // add under new dot
        ds.insert(std::pair<std::pair<key_type, int>, T>(dot, val));
        // make delta
        res.ds.insert(std::pair<std::pair<key_type, int>, T>(dot, val));
        res.c.insertdot(dot);
        return res;
    }


    dot_kernel<T> add(key_type &id, T &&val) {

        dot_kernel<T> res;
        // get new dot
        std::pair<key_type , int> dot = c.makedot(id);
        // add under new dot
        ds.insert(std::pair<std::pair<key_type , int>, T>(dot, val));
        // make delta
        res.ds.insert(std::pair<std::pair<key_type , int>, T>(dot, val));
        res.c.insertdot(dot);
        return res;
    }

    // Add that returns the added dot, instead of kernel delta
    std::pair<key_type , int> dotadd(const key_type &id, const T &val) {
        // get new dot
        std::pair<key_type , int> dot = c.makedot(id);
        // add under new dot
        ds.insert(std::pair<std::pair<key_type , int>, T>(dot, val));
        return dot;
    }


    void clean() {

        std::map<key_type , int> x;
        for (auto dsit = ds.begin(); dsit != ds.end(); dsit++) {
            if (x.find(dsit->first.first) != x.end()) // match
            {
                if (x[dsit->first.first] < dsit->first.second) {
                    x[dsit->first.first] = dsit->first.second;
                }
            } else {
                x.insert(dsit->first);
            }
        }

        for (auto dsit = ds.begin(); dsit != ds.end();) {
            if (x[dsit->first.first] != dsit->first.second)
                ds.erase(dsit++);
            else {
                ++dsit;
            }
        }

    }


    dot_kernel<T> rmv(const T &val)  // remove all dots matching value
    {
        dot_kernel<T> res;
        for (auto dsit = ds.begin(); dsit != ds.end();) {
            if (dsit->second == val) // match
            {
                res.c.insertdot(dsit->first, false); // result knows removed dots
                ds.erase(dsit++);
            } else
                ++dsit;
        }
        res.c.compact(); // Maybe several dots there, so atempt compactation
        return res;
    }


    dot_kernel<T> rmv(const std::pair<key_type , int> &dot)  // remove a dot
    {
        dot_kernel<T> res;
        auto dsit = ds.find(dot);
        if (dsit != ds.end()) // found it
        {
            res.c.insertdot(dsit->first, false); // result knows removed dots
            ds.erase(dsit++);
        }
        ////cout <<__PRETTY_FUNCTION__ <<":"<<__LINE__<< " " << res << endl;
        res.c.compact(); // Atempt compactation
        return res;
    }

    dot_kernel<T> rmv()  // remove all dots
    {
        dot_kernel<T> res;
        for (const auto &dv : ds)
            res.c.insertdot(dv.first, false);
        res.c.compact();
        ds.clear(); // Clear the payload, but remember context
        return res;
    }

    friend std::ostream &operator<<(std::ostream &output, const dot_kernel<T> &o) {
        output << "Kernel: DS ( ";
        for (const auto &dv : o.ds) {
            output << dv.first.first << ":" << dv.first.second <<
                   "->" << dv.second << " ";
        }
        output << ") ";

        output << o.c;
        return output;
    }



    bool operator==(const dot_kernel &rhs) const {
        return ds == rhs.ds;// &&
        //cbase == rhs.cbase &&
        //c == rhs.c;
    }

    bool operator!=(const dot_kernel &rhs) const {
        return !(rhs == *this);
    }

    bool operator<(const dot_kernel &rhs) const {
        if (ds < rhs.ds)
            return true;
        return false;
    }

    bool operator>(const dot_kernel &rhs) const {
        return rhs < *this;
    }

    bool operator<=(const dot_kernel &rhs) const {
        return !(rhs < *this);
    }

    bool operator>=(const dot_kernel &rhs) const {
        return !(*this < rhs);
    }
};






template<typename V>
class mvreg    // Multi-value register, Optimized
{
public:
    key_type id;
    dot_kernel<V> dk; // Dot kernel

    mvreg(): id(0) {}

    mvreg(const mvreg &o) {
        dk = o.dk;
        id = o.id;
    }

    mvreg(mvreg &&o) {
        dk = std::move(o.dk);
        id = o.id;
    }

    mvreg &operator=(const mvreg &o) {
        if (&o == this) return *this;
        dk = o.dk;
        id = o.id;
        return *this;
    }

    mvreg &operator=(mvreg &&o) {
        if (&o == this) return *this;
        dk = std::move(o.dk);
        id = o.id;
        return *this;
    }

    dot_context &context() {
        return dk.c;
    }

    dot_context &context() const {
        return dk.c;
    }

    void set_context(const dot_context &cbase) {
        dk.c = cbase;
    }

    void set_context(dot_context &&cbase) {
        dk.c = std::move(cbase);
    }

    mvreg<V> write(const V &val) {
        mvreg<V> r, a;
        r.dk = dk.rmv();
        a.dk = dk.add(id, val);
        r.join(std::move(a));
        assert(r.dk.ds.size() <= 1);
        return r;
    }

    mvreg<V> write(V &&val) {
        mvreg<V> r, a;
        r.dk = dk.rmv();
        a.dk = dk.add(id, std::move(val));
        r.join(std::move(a));
        assert(r.dk.ds.size() <= 1);
        return r;
    }

    const V &read_reg() const {
        return dk.ds.begin()->second;
    }

    V &read_reg() {
        return dk.ds.begin()->second;
    }


    bool empty() {
        return dk.ds.empty();
    }

    friend std::ostream &operator<<(std::ostream &output, const mvreg<V> &o) {
        output << "MVReg:" << o.dk;
        return output;
    }

    mvreg<V> reset() {
        mvreg<V> r;
        r.dk = dk.rmv();
        return r;
    }

    void join(mvreg<V> &&o) {
        dk.join_replace_conflict(std::move(o.dk));
        assert(dk.ds.size() <= 1);
    }

    bool operator==(const mvreg &rhs) const {
        return id == rhs.id &&
               dk == rhs.dk;
    }

    bool operator!=(const mvreg &rhs) const {
        return !(rhs == *this);
    }

    bool operator<(const mvreg &rhs) const {
        if (id < rhs.id)
            return true;
        if (rhs.id < id)
            return false;
        return dk < rhs.dk;
    }

    bool operator>(const mvreg &rhs) const {
        return rhs < *this;
    }

    bool operator<=(const mvreg &rhs) const {
        return !(rhs < *this);
    }

    bool operator>=(const mvreg &rhs) const {
        return !(*this < rhs);
    }
};


#endif