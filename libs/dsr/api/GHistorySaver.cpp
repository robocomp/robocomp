//
// Created by juancarlos on 30/11/21.
//

#include <dsr/api/GHistorySaver.h>
#include <fstream>
#include <utility>


//////////////////////////
/// Serialize date impl
/////////////////////////
namespace {

////////////////////////////////
//////// Serialize
////////////////////////////////

    void serialize_string(const std::string& i, Serializer &ser)
    {
        ser.ser_att_var_size(i.data(), i.size(), DSR::STRING);
    }

    void serialize_int(const std::int32_t& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(std::int32_t), DSR::INT);
    }

    void serialize_uint(const std::uint32_t& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(std::uint32_t), DSR::UINT);
    }

    void serialize_uint64(const std::uint64_t& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(std::uint64_t), DSR::UINT64);
    }

    void serialize_float(const float& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(float), DSR::FLOAT);
    }

    void serialize_double(const double& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(double), DSR::DOUBLE);
    }

    void serialize_bool(const bool& i, Serializer &ser)
    {
        ser.ser_att(&i, sizeof(bool), DSR::BOOL);
    }

    void serialize_vecu8(const std::vector<uint8_t>& i, Serializer &ser)
    {
        ser.ser_att_var_size(i.data(), i.size(), DSR::BYTE_VEC);
    }

    void serialize_vecfloat(const std::vector<float>& i, Serializer &ser)
    {
        ser.ser_att_var_size(i.data(), i.size()*sizeof(float), DSR::FLOAT_VEC);
    }

    void serialize_vecu64(const std::vector<uint64_t>& i, Serializer &ser)
    {
        ser.ser_att_var_size(i.data(), i.size()*sizeof(std::uint64_t), DSR::U64_VEC);
    }

    void serialize_vecfloat2(const std::array<float, 2>& i, Serializer &ser)
    {
        ser.ser_att(i.data(), sizeof(float)*2, DSR::VEC2);
    }

    void serialize_vecfloat3(const std::array<float, 3>& i, Serializer &ser)
    {
        ser.ser_att(i.data(), sizeof(float)*3, DSR::VEC3);
    }

    void serialize_vecfloat4(const std::array<float, 4>& i, Serializer &ser)
    {
        ser.ser_att(i.data(), sizeof(float)*4, DSR::VEC4);
    }

    void serialize_vecfloat6(const std::array<float, 6>& i, Serializer &ser)
    {
        ser.ser_att(i.data(), sizeof(float)*6, DSR::VEC6);
    }

    void serialize_att(const DSR::Attribute& att, Serializer &ser)
    {
        serialize_uint64(att.timestamp(), ser);
        serialize_uint(att.agent_id(), ser);
        switch(att.selected())
        {
            case DSR::STRING:
                serialize_string(att.str(), ser);
                break;
            case DSR::INT:
                serialize_int(att.dec(), ser);
                break;
            case DSR::FLOAT:
                serialize_float(att.fl(), ser);
                break;
            case DSR::FLOAT_VEC:
                serialize_vecfloat(att.float_vec(), ser);
                break;
            case DSR::BOOL:
                serialize_bool(att.bl(), ser);
                break;
            case DSR::BYTE_VEC:
                serialize_vecu8(att.byte_vec(), ser);
                break;
            case DSR::UINT:
                serialize_uint(att.uint(), ser);
                break;
            case DSR::UINT64:
                serialize_uint64(att.uint64(), ser);
                break;
            case DSR::DOUBLE:
                serialize_double(att.dob(), ser);
                break;
            case DSR::U64_VEC:
                serialize_vecu64(att.u64_vec(), ser);
                break;
            case DSR::VEC2:
                serialize_vecfloat2(att.vec2(), ser);
                break;
            case DSR::VEC3:
                serialize_vecfloat3(att.vec3(), ser);
                break;
            case DSR::VEC4:
                serialize_vecfloat4(att.vec4(), ser);
                break;
            case DSR::VEC6:
                serialize_vecfloat6(att.vec6(), ser);
                break;
        }
    }



    void serialize_atts(const std::map<std::string, DSR::Attribute>& atts, Serializer &ser, const std::set<std::string>& ignore_atts={"cam_depth", "cam_rgb", "laser_angles", "laser_dists"})
    {
        size_t s = atts.size();

        std::for_each(atts.begin(), atts.end(), [&](auto &kv) {
            if (ignore_atts.contains(kv.first)) s--;
        });

        //std::cout << __FUNCTION__  << " " << s << std::endl;
        serialize_uint64(s, ser);
        for (const auto & [k, v] : atts)
        {
            if (ignore_atts.contains(k)) continue;
            serialize_string(k, ser);
            serialize_att(v, ser);
        }
    }

    void serialize_edge(const DSR::Edge &e, Serializer &ser)
    {
        serialize_uint64(e.to(), ser);
        serialize_uint64(e.from(), ser);
        serialize_uint(e.agent_id(), ser);
        serialize_string(e.type(), ser);
        serialize_atts(e.attrs(), ser);
    }

    void serialize_edges(const std::map<std::pair<uint64_t, std::string>, DSR::Edge > &edges, Serializer &ser)
    {
        //std::cout << __FUNCTION__  << " " << edges.size() << std::endl;

        serialize_uint64(edges.size(), ser);
        for (const auto &[k, v] : edges)
        {
            const auto&[id, type] = k;
            serialize_uint64(id, ser);
            serialize_string(type, ser);
            serialize_edge(v, ser);
        }
    }

    void serialize_node(const DSR::Node& n, Serializer &ser)
    {
        serialize_uint64(n.id(), ser);
        serialize_uint(n.agent_id(), ser);
        serialize_string(n.type(), ser);
        serialize_string(n.name(), ser);
        serialize_atts(n.attrs(), ser);
        serialize_edges(n.fano(), ser);
    }


    void serialize(DSR::DSRGraph *G, Serializer &ser) {

        std::map<uint64_t, DSR::Node> M = G->getCopy();

        //serialize_string(title, ser);
        serialize_uint64(M.size(), ser);
        for (auto &[i, n] : M)
        {
            //std::cout << "Node " << i << std::endl;
            serialize_node(n, ser);
        }

    }


////////////////////////////////
//////// Deserialize
////////////////////////////////

    void deserialize_string(std::string& i, Serializer &ser)
    {
        auto [ptr, s] = ser.deser_att_var_size();
        i = std::string((char *)ptr, (char *)ptr+s);
    }

    void deserialize_int(std::int32_t& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(std::int32_t));
    }

    void deserialize_uint(std::uint32_t& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(std::uint32_t));
    }

    void deserialize_uint64(std::uint64_t& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(std::uint64_t));
    }

    void deserialize_float(float& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(float));
    }

    void deserialize_double(double& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(double));
    }

    void deserialize_bool(bool& i, Serializer &ser)
    {
        ser.deser_att(&i, sizeof(bool));
    }

    void deserialize_vecu8(std::vector<uint8_t>& i, Serializer &ser)
    {
        auto [ptr, s] = ser.deser_att_var_size();
        i = std::vector<uint8_t>((uint8_t *)ptr, (uint8_t *)ptr+s);
    }

    void deserialize_vecfloat(std::vector<float>& i, Serializer &ser)
    {
        auto [ptr, s] = ser.deser_att_var_size();
        i = std::vector<float>((float *)ptr, (float *)ptr+s/sizeof(float));
    }

    void deserialize_vecu64(std::vector<uint64_t>& i, Serializer &ser)
    {
        auto [ptr, s] = ser.deser_att_var_size();
        i = std::vector<uint64_t>((uint64_t *)ptr, (uint64_t *)ptr+s/sizeof(uint64_t));
    }

    void deserialize_vecfloat2(std::array<float, 2>& i, Serializer &ser)
    {
        ser.deser_att(i.data(), sizeof(float)*2);
    }

    void deserialize_vecfloat3(std::array<float, 3>& i, Serializer &ser)
    {
        ser.deser_att(i.data(), sizeof(float)*3);
    }

    void deserialize_vecfloat4(std::array<float, 4>& i, Serializer &ser)
    {
        ser.deser_att(i.data(), sizeof(float)*4);
    }

    void deserialize_vecfloat6(std::array<float, 6>& i, Serializer &ser)
    {
        ser.deser_att(i.data(), sizeof(float)*6);
    }

    void deserialize_att(DSR::Attribute& att, Serializer &ser)
    {
        //std::cout << __FUNCTION__  << std::endl;
        uint64_t time;
        uint32_t aid;

        deserialize_uint64(time, ser);
        deserialize_uint(aid, ser);
        att.agent_id(aid);
        att.timestamp(time);

        uint8_t selected = ser.next_byte();
        switch(selected)
        {
            case DSR::STRING: {
                std::string str;
                deserialize_string(str, ser);
                att.str(std::move(str));
                break;
            }
            case DSR::INT: {
                int32_t dec;
                deserialize_int(dec, ser);
                att.dec(dec);
                break;
            }
            case DSR::FLOAT: {
                float fl;
                deserialize_float(fl, ser);
                att.fl(fl);
                break;
            }
            case DSR::FLOAT_VEC: {
                std::vector<float> fv;
                deserialize_vecfloat(fv, ser);
                att.float_vec(std::move(fv));
                break;
            }
            case DSR::BOOL: {
                bool bl;
                deserialize_bool(bl, ser);
                att.bl(bl);
                break;
            }
            case DSR::BYTE_VEC: {
                std::vector<uint8_t> u8v;
                deserialize_vecu8(u8v, ser);
                att.byte_vec(std::move(u8v));
                break;
            }
            case DSR::UINT: {
                uint32_t uint;
                deserialize_uint(uint, ser);
                att.uint(uint);
                break;
            }
            case DSR::UINT64: {
                uint64_t u64;
                deserialize_uint64(u64, ser);
                att.uint64(u64);
                break;
            }
            case DSR::DOUBLE: {
                double dob;
                deserialize_double(dob, ser);
                att.dob(dob);
                break;
            }
            case DSR::U64_VEC: {
                std::vector<uint64_t> u64v;
                deserialize_vecu64(u64v, ser);
                att.u64_vec(std::move(u64v));
                break;
            }
            case DSR::VEC2: {
                std::array<float, 2> f2v{};
                deserialize_vecfloat2(f2v, ser);
                att.vec2(f2v);
                break;
            }
            case DSR::VEC3: {
                std::array<float, 3> f3v{};
                deserialize_vecfloat3(f3v, ser);
                att.vec3(f3v);
                break;
            }
            case DSR::VEC4: {
                std::array<float, 4> f4v{};
                deserialize_vecfloat4(f4v, ser);
                att.vec4(f4v);
                break;
            }
            case DSR::VEC6: {
                std::array<float, 6> f6v{};
                deserialize_vecfloat6(f6v, ser);
                att.vec6(f6v);
                break;
            }
            default:break;
        }
    }



    void deserialize_atts(std::map<std::string, DSR::Attribute>& atts, Serializer &ser, const std::set<std::string>& ignore_atts={"cam_depth", "cam_rgb", "laser_angles", "laser_dists"})
    {

        uint64_t size;
        deserialize_uint64(size, ser);
        //std::cout << __FUNCTION__  << " " << size <<std::endl;

        for (size_t i = 0; i < size; i++)
        {
            DSR::Attribute att{};
            std::string key;
            deserialize_string(key, ser);
            deserialize_att(att, ser);
            atts.emplace(std::move(key), std::move(att));
        }
    }

    void deserialize_edge(DSR::Edge &e, Serializer &ser)
    {
        //std::cout << __FUNCTION__  << std::endl;
        uint64_t to, from;
        uint32_t agent_id;
        deserialize_uint64(to, ser);
        deserialize_uint64(from, ser);
        deserialize_uint(agent_id, ser);
        e.to(to);
        e.from(from);
        e.agent_id(agent_id);
        deserialize_string(e.type(), ser);
        deserialize_atts(e.attrs(), ser);
    }

    void deserialize_edges(std::map<std::pair<uint64_t, std::string>, DSR::Edge > &edges, Serializer &ser)
    {

        uint64_t size;
        deserialize_uint64(size, ser);
        //std::cout << __FUNCTION__  << " " << size <<std::endl;

        for (size_t i = 0; i < size; i++)
        {
            uint64_t id;
            std::string type;
            DSR::Edge edge;
            deserialize_uint64(id, ser);
            deserialize_string(type, ser);
            deserialize_edge(edge, ser);
            edges.emplace(std::pair<uint64_t, std::string>{id, std::move(type)}, std::move(edge));
        }
    }

    void deserialize_node(DSR::Node& n, Serializer &ser)
    {
        //std::cout << __FUNCTION__  << std::endl;
        uint64_t id;
        uint32_t agent_id;
        deserialize_uint64(id, ser);
        deserialize_uint(agent_id, ser);
        n.id(id);
        n.agent_id(agent_id);
        deserialize_string(n.type(), ser);
        deserialize_string(n.name(), ser);
        deserialize_atts(n.attrs(), ser);
        deserialize_edges(n.fano(), ser);
    }


    std::map<uint64_t, DSR::Node> deserialize(Serializer &ser) {

        std::map<uint64_t, DSR::Node> M;
        uint64_t m_size;
        deserialize_uint64(m_size, ser);
        for (size_t i = 0; i < m_size; i++)
        {
            DSR::Node n;
            deserialize_node(n, ser);
            M.emplace(n.id(), std::move(n));
        }

        return M;
    }

}


//////////////////////////
/// Serializer impl
/////////////////////////
void Serializer::ser_att(const void *src, size_t s, uint8_t type)
{
    if (p + s + 1 > size) (size+s > size*2) ? reserve(size+static_cast<size_t>((double)s*1.2)) : reserve(size*2);
    ptr[p++] = type;
    std::memcpy(ptr+p, src, s);
    p+=s;
}

void Serializer::ser_att_var_size(const void *src, size_t s, uint8_t type)
{
    if (p + s + 1 + sizeof(size_t)> size) (size+s > size*2) ? reserve(size+static_cast<size_t>((double)s*1.2)) : reserve(size*2);
    ptr[p++] = type;
    std::memcpy(ptr+p, &s, sizeof(size_t));
    p+=sizeof(size_t);
    std::memcpy(ptr+p, src, s);
    p+=s;
}

uint8_t Serializer::next_byte() const
{
    return ptr[read];
}

uint8_t Serializer::next_byte_and_advance()
{
    return ptr[read++];
}

void Serializer::next_byte(uint8_t val)
{
    ptr[p++] = val;
}

void Serializer::deser_att(void *dst, size_t s)
{
    read++; //Skip type byte;
    //std::cout << "DESER ATT: start: " << read-1<< " data at: "<< read << " ( "<< s <<"B ) ends: " << read+s<< std::endl;
    std::memcpy(dst,  ptr + read, s);
    read+=s;
}

std::pair<unsigned char*, size_t> Serializer::deser_att_var_size()
{
    read++; //Skip type byte;
    //std::cout << "DESER ATT VAR SIZE: start: " << read-1 << " size at: " << read;
    size_t s;
    std::memcpy(&s, ptr + read, sizeof(size_t)); //Get size of variable size attribute
    read+=sizeof(size_t);
    //std::cout << " data at: " <<  read << " (" << s << ") ends: " << read+s << std::endl;
    unsigned char *slice = ptr+read;
    read+=s;
    return {slice, s};
}

void Serializer::shrink_to_fit()
{
    ptr = static_cast<unsigned char *>(std::realloc(ptr, p));
    size = p;
}

void Serializer::reserve(size_t s)
{
    //std::cout << "Resizing buffer to size: " << s << std::endl;
    ptr = static_cast<unsigned char *>(std::realloc(ptr, s));
    size = s;
}

//////////////////////////
/// ChangeInfo impl
/////////////////////////

void ChangeInfo::serialize(Serializer & ser) const
{
    ser.next_byte(op);
    serialize_uint(agent_id, ser);
    serialize_uint64(node_or_from_id, ser);
    serialize_uint64(maybe_to_id, ser);
    serialize_string(maybe_edge_type, ser);
    serialize_uint64(timestamp, ser);
}

void ChangeInfo::deserialize(Serializer & ser)
{
    op = static_cast<OPER>(ser.next_byte_and_advance());
    deserialize_uint(agent_id, ser);
    deserialize_uint64(node_or_from_id, ser);
    deserialize_uint64(maybe_to_id, ser);
    deserialize_string(maybe_edge_type, ser);
    deserialize_uint64(timestamp, ser);
}


//////////////////////////
/// GSerializer impl
/////////////////////////

GSerializer::GSerializer(DSR::DSRGraph* G_, std::string save_file)
    : G(G_), total_size(0), used_size(0), out_file(std::move(save_file))
{}

GSerializer::~GSerializer()
{
    if (!out_file.empty()) save_file(out_file);
    //Manually deallocate buffers.
    for (auto [ptr, _] : ops) std::free(ptr);
}


void GSerializer::initialize()
{
    QObject::connect(G, &DSR::DSRGraph::update_node_signal,  [this](auto node, auto type) {

        std::optional<DSR::Node> e = G->get_node(node);
        uint32_t agent_id = 0;
        if (e.has_value()) {
            agent_id = (*e).agent_id();
            auto el_it = std::max_element((*e).attrs().begin(), (*e).attrs().end(), [](auto &e1, auto &e2) {
                return e1.second.timestamp() < e2.second.timestamp();
            });
            if (el_it != (*e).attrs().end())
                agent_id = (*el_it).second.agent_id();
        }

        ChangeInfo c = {
                .op = (ops.empty()) ? ChangeInfo::COMPLETE : ChangeInfo::NODE_CHANGE,
                .agent_id = agent_id, //This may be not accurate if a change has been processed between the signal sending and the execution of this signal.
                .node_or_from_id= node,
                .maybe_to_id=0,
                .timestamp=get_unix_timestamp(),
                .maybe_edge_type="",
        };

        add_change(std::move(c));

    });
    QObject::connect(G, &DSR::DSRGraph::update_edge_signal, [this](auto from, auto to, auto type) {

        std::optional<DSR::Edge> e = G->get_edge(from, to, type);
        uint32_t agent_id = 0;
        if (e.has_value()) {
            agent_id = (*e).agent_id();
            auto el_it = std::max_element((*e).attrs().begin(), (*e).attrs().end(), [](auto &e1, auto &e2) {
                return e1.second.timestamp() < e2.second.timestamp();
            });
            if (el_it != (*e).attrs().end())
                agent_id = (*el_it).second.agent_id();
        }

        ChangeInfo c = {
                .op = (ops.empty()) ? ChangeInfo::COMPLETE : ChangeInfo::EDGE_CHANGE,
                .agent_id = agent_id, //This may be not accurate if a change has been processed between the signal sending and the execution of this signal.
                .node_or_from_id= from,
                .maybe_to_id=to,
                .timestamp=get_unix_timestamp(),
                .maybe_edge_type=type,
        };

        add_change(std::move(c));

    });

    QObject::connect(G, &DSR::DSRGraph::del_edge_signal, [this](auto from, auto to, auto type) {
        ChangeInfo c = {
                .op = (ops.empty()) ? ChangeInfo::COMPLETE : ChangeInfo::EDGE_DEL,
                .agent_id = 0, //We don't have a way to know who made this change.
                .node_or_from_id= from,
                .maybe_to_id=to,
                .timestamp=get_unix_timestamp(),
                .maybe_edge_type=type,
        };

        add_change(std::move(c));
    });
    QObject::connect(G, &DSR::DSRGraph::del_node_signal, [this](auto node) {
        ChangeInfo c = {
                .op = (ops.empty()) ? ChangeInfo::COMPLETE : ChangeInfo::NODE_DEL,
                .agent_id = 0, //We don't have a way to know who made this change.
                .node_or_from_id= node,
                .maybe_to_id=0,
                .timestamp=get_unix_timestamp(),
                .maybe_edge_type="",
        };

        add_change(std::move(c));

    });
}


void GSerializer::save_file(const std::string& name)
{
    std::ofstream wbf(name, std::ios::out | std::ios::binary);

    if(!wbf) {
        std::cout << "Error cannot open file" << std::endl;
        return;
    }

    for (auto [ptr, size] : ops) {
        //std::cout << "Write size: "<< size << " ("<< sizeof(size_t) << ") bytes" << std::endl;
        wbf.write((char*)&size, sizeof(size_t));
        //std::cout << "Write ptr: ("<< size << ") bytes" << std::endl;
        wbf.write((char*)ptr, static_cast<std::streamsize>(size));
    }
    wbf.flush();
    wbf.close();
}

std::vector<std::pair<ChangeInfo, std::variant<std::monostate, DSR::Node, DSR::Edge, std::map<uint64_t, DSR::Node>>>> GSerializer::read_file(const std::string& name)
{
    std::vector<std::pair<ChangeInfo, std::variant<std::monostate, DSR::Node, DSR::Edge, std::map<uint64_t, DSR::Node>>>> res;

    std::ifstream rbf(name, std::ios::in | std::ios::binary);

    if(!rbf) {
        std::cout << "Error cannot open file" << std::endl;
        return res;
    }

    rbf.ignore( std::numeric_limits<std::streamsize>::max() );
    std::streamsize length = rbf.gcount();
    rbf.clear();
    rbf.seekg( 0, std::ios_base::beg );

    //auto start_2 = std::chrono::steady_clock::now();

    while(rbf.tellg() < length)
    {
        std::size_t size;
        rbf.read((char *)&size, sizeof(size_t));
        auto *ptr = static_cast<unsigned char *>(malloc(size));
        rbf.read((char*)ptr, static_cast<std::streamsize>(size));

        ChangeInfo ci;
        Serializer ser = { .ptr = ptr, .size = size, .p = size, .read= 0};
        ci.deserialize(ser);

        switch (ci.op)
        {
            case ChangeInfo::COMPLETE:
            {
                std::map<uint64_t, DSR::Node> map = deserialize(ser);
                res.emplace_back(ci, std::move(map));
                break;
            }
            case ChangeInfo::NODE_CHANGE:
            {
                DSR::Node n;
                deserialize_node(n, ser);
                res.emplace_back(ci, std::move(n));
                break;
            }
            case ChangeInfo::EDGE_CHANGE:
            {
                DSR::Edge e;
                deserialize_edge(e, ser);
                res.emplace_back(ci, std::move(e));
                break;
            }
            case ChangeInfo::NODE_DEL:
            case ChangeInfo::EDGE_DEL:
            {
                res.emplace_back(ci, std::monostate{});
                break;
            }
        }
        std::free(ptr);
    }

    //auto end_2 = std::chrono::steady_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count();
    //std::cout << "Deserializing all changes took: " << duration << " us" << std::endl;

    return res;
}

void GSerializer::add_change(ChangeInfo && c)
{
    //auto start_2 = std::chrono::steady_clock::now();

    Serializer ser = { .ptr = static_cast<unsigned char *>(std::malloc(1000)) };
    ChangeInfo tmp;
    c.serialize(ser);
    switch (c.op)
    {
        case ChangeInfo::COMPLETE:
        {
            serialize(G, ser);
            break;
        }
        case ChangeInfo::NODE_CHANGE:
        {
            auto n = G->get_node(c.node_or_from_id);
            if (n.has_value())
                serialize_node(*n, ser);
            break;
        }
        case ChangeInfo::EDGE_CHANGE:
        {
            auto e = G->get_edge(c.node_or_from_id, c.maybe_to_id, c.maybe_edge_type);
            if (e.has_value())
                serialize_edge(*e, ser);
            break;
        }
        case ChangeInfo::NODE_DEL:
        case ChangeInfo::EDGE_DEL:
        {
            break;
        }
    }

    ser.shrink_to_fit();
    //auto end_2 = std::chrono::steady_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_2 - start_2).count();

    ops.emplace_back(ser.ptr, ser.p);

    //std::cout << "Serializing change took: " << duration << " us" << std::endl;

    total_size+=ser.size;
    used_size+=ser.p;

}