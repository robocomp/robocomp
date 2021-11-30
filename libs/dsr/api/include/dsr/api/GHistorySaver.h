//
// Created by juancarlos on 30/11/21.
//

#ifndef DSR_GHISTORY_H
#define DSR_GHISTORY_H

#include <dsr/api/dsr_api.h>
#include <cstdio>


//The Serializer class writes or reads the information of Nodes, Edges and Attributes to a buffer.
//All the information stored as attributes. These can be fixed size attributes (int, bool, float, etc.) or
//variable sized attributes (string, vector<float>, etc.). In the first case the attribute is encoded with 1 byte
//for the type and sizeof(T) for the data. For the second case, 1 byte for the type, 8 bytes for the size
//and size bytes for the data.
//The memory writes and reads are unaligned.
//The buffer is not handled by this object, but from the GSerializer class.
struct Serializer
{
    unsigned char *ptr = nullptr;
    size_t size = 1000;
    size_t p = 0;
    size_t read = 0;

    ~Serializer()= default;

    void ser_att(const void *src, size_t s, uint8_t type);
    void ser_att_var_size(const void *src, size_t s, uint8_t type);

    [[nodiscard]] uint8_t next_byte() const;
    uint8_t next_byte_and_advance();
    void next_byte(uint8_t val);

    void deser_att(void *dst, size_t s);
    std::pair<unsigned char*, size_t> deser_att_var_size();

    void shrink_to_fit();
private:

    void reserve(size_t s);
};


//Contains the info of a DSR change received in a Qt signal at a Node and Edge level.
//This data is included in the serialized file created after the execution.
//Some of the information may be wrong if changes for the same data
//have arrived between the emission of the signal and the execution of the Serializer.
struct ChangeInfo {

    enum OPER : uint8_t {
        COMPLETE = 0,
        NODE_CHANGE = 1,
        EDGE_CHANGE = 2,
        NODE_DEL = 3,
        EDGE_DEL = 4
    } op;

    uint32_t agent_id;
    uint64_t node_or_from_id;
    uint64_t maybe_to_id;
    std::uint64_t timestamp;
    std::string maybe_edge_type;

    void serialize(Serializer & ser) const;
    void deserialize(Serializer & ser);

};


//This class receive the signals from DSR and saves the changes in nodes and edges.
//After the execution the changes are written to a binary file with the name of the out_file variable.
//The first entry of the file contains a ChangeInfo object and all the nodes and edges in the graph. from there,
//the content of the entries are a ChangeInfo object and either a Node, an Edge or nothing if the operation
//was deletion.
//The file can be loaded with the read_file method.
class GSerializer {

    std::vector<std::pair<unsigned char *, size_t>> ops;
    DSR::DSRGraph* G;
    size_t total_size;
    size_t used_size;
    std::string out_file;


    void add_change(ChangeInfo && c);

public:
    explicit GSerializer(DSR::DSRGraph* G_, std::string save_file = "");
    ~GSerializer();

    void initialize();

    void save_file(const std::string& name);
    static std::vector<std::pair<ChangeInfo, std::variant<std::monostate, DSR::Node, DSR::Edge, std::map<uint64_t, DSR::Node>>>>
        read_file(const std::string& name);
};




#endif //DSR_GHISTORY_H
