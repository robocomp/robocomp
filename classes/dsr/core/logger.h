#ifndef DSR_LOGGER
#define DSR_LOGGER

#include <string>
#include <iostream>

#include "threadpool/threadpool.h"

#include <bsoncxx/json.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/document/value.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/client.hpp>


using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;

/*
const std::set<std::string_view> type_entry = {"INSERT_NODE"sv,
                                               "INSERT_EDGE"sv, 
                                               "REMOVE_NODE"sv, 
                                               "REMOVE_EDGE"sv, 
                                               "UPDATE_ATTRIBUTE_NODE"sv, 
                                               "UPDATE_ATTRIBIBUTE_EDGE"sv};
*/
class Logger
{
public:
    static Logger *get_logger()
    {
        std::call_once(init_flag, [&]() { instance_logger = new Logger(); });
        return instance_logger;
    }

    void Log(uint32_t agent_id,
             uint64_t timestamp,
             std::string && type,
             uint32_t created_by,
             bool recv, 
             std::unordered_map<std::string, std::string>&& extra)
    {

        worker.spawn_task([this, agent_id, timestamp, type = std::move(type), created_by, recv, extra = std::move(extra)](){

            //std::cout << type << std::endl;
            try {

                auto builder = bsoncxx::builder::stream::document{};
                builder
                        << "agent_id" << (int) agent_id
                        << "type" << type
                        << "created_by" << (int) created_by
                        << "recv" << recv
                        << "extra_info" << open_document;


                if (type == "INSERT_NODE" || type == "REMOVE_NODE") {
                    builder << "node_id" << std::stoi(extra.at("node_id"))
                            << "node_type" << extra.at("node_type");
                } else if (type == "INSERT_EDGE" || type == "REMOVE_EDGE") {
                    builder << "from_id" << std::stoi(extra.at("from_id"))
                            << "to_id" << std::stoi(extra.at("to_id"))
                            << "edge_type" << extra.at("edge_type");
                } else if (type == "UPDATE_ATTRIBUTE_NODE") {
                    builder << "node_id" << std::stoi(extra.at("node_id"))
                            << "node_type" << extra.at("node_type")
                            << "attribute" << extra.at("attribute")
                            << "operation" << extra.at("operation");
                } else if (type == "UPDATE_ATTRIBIBUTE_EDGE") {
                    builder << "from_id" << std::stoi(extra.at("from_id").data())
                            << "to_id" << std::stoi(extra.at("to_id").data())
                            << "attribute" << extra.at("attribute")
                            << "operation" << extra.at("operation")
                            << "edge_type" << extra.at("edge_type");
                } else if (type == "FULL_GRAPH") {
                    return; //De momento no guardar este tipo de mensaje
                    //builder << "from_id" << std::stoi(extra.at("size").data());
                }

                //Revisar esto

                builder << "pair_first" << std::stoi(extra.at("pair_first"))
                            << "pair_second" << std::stoi(extra.at("pair_second"));

                builder << close_document;

                if (recv) {
                    builder << "received_timestamp" <<  static_cast<int64_t>(timestamp);
                    auto coll = client["tests"]["table_recv_" + std::to_string(agent_id)];
                    auto coll2 =  client["tests"]["table_created_" + std::to_string(created_by)];
                    if (type == "INSERT_NODE" || type == "REMOVE_NODE") {
                        auto cursor =
                                coll2.find(
                                        document{} << "extra_info.node_id" << std::stoi(extra.at("node_id"))
                                                   //<< "extra_info.node_type" << extra.at("node_type")
                                                   << "extra_info.pair_second" << std::stoi(extra.at("pair_second"))
                                                   << "type" << type
                                                   << finalize);
                        for (auto &&doc : cursor) {
                            auto created_at = (uint64_t) doc.find("created_timestamp")->get_int64();
                            builder<< "lat_ns" <<static_cast<int64_t>(timestamp - created_at);
                            //builder << bsoncxx::to_json(*result);
                        } /*else {
                            puts( "no encontrado");
                        }*/
                    } else if (type == "INSERT_EDGE" || type == "REMOVE_EDGE") {
                        auto cursor  =
                                coll2.find(
                                document{} << "extra_info.from_id" << std::stoi(extra.at("from_id"))
                                           << "extra_info.to_id" << std::stoi(extra.at("to_id"))
                                           << "extra_info.edge_type" << extra.at("edge_type")
                                           << "extra_info.pair_second" << std::stoi(extra.at("pair_second"))
                                           << "type" << type
                                           << finalize);

                        for (auto &&doc : cursor) {
                            auto created_at = (uint64_t) doc.find("created_timestamp")->get_int64();
                            builder<< "lat_ns" <<static_cast<int64_t>(timestamp - created_at);
                        } /*else {
                            puts( "no encontrado");
                        }*/
                    } else if (type == "UPDATE_ATTRIBUTE_NODE") {
                        auto cursor =
                                coll2.find(
                                        document{} << "extra_info.node_id" << std::stoi(extra.at("node_id"))
                                                   //<< "extra_info.node_type" << extra.at("node_type")
                                                   << "extra_info.attribute" << extra.at("attribute")
                                                   //<< "extra_info.operation" << extra.at("operation")
                                                   << "type" << type
                                                   <<"extra_info.pair_second" << std::stoi(extra.at("pair_second"))
                                                   << finalize);
                        for (auto &&doc : cursor) {
                            auto created_at = (uint64_t) doc.find("created_timestamp")->get_int64();
                            builder<< "lat_ns" <<static_cast<int64_t>(timestamp - created_at);
                        }/*else {
                            puts( "no encontrado");
                        }*/
                    } else if (type == "UPDATE_ATTRIBIBUTE_EDGE") {
                        auto cursor =
                                coll2.find(
                                        document{} << "extra_info.from_id" << std::stoi(extra.at("from_id").data())
                                                   << "extra_info.to_id" << std::stoi(extra.at("to_id").data())
                                                   << "extra_info.attribute" << extra.at("attribute")
                                                   //<< "extra_info.operation" << extra.at("operation")
                                                   << "extra_info.edge_type" << extra.at("edge_type")
                                                   << "type" << type
                                                   <<"extra_info.pair_second" << std::stoi(extra.at("pair_second"))
                                                   << finalize);
                        for (auto &&doc : cursor) {
                            auto created_at = (uint64_t) doc.find("created_timestamp")->get_int64();
                            builder<< "lat_ns" <<static_cast<int64_t>(timestamp - created_at);
                        }/*else {
                            puts( "no encontrado");
                        }*/
                    } /*else if (type == "FULL_GRAPH") {

                    }*/


                    bsoncxx::view_or_value<bsoncxx::document::view, bsoncxx::document::value> document =
                            builder << finalize;
                    coll.insert_one(document);
                } else {

                    builder << "created_timestamp" << static_cast<int64_t>(std::stoll(extra.at("timestamp")));
                    auto coll = client["tests"]["table_created_" + std::to_string(agent_id)];
                    bsoncxx::view_or_value<bsoncxx::document::view, bsoncxx::document::value> document =
                            builder << finalize;
                    coll.insert_one(document);
                }


            } catch(std::exception &e) {
                std::cout << e.what() << std::endl;
            }

        });

    }

private:
    Logger() : worker(1), client{mongocxx::uri{"mongodb://localhost:27017"}}
    {
    }

    ThreadPool worker;
    mongocxx::instance instance; // This should be done only once.
    mongocxx::client client;
    static std::once_flag init_flag;
    static Logger *instance_logger;
};


inline std::once_flag Logger::init_flag;
inline Logger * Logger::instance_logger;

#endif
