//
// Created by juancarlos on 18/8/20.
//

#ifndef TRAITS_H
#define TRAITS_H

#include <type_traits>


template<typename Va>
static bool constexpr allowed_types =   std::is_same<std::int32_t, Va>::value ||
                                        std::is_same<std::uint32_t, Va>::value ||
                                        std::is_same<std::string, Va>::value ||
                                        std::is_same<std::reference_wrapper<const std::string>, Va>::value ||
                                        std::is_same<std::reference_wrapper<const std::vector<float_t>>, Va>::value ||
                                        std::is_same<std::reference_wrapper<const std::vector<uint8_t>>, Va>::value ||
                                        std::is_same<std::float_t, Va>::value ||
                                        std::is_same<std::double_t, Va>::value ||
                                        std::is_same<std::vector<float_t>, Va>::value ||
                                        std::is_same<std::vector<uint8_t>, Va>::value ||
                                        std::is_same<bool, Va>::value;
template<typename Va>
static bool constexpr any_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                                         std::is_same<DSR::CRDTEdge, Va>::value ||
                                         std::is_same<DSR::Node, Va>::value ||
                                         std::is_same<DSR::Edge, Va>::value
;

template<typename Va>
static bool constexpr node_or_edge = std::is_same<DSR::Node, Va>::value ||
                                     std::is_same<DSR::Edge, Va>::value
;


template<typename Va>
static bool constexpr crdt_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                                          std::is_same<DSR::CRDTNode, Va>::value
;

template<typename Va>
static bool constexpr allowed_return_types = std::is_same<std::int32_t, Va>::value ||
                                             std::is_same<std::uint32_t, Va>::value ||
                                             std::is_same<std::string, Va>::value ||
                                             std::is_same<std::float_t, Va>::value ||
                                             std::is_same<std::vector<float_t>, Va>::value ||
                                             std::is_same<std::vector<uint8_t>, Va>::value ||
                                             std::is_same<bool, Va>::value ||
                                             std::is_same<QVec, Va>::value ||
                                             std::is_same<QMat, Va>::value;



//Comprueba si en el tipo T existen los attributos attr_type y attr_name
template <typename, typename = void, typename = void>
struct is_attr_name : std::false_type {};
template <typename T>
struct is_attr_name<T, std::void_t<decltype(T::attr_type), decltype(T::attr_name)>, typename std::enable_if<T::attr_type >::type > : std::true_type {};


template<typename T>
struct is_reference_wrapper : false_type {};
template<typename T>
struct is_reference_wrapper<reference_wrapper<T>> : true_type{};



#endif //TRAITS_H
