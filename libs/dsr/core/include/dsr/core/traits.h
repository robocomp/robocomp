//
// Created by juancarlos on 18/8/20.
//

#ifndef TRAITS_H
#define TRAITS_H

#include <type_traits>

namespace DSR {
    class Node;
    class Edge;
    class CRDTNode;
    class CRDTEdge;
};

template<typename Va>
concept allowed_types = std::is_same<std::int32_t, Va>::value ||
                        std::is_same<std::uint32_t, Va>::value ||
                        std::is_same<std::uint64_t, Va>::value ||
                        std::is_same<std::string, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::string>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::vector<float_t>>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::vector<uint8_t>>, Va>::value ||
                        std::is_same<std::float_t, Va>::value ||
                        std::is_same<std::double_t, Va>::value ||
                        std::is_same<std::vector<float_t>, Va>::value ||
                        std::is_same<std::vector<uint8_t>, Va>::value ||
                        std::is_same<bool, Va>::value ||
                        std::is_same<std::vector<uint64_t>, Va>::value ||
                        std::is_same<std::array<float, 2>, Va>::value ||
                        std::is_same<std::array<float, 3>, Va>::value ||
                        std::is_same<std::array<float, 4>, Va>::value ||
                        std::is_same<std::array<float, 6>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::vector<uint64_t>>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::array<float, 2>>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::array<float, 3>>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::array<float, 4>>, Va>::value ||
                        std::is_same<std::reference_wrapper<const std::array<float, 6>>, Va>::value
;


template<typename Va>
concept any_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                           std::is_same<DSR::CRDTEdge, Va>::value ||
                           std::is_same<DSR::Node, Va>::value ||
                           std::is_same<DSR::Edge, Va>::value;;

template<typename Va>
concept node_or_edge = std::is_same<DSR::Node, Va>::value ||
                       std::is_same<DSR::Edge, Va>::value;;


template<typename Va>
concept crdt_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                            std::is_same<DSR::CRDTNode, Va>::value;;



//Comprueba si en el tipo T existen los attributos attr_type y attr_name
template<typename, typename = void, typename = void>
struct is_attr_name : std::false_type {};
template<typename T>
struct is_attr_name<T, std::void_t<decltype(T::attr_type), decltype(T::attr_name)>, typename std::enable_if<T::attr_type>::type>
        : std::true_type
{
};


template<typename T>
struct is_reference_wrapper : std::false_type {};
template<typename T>
struct is_reference_wrapper<std::reference_wrapper<T>> : std::true_type {};


#endif //TRAITS_H
