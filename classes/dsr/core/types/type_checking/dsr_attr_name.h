//
// Created by juancarlos on 31/7/20.
//

#ifndef DSR_ATTR_NAME_H
#define DSR_ATTR_NAME_H


#include <typeindex>
#include <tuple>
#include <cstdint>
#include <string>
#include <vector>
#include <type_traits>
#include <functional>
#include <unordered_map>
#include <any>
#include <cmath>
#include <memory>
#include "../../traits.h"
#include "../crdt_types.h"
#include "../user_types.h"
#include "type_checker.h"
#include <qmat/QMatAll>


// Attributes
//Define el tipo utilizado para validar los tipos de atributos durante la compilación
template<const std::string_view& n, typename Tn>
struct Attr {
    static constexpr bool attr_type = bool_constant<allowed_types<Tn>>();
    static constexpr std::string_view attr_name = std::string_view(n);
    static Tn type;
};

template<typename name, class Ta>
static constexpr bool valid_type ()
{
    if constexpr(is_reference_wrapper<decltype(name::type)>::value) {
        using ref_type = typename decltype(name::type)::type;
        using Selected_Type = std::remove_reference_t<std::remove_cv_t<ref_type>>;
        return std::is_same_v<Selected_Type, std::remove_cv_t<std::remove_reference_t<Ta>>>;
    } else {
        using Selected_Type = std::remove_reference_t<std::remove_cv_t<decltype(name::type)>>;
        return std::is_same_v<Selected_Type, std::remove_cv_t<std::remove_reference_t<Ta>>>;
    }
}

template<typename tp>
static constexpr auto reg_fn = []() -> auto
            {
                if constexpr (is_reference_wrapper<tp>::value) {
                    using tp_c = std::remove_const_t<typename tp::type>;
                    return tp_c();
                } else {
                    static_assert(std::is_constructible_v<tp>, "tp is not constructible without arguments, register your type mmanually");
                    return tp();
                }
            };

#define REGISTER_FN(x, it)  \
                            [[maybe_unused]] inline bool x ##_b =  ATTRIBUTE_TYPES::REGISTER( x##_str, reg_fn<it>() );     \
                            \


#define REGISTER_TYPE(x, ot) \
                            static constexpr auto    x ##_str = std::string_view(#x ); \
                            using x = Attr< x##_str, ot>;                        \
                            typedef x  x ##_att;                                          \
                            REGISTER_FN(x, ot) \
                            \



inline std::unordered_map<std::string_view, std::function<bool(const std::any&)>> ATTRIBUTE_TYPES::map_fn_;

REGISTER_TYPE(pos_x, float)
REGISTER_TYPE(pos_y, float)
REGISTER_TYPE(level, int)
REGISTER_TYPE(name, std::reference_wrapper<const std::string>)
REGISTER_TYPE(parent, std::uint32_t)
REGISTER_TYPE(rotation_euler_xyz, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(translation, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(color, std::reference_wrapper<const std::string>)
REGISTER_TYPE(texture, std::reference_wrapper<const std::string>)
REGISTER_TYPE(width, int)
REGISTER_TYPE(height, int)
REGISTER_TYPE(scalex, int)
REGISTER_TYPE(scaley, int)
REGISTER_TYPE(scalez, int)
REGISTER_TYPE(path, std::reference_wrapper<const std::string>)
REGISTER_TYPE(angles, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(dists, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(rgb, std::reference_wrapper<const std::vector<uint8_t>>)
REGISTER_TYPE(cameraID, int)
REGISTER_TYPE(focalx, int)
REGISTER_TYPE(focaly, int)
REGISTER_TYPE(alivetime, int)
REGISTER_TYPE(linear_speed, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(angular_speed, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(ref_adv_speed, float)
REGISTER_TYPE(ref_rot_speed, float)
REGISTER_TYPE(ref_side_speed, float)
REGISTER_TYPE(base_target_x, float)
REGISTER_TYPE(base_target_y, float)
REGISTER_TYPE(social_x_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(social_y_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(personal_x_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(personal_y_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(sharedWidth, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(intimate_x_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(intimate_y_pos, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(rgb_cameraID, int)
REGISTER_TYPE(rgb_focalx, int)
REGISTER_TYPE(rgb_focaly, int)
REGISTER_TYPE(rgb_alivetime, int)
REGISTER_TYPE(rgb_width, int)
REGISTER_TYPE(rgb_height, int)
REGISTER_TYPE(rgb_depth, int)
REGISTER_TYPE(depth, int)
REGISTER_TYPE(img_depth, std::reference_wrapper<const std::vector<uint8_t>>)
REGISTER_TYPE(depth_cameraID, int)
REGISTER_TYPE(depthFactor, float)
REGISTER_TYPE(depth_height, int)
REGISTER_TYPE(depth_width, int)
REGISTER_TYPE(OuterRegionLeft, int)
REGISTER_TYPE(OuterRegionRight, int)
REGISTER_TYPE(OuterRegionBottom, int)
REGISTER_TYPE(OuterRegionTop, int)
REGISTER_TYPE(target_node_id, int)
REGISTER_TYPE(viriato_pan_tilt_nose_target, std::reference_wrapper<const std::vector<float>>)
REGISTER_TYPE(mass, int)

#endif //DSR_ATTR_NAME_H
