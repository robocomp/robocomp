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
#include <dsr/core/traits.h>
#include <dsr/core/types/crdt_types.h>
#include <dsr/core/types/user_types.h>
#include "type_checker.h"
#include <qmat/QMatAll>


// Attributes
//Define el tipo utilizado para validar los tipos de atributos durante la compilación
template<const std::string_view& n, typename Tn>
struct Attr {
    static constexpr bool attr_type = std::bool_constant<allowed_types<Tn>>();
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
                    static_assert(std::is_constructible_v<tp>, "tp is not constructible without arguments, register your type manually");
                    return tp();
                }
            };




#define REGISTER_FN(x, it, stream)  \
                            [[maybe_unused]] inline bool x ##_b =  attribute_types::register_type( x##_str, reg_fn<it>(), stream);     \
                            \


#define REGISTER_TYPE(x, ot, stream) \
                            static constexpr auto    x ##_str = std::string_view(#x ); \
                            using x##_att = Attr< x##_str, ot>;                        \
                            REGISTER_FN(x, ot, stream) \
                            \



inline std::unordered_map<std::string_view, std::function<bool(const std::any&)>> attribute_types::map_fn_;

/*
 * Generic
 * */
REGISTER_TYPE(level, int, false)
REGISTER_TYPE(pos_x, float, false)
REGISTER_TYPE(pos_y, float, false)
REGISTER_TYPE(parent, std::uint64_t, false)
REGISTER_TYPE(color, std::reference_wrapper<const std::string>, false)
REGISTER_TYPE(texture, std::reference_wrapper<const std::string>, false)
REGISTER_TYPE(width, int, false)
REGISTER_TYPE(height, int, false)
REGISTER_TYPE(depth, int, false)
REGISTER_TYPE(mass, int, false)
REGISTER_TYPE(scalex, int, false)
REGISTER_TYPE(scaley, int, false)
REGISTER_TYPE(scalez, int, false)
REGISTER_TYPE(path, std::reference_wrapper<const std::string>, false)
REGISTER_TYPE(name, std::reference_wrapper<const std::string>, false)

/*
 * RT
 * */
REGISTER_TYPE(rt_rotation_euler_xyz, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(rt_translation, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(rt_quaternion, std::reference_wrapper<const std::vector<float>>, true)
/*
 * looking-at
 * */
REGISTER_TYPE(looking_at_rotation_euler_xyz, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(looking_at_translation, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(looking_at_quaternion, std::reference_wrapper<const std::vector<float>>, true)
/*
 * Laser
 * */
REGISTER_TYPE(laser_angles, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(laser_dists, std::reference_wrapper<const std::vector<float>>, true)
/*
 * Person
 * */
REGISTER_TYPE(person_social_x_pos, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(person_social_y_pos, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(person_personal_x_pos, std::reference_wrapper<const std::vector<float>>,true )
REGISTER_TYPE(person_personal_y_pos, std::reference_wrapper<const std::vector<float>>,true)
REGISTER_TYPE(person_sharedWidth, std::reference_wrapper<const std::vector<float>>,true)
REGISTER_TYPE(person_intimate_x_pos, std::reference_wrapper<const std::vector<float>>,true)
REGISTER_TYPE(person_intimate_y_pos, std::reference_wrapper<const std::vector<float>>,true)

/*
 * Personal Space
 * */
REGISTER_TYPE(ps_social_x_pos, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(ps_social_y_pos, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(ps_personal_x_pos, std::reference_wrapper<const std::vector<float>>,true )
REGISTER_TYPE(ps_personal_y_pos, std::reference_wrapper<const std::vector<float>>,true)
REGISTER_TYPE(ps_intimate_x_pos, std::reference_wrapper<const std::vector<float>>,true)
REGISTER_TYPE(ps_intimate_y_pos, std::reference_wrapper<const std::vector<float>>,true)

/*
 * Object
 * */
REGISTER_TYPE(obj_width, int, false)
REGISTER_TYPE(obj_height, int, false)
REGISTER_TYPE(obj_depth, int, false)
/*
 * camera
 * */
REGISTER_TYPE(cam_rgb, std::reference_wrapper<const std::vector<uint8_t>>, true)
REGISTER_TYPE(cam_depth_focalx, int, false)
REGISTER_TYPE(cam_depth_focaly, int, false)
REGISTER_TYPE(cam_depth_alivetime, int, false)
REGISTER_TYPE(cam_rgb_cameraID, int, false)
REGISTER_TYPE(cam_rgb_focalx, int, false)
REGISTER_TYPE(cam_rgb_focaly, int, false)
REGISTER_TYPE(cam_rgb_alivetime, int, true)
REGISTER_TYPE(cam_rgb_width, int, false)
REGISTER_TYPE(cam_rgb_height, int, false)
REGISTER_TYPE(cam_rgb_depth, int, true)
REGISTER_TYPE(cam_depth, std::reference_wrapper<const std::vector<uint8_t>>, true)
REGISTER_TYPE(cam_depth_cameraID, int, false)
REGISTER_TYPE(cam_depthFactor, float, false)
REGISTER_TYPE(cam_depth_height, int, false)
REGISTER_TYPE(cam_depth_width, int, false)

/*
 * Robot
 * */

REGISTER_TYPE(viriato_head_pan_tilt_nose_target, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(robot_current_advance_speed, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(robot_current_angular_speed, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(robot_current_side_speed, std::reference_wrapper<const std::vector<float>>, true)
REGISTER_TYPE(robot_current_linear_speed, std::reference_wrapper<const std::vector<float>>, true) //deprecated
REGISTER_TYPE(robot_ref_adv_speed, float, true)
REGISTER_TYPE(robot_ref_rot_speed, float, true)
REGISTER_TYPE(robot_ref_side_speed, float, true)
REGISTER_TYPE(robot_target_x, float, true)
REGISTER_TYPE(robot_target_y, float, true)
REGISTER_TYPE(robot_target_angle, float, true)

/*
 * Arm
 * */
REGISTER_TYPE(viriato_arm_tip_target, std::reference_wrapper<const std::vector<float>>,  false)
/*
 * Plan
 * */
REGISTER_TYPE(plan, std::reference_wrapper<const std::string>, false)
REGISTER_TYPE(plan_target_node_id, int, false)

/*
 * World
 * */
REGISTER_TYPE(OuterRegionLeft, int, false)
REGISTER_TYPE(OuterRegionRight, int, false)
REGISTER_TYPE(OuterRegionBottom, int, false)
REGISTER_TYPE(OuterRegionTop, int, false)
REGISTER_TYPE(world_outline_x, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(world_outline_y, std::reference_wrapper<const std::vector<float>>, false)

/*
 * Path to target
 * */
REGISTER_TYPE(path_x_values, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(path_y_values, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(path_target_x, float, false)
REGISTER_TYPE(path_target_y, float, false)

/*
 * Battery
 * */
REGISTER_TYPE(battery_load, int, false);

#endif //DSR_ATTR_NAME_H
