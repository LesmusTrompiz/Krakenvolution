#pragma once

/*
    Este struct permite manejar comodamete la posición del robot,
    es lo que antes se conocia como geometry_msg::msg::Pose2D
*/
struct Pose2d
{
    float x;
    float y;
    float a;

    Pose2d()                             : x{0.0} , y{0.0} , a{0.0} {};
    Pose2d(float _x, float _y, float _a) : x{_x}  , y{_y}  , a{_a}  {};

    friend inline bool operator==(const Pose2d& lhs, const Pose2d& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.a == rhs.a); 
    }

    friend inline bool operator!=(const Pose2d& lhs, const Pose2d& rhs) {
        return !(lhs == rhs);
    }
};
