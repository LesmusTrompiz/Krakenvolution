#ifndef point2d_CONST

#define point2d_CONST

struct Point2d
{
    float x;
    float y;
    Point2d()                   : x{0.0} , y{0.0} {};
    Point2d(float _x, float _y) : x{_x}  , y{_y}  {};

    friend inline bool operator==(const Point2d& lhs, const Point2d& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y); 
    }

    friend inline bool operator!=(const Point2d& lhs, const Point2d& rhs) {
        return !(lhs == rhs);
    }

};



#endif