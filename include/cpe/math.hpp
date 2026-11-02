#pragma once
#include <algorithm>
#include <cmath>

namespace cpe {
struct Vec2 { double x=0, z=0; };
inline Vec2 operator+(Vec2 a,Vec2 b){return {a.x+b.x,a.z+b.z};}
inline Vec2 operator-(Vec2 a,Vec2 b){return {a.x-b.x,a.z-b.z};}
inline Vec2 operator*(Vec2 a,double s){return {a.x*s,a.z*s};}
inline double Dot(Vec2 a,Vec2 b){return a.x*b.x+a.z*b.z;}
inline double Cross(Vec2 a,Vec2 b){return a.x*b.z-a.z*b.x;}
inline double Length(Vec2 a){return std::hypot(a.x,a.z);}
inline double Distance(Vec2 a,Vec2 b){return Length(a-b);}
inline Vec2 Normalized(Vec2 a){double d=Length(a);return d>1e-10?a*(1/d):Vec2{};}
inline bool Finite(Vec2 a){return std::isfinite(a.x)&&std::isfinite(a.z);}
inline bool Near(Vec2 a,Vec2 b){return Distance(a,b)<1e-7;}
struct Box { Vec2 min,max; };
inline bool PointInside(Box b,Vec2 p){
    return p.x>=b.min.x && p.x<=b.max.x && p.z>=b.min.z && p.z<=b.max.z;
}
inline bool DiskOverlaps(Box b,Vec2 p,double radius){
    Vec2 nearest{std::clamp(p.x,b.min.x,b.max.x),std::clamp(p.z,b.min.z,b.max.z)};
    return Distance(p,nearest)<radius-1e-8;
}
// Segment against a closed AABB, using a parametric slab test.
inline bool SegmentHits(Box b,Vec2 a,Vec2 end){
    Vec2 d=end-a; double lo=0,hi=1;
    auto slab=[&](double origin,double delta,double mn,double mx){
        if(std::abs(delta)<1e-12) return origin>=mn && origin<=mx;
        double u=(mn-origin)/delta,v=(mx-origin)/delta;
        if(u>v)std::swap(u,v);
        lo=std::max(lo,u);hi=std::min(hi,v);return lo<=hi;
    };
    return slab(a.x,d.x,b.min.x,b.max.x)&&slab(a.z,d.z,b.min.z,b.max.z);
}
inline Box Expanded(Box b,double r){return {{b.min.x-r,b.min.z-r},{b.max.x+r,b.max.z+r}};}
}
