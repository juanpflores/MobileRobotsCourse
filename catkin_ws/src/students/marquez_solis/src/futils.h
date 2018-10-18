#ifndef FUTILS_H_SRC
#define FUTILS_H_SRC

#include <iostream>
#include <functional>
#include <array>
#include <cmath>
constexpr double PI = 3.1415926535897932384626433832795028841971;
inline double norm_angle (double angle){
    double trunc = fmod(angle,2*PI);
    if (trunc > PI) return -2*PI + trunc;
    else if (trunc < -PI) return 2*PI + trunc;
    else return trunc;
}
inline double raised_cosine_norm(double arg){
    return std::abs(arg) < 1 ? 0.5 + 0.5*cos(arg*PI) : 0; 
}
inline double single_sine_norm(double arg){
    if (arg < -1) return -1;
    if (arg > 1) return 1;
    auto a = sin(arg*PI/2);
    return a*a*a;
}

using namespace std::placeholders;

template <class T>
using fun = std::function<T>;

using std::array;
template<int N>
class Vec{
public:
    double n[N];
    inline Vec<N> operator+(const Vec<N> &rhs) const{
        Vec<N> res;
        for (int i = 0;i < N; i++)
            res[i] = n[i] + rhs[i];
        return res;
    }
    inline Vec<N> operator-(const Vec<N> &rhs) const{
        Vec<N> res;
        for (int i = 0;i < N; i++)
            res.n[i] = n[i] - rhs[i];
        return res;
    }
    inline Vec<N> operator*( double rhs) const{
        Vec<N> res;
        for (int i = 0;i < N; i++)
            res.n[i] = n[i] *rhs;
        return res;
    }
    inline Vec<N>& operator=(const Vec<N> &rhs){
        for (int i = 0;i < N; i++)
            n[i] = rhs[i];
        return *this;
    }
    inline Vec<N>& operator=(double rhs){
        for (int i = 0;i < N; i++)
            n[i] = rhs;
        return *this;
    }
    inline Vec<N>& operator+=(const Vec<N> &rhs){
        for (int i = 0;i < N; i++)
            n[i] += rhs[i];
        return *this;
    }
    inline Vec<N>& operator*=(double rhs){
        for (int i = 0;i < N; i++)
            n[i] *= rhs;
        return *this;
    }
    inline double& operator[](int i){
        return n[i];
    }
    inline const double& operator[](int i) const{
        return n[i];
    }
    inline static Vec<N> setAll(double k){
        Vec<N> res;
        for (int i = 0;i < N; i++)
            res.n[i] = k;
        return res;
    }
    template<int K>
    friend std::ostream& operator<<(std::ostream& os,Vec<K> out){
        os << "{";
        for (int i = 0;i < N; i++)
            os << out.n[i] << ",";
        return os << "}";
    }
};

template <class E>
E zero (){
    E i;
    return i;
}

template <>
double zero<double>(){
    return 0;
}

template <>
Vec<2> zero< Vec<2> >(){
    return Vec<2>::setAll(0);
}


#endif