#ifndef FUTILS_H_SRC
#define FUTILS_H_SRC

#include <iostream>
#include <functional>
#include <array>


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