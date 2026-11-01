#pragma once
#include <array>
#include <cmath>
#include <cstddef>
namespace planner{

template <typename T, std::size_t Dim>
struct Point{
    std::array<T, Dim> coords{};
    T& operator[](std::size_t i){return coords[i];}
    const T& operator[](std::size_t i) const {return coords[i];}
    bool operator==(const Point&) const = default;
};

template <typename T, std::size_t Dim> 
double squaredDistance(const Point<T, Dim>& a, const Point<T, Dim>& b){
    double sum = 0.0;
    for (std::size_t i = 0; i<Dim; ++i){
        const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        sum += d*d;
    }
    return sum;
}

template <typename T, std::size_t Dim>
double distance(const Point<T, Dim>& a, const Point<T, Dim>& b){
    return std::sqrt(squaredDistance(a,b));
}



}