#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <boost/serialization/array.hpp>

namespace meshlib {

template <typename T>
class Vector {
public:
    typedef std::size_t Axis;
    typedef T           Type;

    Vector() 
    {
        for (Axis d = 0; d < 3; d++) {
            val_[d] = T();
        }
    }
    
    Vector(const T& rhs) 
    {
        for (Axis d = 0; d < 3; d++) {
            val_[d] = rhs;
        }
    }
    
    Vector(const std::array<T, 3>& rhs):
        val_{rhs}
    {}

    Vector& operator=(const std::array<T, 3>& rhs) {
        for (Axis d = 0; d < 3; d++) {
            val_[d] = rhs[d];
        }
        return *this;
    }


    T&       operator[](const std::size_t& d)       { return val_[d]; }
    const T& operator[](const std::size_t& d) const { return val_[d]; }

    T&       operator()(const std::size_t& d)       { return val_[d]; }
    const T& operator()(const std::size_t& d) const { return val_[d]; }

    Vector& operator+=(const Vector& rhs) {
        for (Axis d = 0; d < 3; d++) {
            val_[d] += rhs[d];
        }
        return *this;
    }
    Vector& operator-=(const Vector& rhs) {
        for (Axis d = 0; d < 3; d++) {
            val_[d] -= rhs[d];
        }
        return *this;
    }
    Vector& operator*=(const T& factor) {
        for (Axis d = 0; d < 3; d++) {
            val_[d] *= factor;
        }
        return *this;
    }
    Vector& operator/=(const T& factor) {
        for (Axis d = 0; d < 3; d++) {
            val_[d] /= factor;
        }
        return *this;
    }
    Vector& operator^=(const Vector& rhs) {
        *this = *this ^ rhs;
        return *this;
    }

    Vector operator+(const Vector& rhs) const {
        Vector res(*this);
        res += rhs;
        return res;
    }
    Vector operator-(const Vector& rhs) const {
        Vector res(*this);
        res -= rhs;
        return res;
    }
    Vector operator-() const {
        Vector res(0);
        res -= *this;
        return res;
    }
    Vector operator*(const T& factor) const {
        Vector res(*this);
        res *= factor;
        return res;
    }
    double operator*(const Vector& rhs) const {
        double res = 0.0;
        for (Axis d = 0; d < 3; d++) {
            res += val_[d] * rhs[d];
        }
        return res;
    }
    Vector operator/(const T& factor) const {
        Vector res(*this);
        res /= factor;
        return res;
    }
    Vector operator^(const Vector& rhs) const {
        Vector res;
        res.val_[0] = val_[1] * rhs.val_[2] - rhs.val_[1] * val_[2];
        res.val_[1] = val_[2] * rhs.val_[0] - rhs.val_[2] * val_[0];
        res.val_[2] = val_[0] * rhs.val_[1] - rhs.val_[0] * val_[1];
        return res;
    }

    bool operator==(const Vector& rhs) const {
        return val_ == rhs.val_;
    }
    bool operator!=(const Vector& rhs) const {
        return val_ != rhs.val_;
    }
    bool operator<=(const Vector& rhs) const {
        return val_ <= rhs.val_;
    }
    bool operator< (const Vector& rhs) const {
        return val_ < rhs.val_;
    }
    bool operator>=(const Vector& rhs) const {
        return val_ >= rhs.val_;
    }
    bool operator> (const Vector& rhs) const {
        return val_ > rhs.val_;
    }

    Vector abs() const {
        Vector res;
        for (Axis d = 0; d < 3; d++) {
            res[d] = std::abs((*this)[d]);
        }
        return res;
    }

    Vector round(const T& tolerance) const {
        Vector res;
        for (Axis d = 0; d < 3; d++) {
            res[d] = std::round((*this)[d] * tolerance) / tolerance;
        }
        return res;
    }

    T sum() const {
        T res = 0;
        for (std::size_t i = 0; i < 3; i++) {
            res += (*this)[i];
        }
        return res;
    }

    double norm() const {
        return std::sqrt((*this)*(*this));
    }

    double angleDeg(const Vector<double>& vec) const {
        const double pi = atan(1) * 4.0;
        return angle(vec) / (2.0 * pi) * 360.0;
    }

    double angle(const Vector<double>& vec) const {
        if (this->norm() == 0.0 || vec.norm() == 0.0) {
            throw std::logic_error(
                "Error determining angle between zero length vectors");
        }
        Vector<double> v1 = *this / this->norm();
        Vector<double> v2 = vec / vec.norm();
        double aux = v1 * v2;
        if (aux > 1.0) {
            aux = 1.0;
        }
        if (aux < -1.0) {
            aux = -1.0;
        }
        return acos(aux);
    }

    template<class S>
    Vector<S> as() const {
        Vector<S> res;
        for (std::size_t i = 0; i < 3; i++) {
            res[i] = (S)(*this)[i];
        }
        return res;
    }

    std::string str() const {
        std::stringstream s;
        s << "(";
        for (std::size_t i = 0; i < val_.size(); i++) {
            s << val_[i];
            if (i + 1 < val_.size()) {
                s << ", ";
            }
        }
        s << ")";
        return s.str();
    }

private:
    std::array<T, 3> val_;
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& val_;
    }
};

typedef Vector<double> VecD;
typedef Vector<float> VecF;
typedef Vector<int> VecI;
}

