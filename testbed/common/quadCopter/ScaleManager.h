//
// Created by kirill on 13.04.2020.
//

#ifndef REACTPHYSICS3D_SCALEMANAGER_H
#define REACTPHYSICS3D_SCALEMANAGER_H

#include <iostream>
#include <functional>
#include <utility>


namespace quad {
    template<typename OutType, typename ArgType>
    class ScaleManager {
    private:
        // Attributes
        std::function<OutType(ArgType)> _f;

    public:
        // Constructor
        explicit ScaleManager(std::function<OutType(ArgType)> f) : _f(std::move(f)) {};

        // Making the object callable
        OutType operator()(ArgType x) { return _f(x); }

        std::function<OutType(ArgType)> getFunction() const;

        void setFunction(std::function<OutType(ArgType)> f);
    };

    template<typename OutType, typename ArgType>
    std::function<OutType(ArgType)> ScaleManager<OutType, ArgType>::getFunction() const {
        return _f;
    }

    template<typename OutType, typename ArgType>
    void ScaleManager<OutType, ArgType>::setFunction(const std::function<OutType(ArgType)> f) {
        _f = f;
    }
//
//    template<class OutType, class ArgType>
//    ArgType call(ScaleManager<OutType, ArgType> p, ArgType x) {
//        return p(x);
//    }
//
//    int main() {
//        ScaleManager<double, double> scaleFunc([](double x) { return x * x; });
//        std::cout << call<double, double>(scaleFunc, 2) << scaleFunc(2);
//        return 0;
//    }


}

#endif //REACTPHYSICS3D_SCALEMANAGER_H
