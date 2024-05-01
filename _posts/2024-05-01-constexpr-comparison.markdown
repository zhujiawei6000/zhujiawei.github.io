---
layout: post
title:  "Constexpr implementation of comparison functions"
date:   2024-03-03 11:58:00
categories: C++
brief: "Constexpr transfer the computation from runtime to compile-time. It is exclusively used in c++ meta programming. This time I will explain how to implement constexpr comparison functions."
---

## Compare functions
In C++, comparision function is a binary function that inputs the values to compare and outputs the compare results like `bool operator<(T a, T b)`. So 
- what if a and b are not same type
- what if I want to compare the values at compile time

## Naive Implementation
To solve the problems above, we define the function in the following
```c++
template<typename T, typename U>
constexpr cmp_less(T t, U u) {
    return t < u;
}
```
The code works in most cases, but warns when one of the parameter is signed and another is unsigned.
```c++
cmp_less(1U, -2);  // warning: comparison of integer expressions of different signedness: 'unsigned int' and 'int' [-Wsign-compare]
```
The compiler warns there is a comparison between signed and unsigned. To avoid this warning, we can use ```if constexpr``` to specify the case and make explicit conversions to unsigned integers. For example
```c++
template<typename T, typename U>
constexpr cmp_less(T t, U u) {
    if constexpr (std::is_signed<T>::value &&
                 !std::is_signed<U>::value) {
        return t < 0 ? true : 
                       std::make_unsigned_t<T>(t) < u;
    } else if constexpr (std::is_signed<U>::value &&
              !std::is_signed<T>::value) {
        return u < 0 ? false :
                       t < std::make_unsigned_t<U>(u);
    } else {
        return t < u;
    }
}
```
The code above is valid for C++17 since that we support ```if constexpr``` feature. However, if our project is limited to C++14 standard, then we have to make a little more effort to implement the if statement. Solution is C++ SFINAE(Substitutor Failure Is Not An Error) pattern.

## C++14 Implementation
Instead of using ```t < u```, we define a functor class that implements the ```operator()``` operator. For example
```c++
template<typename T, typename U, typename Enable = void>
struct CmpLessImpl;

template<typename T, typename U>
constexpr bool cmp_less(T t, U u) {
    return CmpLessImpl<T, U>{}(t, u);
}
```

Here the last template parameter```typename Enable``` is the if statement which will tell the compiler what version should be used. Then we define the rest versions of the ```Class CmpLessImpl```
```c++
template<typename T, typename U>
struct CmpLessImpl<T, U, std::enable_if_t<std::is_signed<T>::value == std::is_signed<U>::value>> {
    constexpr bool operator()(T t, U u) {
        return t < u;
    }
};

template<typename T, typename U>
struct CmpLessImpl<T, U, std::enable_if_t<std::is_signed<T>::value && !std::is_signed<U>::value>> {
    constexpr bool operator()(T t, U u) {
        return t < 0 ? true : std::make_unsigned_t<T>(t) < u;
    }
};

template<typename T, typename U>
struct CmpLessImpl<T, U, std::enable_if_t<!std::is_signed<T>::value && std::is_signed<U>::value>> {
    constexpr bool operator()(T t, U u) {
        return u < 0 ? false : t < std::make_unsigned_t<U>(u);
    }
};
```
When we compile the program, compiler will evaluate the ```std::enable_if_t``` with type ```T``` and ```U``` and choose the corresponding version of ```CmpLessImpl```. If all of the class specifications failed, we will fall back to the default implementation which has no definition and output error message to you.

## Conlusion
In C++ constexpr is a great tool to build efficient code. It also makes the code safer because it shifts the runtime error to compile errors. Thanks to SFINAE pattern we can make constexpr powerful even in C++14 standard.