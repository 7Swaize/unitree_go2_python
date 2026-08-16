#ifndef UTILS_TYPING_HPP_
#define UTILS_TYPING_HPP_

#include <concepts>

template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <typename T, typename... Us>
concept NumericOrSpecial = std::integral<T> || std::floating_point<T> || (std::is_same_v<T, Us> || ...);

#endif