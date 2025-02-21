#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <algorithm>
#include <array>
#include <bit>
#include <bitset>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace Utility {

    template <typename T>
    concept Trivial = std::is_trivial_v<T>;

    template <typename T>
    concept Arithmetic = std::is_arithmetic_v<T>;

    template <std::floating_point T>
    [[nodiscard]] T differentiate(T const value,
                                  T const prev_value,
                                  T const sampling_time,
                                  T const prev_derivative,
                                  T const time_constant)
    {
        if (time_constant + sampling_time == static_cast<T>(0)) {
            throw std::runtime_error{"Division by 0!"};
        }
        return (value - prev_value + prev_derivative * time_constant) / (time_constant + sampling_time);
    }

    template <std::floating_point T>
    [[nodiscard]] T differentiate(T const value, T const prev_value, T const sampling_time)
    {
        if (sampling_time == static_cast<T>(0)) {
            throw std::runtime_error{"Division by 0!"};
        }
        return (value - prev_value) / sampling_time;
    }

    template <std::floating_point T>
    [[nodiscard]] T integrate(T const value, T const prev_value, T const sampling_time) noexcept
    {
        return (value + prev_value) * static_cast<T>(0.5F) * sampling_time;
    }

    template <std::floating_point T>
    [[nodiscard]] T degrees_to_radians(T const degrees) noexcept
    {
        return degrees * std::numbers::pi_v<T> / static_cast<T>(360.0);
    }

    template <std::floating_point T>
    [[nodiscard]] T radians_to_degrees(T const radians) noexcept
    {
        return radians * static_cast<T>(360.0) / std::numbers::pi_v<T>;
    }

    template <std::floating_point Float, std::integral Int>
    [[nodiscard]] Int degrees_to_steps(Float const degrees, Int const steps_per_360) noexcept
    {
        return static_cast<Int>(degrees) * steps_per_360 / static_cast<Int>(360.0);
    }

    template <std::floating_point Float, std::integral Int>
    [[nodiscard]] Int degree_diff_to_step_diff(Float const degree_diff, Int const steps_per_360) noexcept
    {
        return (static_cast<Int>(degrees_to_steps(degree_diff, steps_per_360)) + steps_per_360) % steps_per_360;
    }

    template <std::integral Int, std::floating_point Float>
    [[nodiscard]] Float steps_to_degrees(Int const steps, Int const steps_per_360) noexcept
    {
        return static_cast<Float>(steps) * static_cast<Float>(360.0) / steps_per_360;
    }

    template <std::floating_point Float, std::integral Int>
    [[nodiscard]] Float step_diff_to_degree_diff(Int const step_diff, Int const steps_per_360) noexcept
    {
        return std::modulus<Float>{}(steps_to_degrees(step_diff, steps_per_360) + static_cast<Float>(360.0),
                                     static_cast<Float>(360.0));
    }

    template <Trivial Value>
    [[nodiscard]] std::array<std::uint8_t, sizeof(Value)> value_to_bytes(Value const& value) noexcept
    {
        std::array<std::uint8_t, sizeof(Value)> bytes{};
        std::memcpy(bytes.data(), std::addressof(value), sizeof(Value));
        return bytes;
    }

    template <Trivial Value>
    [[nodiscard]] Value bytes_to_value(std::array<std::uint8_t, sizeof(Value)> const& bytes) noexcept
    {
        Value value{};
        std::memcpy(std::addressof(value), bytes.data(), sizeof(Value));
        return value;
    }

    template <std::unsigned_integral UInt>
    inline void write_bits(UInt& data,
                           UInt const write_data,
                           std::uint8_t const write_size,
                           std::uint8_t const write_position) noexcept
    {
        UInt mask = ((1U << write_size) - 1U) << (write_position - write_size + 1U);
        UInt temp = (write_data << (write_position - write_size + 1U)) & mask;
        data &= ~mask;
        data |= temp;
    }

    template <std::unsigned_integral UInt>
    inline void write_bit(UInt& data, bool const write_data, std::uint8_t const write_position) noexcept
    {
        write_data ? (data |= (1U << write_position)) : (data &= ~(1U << write_position));
    }

    template <std::unsigned_integral UInt>
    inline UInt read_bits(UInt data, std::uint8_t const read_size, std::uint8_t const read_position) noexcept
    {
        UInt mask = ((1U << read_size) - 1U) << (read_position - read_size + 1U);
        data &= mask;
        data >>= (read_position - read_size + 1U);
        return data;
    }

    template <std::unsigned_integral UInt>
    inline bool read_bit(UInt data, std::uint8_t const read_position) noexcept
    {
        return (data & (1U << read_position)) ? true : false;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE / 8> bits_to_bytes(std::bitset<SIZE> const& bits) noexcept
    {
        static_assert(SIZE % 8 == 0);
        std::array<std::uint8_t, SIZE / 8> bytes{};
        for (std::size_t i{}; i < bytes.size(); ++i) {
            for (std::size_t j{}; j < 8; ++j) {
                write_bit(bytes[i], bits[i * 8 + j], j);
            }
        }
        return bytes;
    }

    template <std::size_t SIZE>
    inline std::bitset<8 * SIZE> bytes_to_bits(std::array<std::uint8_t, SIZE> const& bytes) noexcept
    {
        std::bitset<8 * SIZE> bits{};
        for (std::size_t i{}; i < bytes.size(); ++i) {
            for (std::size_t j{}; j < 8; ++j) {
                bits[i * 8 + j] = read_bit(bytes[i], j);
            }
        }
        return bits;
    }

    inline std::uint8_t bits_to_byte(std::bitset<8UL> const bits) noexcept
    {
        std::uint8_t byte{};
        for (std::uint8_t i{}; i < 8UL; ++i) {
            write_bit(byte, bits[i], i);
        }
        return byte;
    }

    inline std::bitset<8UL> byte_to_bits(std::uint8_t const byte) noexcept
    {
        std::bitset<8UL> bits{};
        for (std::uint8_t i{}; i < 8UL; ++i) {
            bits[i] = read_bit(byte, i);
        }
        return bits;
    }

    inline std::uint16_t big_endian_bytes_to_word(std::array<std::uint8_t, 2UL> const bytes) noexcept
    {
        return static_cast<std::uint16_t>(bytes[0UL] << 8UL) | static_cast<std::uint16_t>(bytes[1UL]);
    }

    inline std::uint16_t little_endian_bytes_to_word(std::array<std::uint8_t, 2UL> const bytes) noexcept
    {
        return static_cast<std::uint16_t>(bytes[0UL]) | static_cast<std::uint16_t>(bytes[1UL] << 8UL);
    }

    inline std::array<std::uint8_t, 2UL> word_to_big_endian_bytes(std::uint16_t const word) noexcept
    {
        return {static_cast<std::uint8_t>(word >> 8UL), static_cast<std::uint8_t>(word)};
    }

    inline std::array<std::uint8_t, 2UL> word_to_little_endian_bytes(std::uint16_t const word) noexcept
    {
        return {static_cast<std::uint8_t>(word), static_cast<std::uint8_t>(word >> 8UL)};
    }

    inline std::uint32_t big_endian_bytes_to_dword(std::array<std::uint8_t, 4UL> const bytes) noexcept
    {
        return static_cast<std::uint32_t>(bytes[0UL] << 24UL) | static_cast<std::uint32_t>(bytes[1UL] << 16UL) |
               static_cast<std::uint32_t>(bytes[2UL] << 8UL) | static_cast<std::uint16_t>(bytes[3UL]);
    }

    inline std::uint32_t little_endian_bytes_to_dword(std::array<std::uint8_t, 4UL> const bytes) noexcept
    {
        return static_cast<std::uint32_t>(bytes[0UL]) | static_cast<std::uint32_t>(bytes[1UL] << 8UL) |
               static_cast<std::uint32_t>(bytes[2UL] << 16UL) | static_cast<std::uint32_t>(bytes[3UL] << 24UL);
    }

    inline std::array<std::uint8_t, 4UL> dword_to_big_endian_bytes(std::uint32_t const dword) noexcept
    {
        return {static_cast<std::uint8_t>(dword >> 24UL),
                static_cast<std::uint8_t>(dword >> 16UL),
                static_cast<std::uint8_t>(dword >> 8UL),
                static_cast<std::uint8_t>(dword)};
    }

    inline std::array<std::uint8_t, 4UL> dword_to_little_endian_bytes(std::uint32_t const dword) noexcept
    {
        return {static_cast<std::uint8_t>(dword),
                static_cast<std::uint8_t>(dword >> 8UL),
                static_cast<std::uint8_t>(dword >> 16UL),
                static_cast<std::uint8_t>(dword >> 24UL)};
    }

    template <std::size_t SIZE>
    inline std::array<std::uint16_t, SIZE / 2>
    big_endian_bytes_to_words(std::array<std::uint8_t, SIZE> const& bytes) noexcept
    {
        static_assert(SIZE % 2 == 0);
        std::array<std::uint16_t, SIZE / 2> words{};
        for (std::size_t i{}; i < words.size(); ++i) {
            words[i] = static_cast<std::uint16_t>(bytes[2 * i] << 8) | static_cast<std::uint16_t>(bytes[2 * i + 1]);
        }
        return words;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint16_t, SIZE / 2>
    little_endian_bytes_endian_to_words(std::array<std::uint8_t, SIZE> const& bytes) noexcept
    {
        static_assert(SIZE % 2 == 0);
        std::array<std::uint16_t, SIZE / 2> words{};
        for (std::size_t i{}; i < words.size(); ++i) {
            words[i] = static_cast<std::uint16_t>(bytes[2 * i]) | static_cast<std::uint16_t>(bytes[2 * i + 1] << 8);
        }
        return words;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint16_t, SIZE / 2> bytes_to_words(std::array<std::uint8_t, SIZE> const& bytes,
                                                              std::endian const endian = std::endian::big) noexcept
    {
        return endian == std::endian::little ? little_endian_bytes_endian_to_words(bytes)
                                             : big_endian_bytes_to_words(bytes);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 2 * SIZE>
    words_to_big_endian_bytes(std::array<std::uint16_t, SIZE> const& words) noexcept
    {
        std::array<std::uint8_t, 2 * SIZE> bytes{};
        for (std::size_t i{}; i < words.size(); ++i) {
            bytes[2 * i] = static_cast<std::uint8_t>(words[i] >> 8);
            bytes[2 * i + 1] = static_cast<std::uint8_t>(words[i]);
        }
        return bytes;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 2 * SIZE>
    words_to_little_endian_bytes(std::array<std::uint16_t, SIZE> const& words) noexcept
    {
        std::array<std::uint8_t, 2 * SIZE> bytes{};
        for (std::size_t i{}; i < words.size(); ++i) {
            bytes[2 * i] = static_cast<std::uint8_t>(words[i]);
            bytes[2 * i + 1] = static_cast<std::uint8_t>(words[i] >> 8);
        }
        return bytes;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 2 * SIZE> words_to_bytes(std::array<std::uint16_t, SIZE> const& words,
                                                             std::endian const endian = std::endian::big) noexcept
    {
        return endian == std::endian::little ? words_to_little_endian_bytes(words) : words_to_big_endian_bytes(words);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint32_t, SIZE / 4>
    big_endian_bytes_to_dwords(std::array<std::uint8_t, SIZE> const& bytes) noexcept
    {
        static_assert(SIZE % 4 == 0);
        std::array<std::uint32_t, SIZE / 4> dwords{};
        for (std::size_t i{}; i < dwords.size(); ++i) {
            dwords[i] =
                static_cast<std::uint32_t>(bytes[2 * i] << 24) | static_cast<std::uint32_t>(bytes[2 * i + 1] << 16) |
                static_cast<std::uint32_t>(bytes[2 * i + 2] << 8) | static_cast<std::uint32_t>(bytes[2 * i + 3]);
        }
        return dwords;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint32_t, SIZE / 4>
    little_endian_bytes_endian_to_dwords(std::array<std::uint8_t, SIZE> const& bytes) noexcept
    {
        static_assert(SIZE % 4 == 0);
        std::array<std::uint32_t, SIZE / 4> dwords{};
        for (std::size_t i{}; i < dwords.size(); ++i) {
            dwords[i] = static_cast<std::uint32_t>(bytes[2 * i]) | static_cast<std::uint32_t>(bytes[2 * i + 1] << 8) |
                        static_cast<std::uint32_t>(bytes[2 * i + 2] << 16) |
                        static_cast<std::uint32_t>(bytes[2 * i + 3] << 24);
        }
        return dwords;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint32_t, SIZE / 4> bytes_to_dwords(std::array<std::uint8_t, SIZE> const& bytes,
                                                               std::endian const endian = std::endian::big) noexcept
    {
        return endian == std::endian::little ? little_endian_bytes_endian_to_dwords(bytes)
                                             : big_endian_bytes_to_dwords(bytes);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 4 * SIZE>
    dwords_to_big_endian_bytes(std::array<std::uint32_t, SIZE> const& dwords) noexcept
    {
        std::array<std::uint8_t, 4 * SIZE> bytes{};
        for (std::size_t i{}; i < dwords.size(); ++i) {
            bytes[2 * i] = static_cast<std::uint8_t>(dwords[i] >> 24);
            bytes[2 * i + 1] = static_cast<std::uint8_t>(dwords[i] >> 16);
            bytes[2 * i + 2] = static_cast<std::uint8_t>(dwords[i] >> 8);
            bytes[2 * i + 3] = static_cast<std::uint8_t>(dwords[i]);
        }
        return bytes;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 4 * SIZE>
    dwords_to_little_endian_bytes_endian(std::array<std::uint32_t, SIZE> const& dwords) noexcept
    {
        std::array<std::uint8_t, 4 * SIZE> bytes{};
        for (std::size_t i{}; i < dwords.size(); ++i) {
            bytes[2 * i] = static_cast<std::uint8_t>(dwords[i]);
            bytes[2 * i + 1] = static_cast<std::uint8_t>(dwords[i] >> 8);
            bytes[2 * i + 2] = static_cast<std::uint8_t>(dwords[i] >> 16);
            bytes[2 * i + 3] = static_cast<std::uint8_t>(dwords[i] >> 24);
        }
        return bytes;
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, 4 * SIZE> dwords_to_bytes(std::array<std::uint32_t, SIZE> const& dwords,
                                                              std::endian const endian = std::endian::big) noexcept
    {
        return endian == std::endian::little ? dwords_to_little_endian_bytes_endian(dwords)
                                             : dwords_to_big_endian_bytes(dwords);
    }

    template <Arithmetic From, Arithmetic To>
    inline To
    rescale(From const from_value, From const from_min, From const from_max, To const to_min, To const to_max) noexcept
    {
        return static_cast<To>(std::clamp(from_value, from_min, from_max) - from_min) * (to_max - to_min) /
                   static_cast<To>(from_max - from_min) -
               to_min;
    }

}; // namespace Utility

#endif // UTILITY_HPP