#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace Utility {

    struct I2CDevice {
    public:
        I2CDevice() noexcept = default;
        I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept;

        I2CDevice(I2CDevice const& other) noexcept = delete;
        I2CDevice(I2CDevice&& other) noexcept = default;

        auto operator=(I2CDevice const& other) noexcept -> I2CDevice& = delete;
        auto operator=(I2CDevice&& other) noexcept -> I2CDevice& = default;

        ~I2CDevice() noexcept = default;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_dword(DWord const transmit_data) const noexcept -> void;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_word(Word const transmit_data) const noexcept -> void;

        template <std::size_t TRANSMIT_SIZE>
        auto transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void;
        auto transmit_byte(Byte const transmit_data) const noexcept -> void;

        template <std::size_t RECEIVE_SIZE>
        auto receive_dwords() const noexcept -> [[nodiscard]] DWords<RECEIVE_SIZE>;
        auto receive_dword() const noexcept -> [[nodiscard]] DWord;

        template <std::size_t RECEIVE_SIZE>
        auto receive_words() const noexcept -> [[nodiscard]] Words<RECEIVE_SIZE>;
        auto receive_word() const noexcept -> [[nodiscard]] Word;

        template <std::size_t RECEIVE_SIZE>
        auto receive_bytes() const noexcept -> [[nodiscard]] Bytes<RECEIVE_SIZE>;
        auto receive_byte() const noexcept -> [[nodiscard]] Byte;

        template <std::size_t READ_SIZE>
        auto read_dwords(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] DWords<READ_SIZE>;
        auto read_dword(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] DWord;

        template <std::size_t READ_SIZE>
        auto read_words(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Words<READ_SIZE>;
        auto read_word(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Word;

        template <std::size_t READ_SIZE>
        auto read_bytes(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Bytes<READ_SIZE>;
        auto read_byte(std::uint8_t const reg_address) const noexcept -> [[nodiscard]] Byte;
        auto read_bytes(std::uint8_t const reg_address,
                        Byte* const read_data,
                        std::size_t const read_size) const noexcept -> void;

        auto read_bits(std::uint8_t const reg_address,
                       std::uint8_t const read_position,
                       std::size_t const read_size) const noexcept -> [[nodiscard]] Byte;
        auto read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept ->
            [[nodiscard]] Bit;

        template <std::size_t WRITE_SIZE>
        auto write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept -> void;

        template <std::size_t WRITE_SIZE>
        auto write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_word(std::uint8_t const reg_address, Word const write_data) const noexcept -> void;

        template <std::size_t WRITE_SIZE>
        auto write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept -> void;
        auto write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept -> void;
        auto write_bytes(std::uint8_t const reg_address,
                         Byte* const write_data,
                         std::size_t const write_size) const noexcept -> void;

        auto write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept -> void;
        auto write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept -> void;

        auto device_address() const noexcept -> [[nodiscard]] std::uint16_t;

    private:
        static constexpr std::uint32_t I2C_TIMEOUT{100U};
        static constexpr std::uint32_t I2C_SCAN_RETRIES{10U};

        void initialize() noexcept;

        bool initialized_{false};

        I2CBusHandle i2c_bus_{nullptr};
        std::uint16_t device_address_{};
    };

    template <std::size_t TRANSMIT_SIZE>
    auto I2CDevice::transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        this->transmit_bytes(dwords_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    auto I2CDevice::transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        this->transmit_bytes(words_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    auto I2CDevice::transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept -> void
    {
        if (this->initialized_) {
            Bytes<TRANSMIT_SIZE> transmit{transmit_data};
            HAL_I2C_Master_Transmit(this->i2c_bus_,
                                    this->device_address_ << 1,
                                    transmit.data(),
                                    transmit.size(),
                                    I2C_TIMEOUT);
        }
    }

    template <std::size_t RECEIVE_SIZE>
    auto I2CDevice::receive_dwords() const noexcept -> DWords<RECEIVE_SIZE>
    {
        return bytes_to_dwords(this->receive_bytes<4 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    auto I2CDevice::receive_words() const noexcept -> Words<RECEIVE_SIZE>
    {
        return bytes_to_words(this->receive_bytes<2 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    auto I2CDevice::receive_bytes() const noexcept -> Bytes<RECEIVE_SIZE>
    {
        if (this->initialized_) {
            Bytes<RECEIVE_SIZE> receive{};
            HAL_I2C_Master_Receive(this->i2c_bus_,
                                   this->device_address_ << 1,
                                   receive.data(),
                                   receive.size(),
                                   I2C_TIMEOUT);
            return receive;
        }
        std::unreachable();
    }

    template <std::size_t READ_SIZE>
    auto I2CDevice::read_dwords(std::uint8_t const reg_address) const noexcept -> DWords<READ_SIZE>
    {
        return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    auto I2CDevice::read_words(std::uint8_t const reg_address) const noexcept -> Words<READ_SIZE>
    {
        return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    auto I2CDevice::read_bytes(std::uint8_t const reg_address) const noexcept -> Bytes<READ_SIZE>
    {
        if (this->initialized_) {
            Bytes<READ_SIZE> read{};
            HAL_I2C_Mem_Read(this->i2c_bus_,
                             this->device_address_ << 1,
                             reg_address,
                             sizeof(reg_address),
                             read.data(),
                             read.size(),
                             I2C_TIMEOUT);
            return read;
        }
        std::unreachable();
    }

    template <std::size_t WRITE_SIZE>
    auto I2CDevice::write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        this->write_bytes(reg_address, dwords_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    auto I2CDevice::write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        this->write_bytes(reg_address, words_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    auto I2CDevice::write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
        -> void
    {
        if (this->initialized_) {
            Bytes<WRITE_SIZE> write{write_data};
            HAL_I2C_Mem_Write(this->i2c_bus_,
                              this->device_address_ << 1,
                              reg_address,
                              sizeof(reg_address),
                              write.data(),
                              write.size(),
                              I2C_TIMEOUT);
        }
    }

}; // namespace Utility

#endif // I2C_DEVICE_HPP