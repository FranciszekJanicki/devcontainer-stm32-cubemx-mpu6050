#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstdint>
#include <utility>

namespace Utility {

    struct I2CDevice {
    public:
        I2CDevice() noexcept = default;
        I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept;

        I2CDevice(I2CDevice const& other) noexcept = delete;
        I2CDevice(I2CDevice&& other) noexcept = default;

        I2CDevice& operator=(I2CDevice const& other) noexcept = delete;
        I2CDevice& operator=(I2CDevice&& other) noexcept = default;

        ~I2CDevice() noexcept = default;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_dword(DWord const transmit_data) const noexcept;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_word(Word const transmit_data) const noexcept;

        template <std::size_t TRANSMIT_SIZE>
        void transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept;

        void transmit_byte(Byte const transmit_data) const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] DWords<RECEIVE_SIZE> receive_dwords() const noexcept;

        [[nodiscard]] DWord receive_dword() const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Words<RECEIVE_SIZE> receive_words() const noexcept;

        [[nodiscard]] Word receive_word() const noexcept;

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Bytes<RECEIVE_SIZE> receive_bytes() const noexcept;

        [[nodiscard]] Byte receive_byte() const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] DWord read_dword(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] Word read_word(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        void
        read_bytes(std::uint8_t const reg_address, Byte* const read_data, std::size_t const read_size) const noexcept;

        [[nodiscard]] Byte read_byte(std::uint8_t const reg_address) const noexcept;

        [[nodiscard]] Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept;

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bits<READ_SIZE> read_bits(std::uint8_t const reg_address,
                                                std::uint8_t const read_position) const noexcept;

        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address,
                                     std::uint8_t const read_position,
                                     std::size_t const read_size) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept;

        void write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept;

        void write_word(std::uint8_t const reg_address, Word const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept;

        void write_bytes(std::uint8_t const reg_address,
                         Byte* const write_data,
                         std::size_t const write_size) const noexcept;

        void write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept;

        void write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position) const noexcept;

        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept;

        [[nodiscard]] std::uint16_t device_address() const noexcept;

    private:
        static constexpr std::uint32_t I2C_TIMEOUT{100U};
        static constexpr std::uint32_t I2C_SCAN_RETRIES{10U};

        void initialize() noexcept;

        bool initialized_{false};

        I2CBusHandle i2c_bus_{nullptr};
        std::uint16_t device_address_{};
    };

    template <std::size_t TRANSMIT_SIZE>
    void I2CDevice::transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept
    {
        this->transmit_bytes(dwords_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    void I2CDevice::transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept
    {
        this->transmit_bytes(words_to_bytes(transmit_data));
    }

    template <std::size_t TRANSMIT_SIZE>
    void I2CDevice::transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept
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
    DWords<RECEIVE_SIZE> I2CDevice::receive_dwords() const noexcept
    {
        return bytes_to_dwords(this->receive_bytes<4 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    Words<RECEIVE_SIZE> I2CDevice::receive_words() const noexcept
    {
        return bytes_to_words(this->receive_bytes<2 * RECEIVE_SIZE>());
    }

    template <std::size_t RECEIVE_SIZE>
    Bytes<RECEIVE_SIZE> I2CDevice::receive_bytes() const noexcept
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
    DWords<READ_SIZE> I2CDevice::read_dwords(std::uint8_t const reg_address) const noexcept
    {
        return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    Words<READ_SIZE> I2CDevice::read_words(std::uint8_t const reg_address) const noexcept
    {
        return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
    }

    template <std::size_t READ_SIZE>
    Bytes<READ_SIZE> I2CDevice::read_bytes(std::uint8_t const reg_address) const noexcept
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

    template <std::size_t READ_SIZE>
    Bits<READ_SIZE> I2CDevice::read_bits(std::uint8_t const reg_address,
                                         std::uint8_t const read_position) const noexcept
    {
        Byte read = this->read_byte(reg_address);
        Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
        read &= mask;
        read >>= (read_position - READ_SIZE + 1);
        return read;
    }

    template <std::size_t WRITE_SIZE>
    void I2CDevice::write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
    {
        this->write_bytes(reg_address, dwords_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    void I2CDevice::write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
    {
        this->write_bytes(reg_address, words_to_bytes(write_data));
    }

    template <std::size_t WRITE_SIZE>
    void I2CDevice::write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
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

    template <std::size_t WRITE_SIZE>
    void I2CDevice::write_bits(std::uint8_t const reg_address,
                               Byte const write_data,
                               std::uint8_t const write_position) const noexcept
    {
        Byte write = this->read_byte(reg_address);
        Byte mask = ((1 << WRITE_SIZE) - 1) << (write_position - WRITE_SIZE + 1);
        Byte temp = write_data << (write_position - WRITE_SIZE + 1);
        temp &= mask;
        write &= ~(mask);
        write |= temp;
        this->write_byte(reg_address, write);
    }

}; // namespace Utility

#endif // I2C_DEVICE_HPP