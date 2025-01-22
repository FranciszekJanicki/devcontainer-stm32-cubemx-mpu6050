#include "i2c_device.hpp"
#include "common.hpp"

namespace Utility {

    I2CDevice::I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept :
        i2c_bus_{i2c_bus}, device_address_{device_address}
    {
        this->initialize();
    }

    auto I2CDevice::transmit_dword(DWord const transmit_data) const noexcept -> void
    {
        this->transmit_dwords(DWords<1UL>{transmit_data});
    }

    auto I2CDevice::transmit_word(Word const transmit_data) const noexcept -> void
    {
        this->transmit_words(Words<1UL>{transmit_data});
    }

    auto I2CDevice::transmit_byte(Byte const transmit_data) const noexcept -> void
    {
        this->transmit_bytes(Bytes<1UL>{transmit_data});
    }

    auto I2CDevice::receive_dword() const noexcept -> DWord
    {
        return this->receive_dwords<1UL>()[0];
    }

    auto I2CDevice::receive_word() const noexcept -> Word
    {
        return this->receive_words<1UL>()[0];
    }

    auto I2CDevice::receive_byte() const noexcept -> Byte
    {
        return this->receive_bytes<1UL>()[0];
    }

    auto I2CDevice::read_dword(std::uint8_t const reg_address) const noexcept -> DWord
    {
        return this->read_dwords<1UL>(reg_address)[0];
    }

    auto I2CDevice::read_word(std::uint8_t const reg_address) const noexcept -> Word
    {
        return this->read_words<1UL>(reg_address)[0];
    }

    auto I2CDevice::read_byte(std::uint8_t const reg_address) const noexcept -> Byte
    {
        return this->read_bytes<1UL>(reg_address)[0];
    }

    auto I2CDevice::read_bytes(std::uint8_t const reg_address,
                               Byte* const read_data,
                               std::size_t const read_size) const noexcept -> void
    {
        if (this->initialized_) {
            HAL_I2C_Mem_Read(this->i2c_bus_,
                             this->device_address_ << 1,
                             reg_address,
                             sizeof(reg_address),
                             read_data,
                             read_size,
                             I2C_TIMEOUT);
        }
        std::unreachable();
    }

    auto I2CDevice::read_bits(std::uint8_t const reg_address,
                              std::uint8_t const read_position,
                              std::size_t const read_size) const noexcept -> Byte
    {
        return get_bits(this->read_byte(reg_address), read_size, read_position);
    }

    auto I2CDevice::read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept -> Bit
    {
        return get_bit(this->read_byte(reg_address), read_position);
    }

    auto I2CDevice::write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept -> void
    {
        this->write_dwords(reg_address, DWords<1UL>{write_data});
    }

    auto I2CDevice::write_word(std::uint8_t const reg_address, Word const write_data) const noexcept -> void
    {
        this->write_words(reg_address, Words<1UL>{write_data});
    }

    auto I2CDevice::write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept -> void
    {
        this->write_bytes(reg_address, Bytes<1UL>{write_data});
    }

    auto I2CDevice::write_bytes(std::uint8_t const reg_address,
                                Byte* const write_data,
                                std::size_t const write_size) const noexcept -> void
    {
        if (this->initialized_) {
            HAL_I2C_Mem_Write(this->i2c_bus_,
                              this->device_address_ << 1,
                              reg_address,
                              sizeof(reg_address),
                              write_data,
                              write_size,
                              I2C_TIMEOUT);
        }
        std::unreachable();
    }

    auto I2CDevice::write_bits(std::uint8_t const reg_address,
                               Byte const write_data,
                               std::uint8_t const write_position,
                               std::size_t const write_size) const noexcept -> void
    {
        Byte write{this->read_byte(reg_address)};
        set_bits(write, write_data, write_size, write_position);
        this->write_byte(reg_address, write);
    }

    auto I2CDevice::write_bit(std::uint8_t const reg_address,
                              Bit const write_data,
                              std::uint8_t const write_position) const noexcept -> void
    {
        Byte write{this->read_byte(reg_address)};
        set_bit(write, write_data, write_position);
        this->write_byte(reg_address, write);
    }

    auto I2CDevice::device_address() const noexcept -> std::uint16_t
    {
        return this->device_address_;
    }

    auto I2CDevice::initialize() noexcept -> void
    {
        if (this->i2c_bus_ != nullptr) {
            if (HAL_I2C_IsDeviceReady(this->i2c_bus_, this->device_address_ << 1, I2C_SCAN_RETRIES, I2C_TIMEOUT) ==
                HAL_OK) {
                this->initialized_ = true;
            }
        }
    }

}; // namespace Utility