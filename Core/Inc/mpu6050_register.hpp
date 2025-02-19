#ifndef MPU6050_REGISTERS_HPP
#define MPU6050_REGISTERS_HPP

#include <cstdint>

#define packed __attribute__((__packed__))

namespace MPU6050 {

    enum struct RA : std::uint8_t {
        XG_OFFS_TC = 0x00,
        YG_OFFS_TC = 0x01,
        ZG_OFFS_TC = 0x02,
        X_FINE_GAIN = 0x03,
        Y_FINE_GAIN = 0x04,
        Z_FINE_GAIN = 0x05,
        XA_OFFS_H = 0x06,
        XA_OFFS_L_TC = 0x07,
        YA_OFFS_H = 0x08,
        YA_OFFS_L_TC = 0x09,
        ZA_OFFS_H = 0x0A,
        ZA_OFFS_L_TC = 0x0B,
        SELF_TEST_X = 0x0D,
        SELF_TEST_Y = 0x0E,
        SELF_TEST_Z = 0x0F,
        SELF_TEST_A = 0x10,
        XG_OFFS_USRH = 0x13,
        XG_OFFS_USRL = 0x14,
        YG_OFFS_USRH = 0x15,
        YG_OFFS_USRL = 0x16,
        ZG_OFFS_USRH = 0x17,
        ZG_OFFS_USRL = 0x18,
        SMPLRT_DIV = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        FF_THR = 0x1D,
        FF_DUR = 0x1E,
        MOT_THR = 0x1F,
        MOT_DUR = 0x20,
        ZRMOT_THR = 0x21,
        ZRMOT_DUR = 0x22,
        FIFO_EN = 0x23,
        I2C_MST_CTRL = 0x24,
        I2C_SLV0_ADDR = 0x25,
        I2C_SLV0_REG = 0x26,
        I2C_SLV0_CTRL = 0x27,
        I2C_SLV1_ADDR = 0x28,
        I2C_SLV1_REG = 0x29,
        I2C_SLV1_CTRL = 0x2A,
        I2C_SLV2_ADDR = 0x2B,
        I2C_SLV2_REG = 0x2C,
        I2C_SLV2_CTRL = 0x2D,
        I2C_SLV3_ADDR = 0x2E,
        I2C_SLV3_REG = 0x2F,
        I2C_SLV3_CTRL = 0x30,
        I2C_SLV4_ADDR = 0x31,
        I2C_SLV4_REG = 0x32,
        I2C_SLV4_DO = 0x33,
        I2C_SLV4_CTRL = 0x34,
        I2C_SLV4_DI = 0x35,
        I2C_MST_STATUS = 0x36,
        INT_PIN_CFG = 0x37,
        INT_ENABLE = 0x38,
        DMP_INT_STATUS = 0x39,
        INT_STATUS = 0x3A,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        EXT_SENS_DATA_00 = 0x49,
        EXT_SENS_DATA_01 = 0x4A,
        EXT_SENS_DATA_02 = 0x4B,
        EXT_SENS_DATA_03 = 0x4C,
        EXT_SENS_DATA_04 = 0x4D,
        EXT_SENS_DATA_05 = 0x4E,
        EXT_SENS_DATA_06 = 0x4F,
        EXT_SENS_DATA_07 = 0x50,
        EXT_SENS_DATA_08 = 0x51,
        EXT_SENS_DATA_09 = 0x52,
        EXT_SENS_DATA_10 = 0x53,
        EXT_SENS_DATA_11 = 0x54,
        EXT_SENS_DATA_12 = 0x55,
        EXT_SENS_DATA_13 = 0x56,
        EXT_SENS_DATA_14 = 0x57,
        EXT_SENS_DATA_15 = 0x58,
        EXT_SENS_DATA_16 = 0x59,
        EXT_SENS_DATA_17 = 0x5A,
        EXT_SENS_DATA_18 = 0x5B,
        EXT_SENS_DATA_19 = 0x5C,
        EXT_SENS_DATA_20 = 0x5D,
        EXT_SENS_DATA_21 = 0x5E,
        EXT_SENS_DATA_22 = 0x5F,
        EXT_SENS_DATA_23 = 0x60,
        MOT_DETECT_STATUS = 0x61,
        I2C_SLV0_DO = 0x63,
        I2C_SLV1_DO = 0x64,
        I2C_SLV2_DO = 0x65,
        I2C_SLV3_DO = 0x66,
        I2C_MST_DELAY_CTRL = 0x67,
        SIGNAL_PATH_RESET = 0x68,
        MOT_DETECT_CTRL = 0x69,
        USER_CTRL = 0x6A,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        BANK_SEL = 0x6D,
        MEM_START_ADDR = 0x6E,
        MEM_R_W = 0x6F,
        DMP_CFG_1 = 0x70,
        DMP_CFG_2 = 0x71,
        FIFO_COUNTH = 0x72,
        FIFO_COUNTL = 0x73,
        FIFO_R_W = 0x74,
        WHO_AM_I = 0x75,
    };

    struct XG_OFFS_TC {
        bool aux_vddio : 1;
        uint8_t xg_offs_tc : 7;
    } packed;

    struct YG_OFFS_TC {
        bool : 1;
        uint8_t yg_offs_tc : 7;
    } packed;

    struct ZG_OFFS_TC {
        bool : 1;
        uint8_t zg_offs_tc : 7;
    } packed;

    struct X_FINE_GAIN {
        uint8_t x_fine_gain : 8;
    } packed;

    struct Y_FINE_GAIN {
        uint8_t y_fine_gain : 8;
    } packed;

    struct Z_FINE_GAIN {
        uint8_t z_fine_gain : 8;
    } packed;

    struct XA_OFFS {
        uint8_t xa_offs_h : 8;
        uint8_t xa_offs_l_tc : 8;
    } packed;

    struct YA_OFFS {
        uint8_t ya_offs_h : 8;
        uint8_t ya_offs_l_tc : 8;
    } packed;

    struct ZA_OFFS {
        uint8_t za_offs_h : 8;
        uint8_t za_offs_l_tc : 8;
    } packed;

    struct XG_OFFS_USR {
        uint8_t xg_offs_usrh : 8;
        uint8_t xg_offs_usrhl : 8;
    } packed;

    struct YG_OFFS_USR {
        uint8_t yg_offs_usrh : 8;
        uint8_t yg_offs_usrhl : 8;
    } packed;

    struct ZG_OFFS_USR {
        uint8_t zg_offs_usrh : 8;
        uint8_t zg_offs_usrhl : 8;
    } packed;

    struct SELF_TEST_X {
        uint8_t xa_test : 3;
        uint8_t xg_test : 5;
    };

    struct SELF_TEST_Y {
        uint8_t ya_test : 3;
        uint8_t yg_test : 5;
    };

    struct SELF_TEST_Z {
        uint8_t za_test : 3;
        uint8_t zg_test : 5;
    };

    struct SELF_TEST_A {
        uint8_t : 2;
        uint8_t xa_test : 2;
        uint8_t ya_test : 2;
        uint8_t za_test : 2;
    };

    struct SMPLRT_DIV {
        uint8_t smplrt_div : 8;
    } packed;

    struct CONFIG {
        uint8_t : 2;
        uint8_t ext_sync_set : 3;
        uint8_t dlpf_cfg : 3;
    } packed;

    struct GYRO_CONFIG {
        bool xg_st : 1;
        bool yg_st : 1;
        bool zg_st : 1;
        uint8_t fs_sel : 2;
        uint8_t : 3;
    } packed;

    struct ACCEL_CONFIG {
        bool xa_st : 1;
        bool ya_st : 1;
        bool za_st : 1;
        uint8_t afs_sel : 2;
        uint8_t accel_hpf : 3;
    } packed;

    struct FF_THR {
        uint8_t ff_thr : 8;
    } packed;

    struct FF_DUR {
        uint8_t ff_dur : 8;
    } packed;

    struct MOT_THR {
        uint8_t mot_thr : 8;
    } packed;

    struct MOT_DUR {
        uint8_t mot_dur : 8;
    } packed;

    struct ZRMOT_THR {
        uint8_t zrmot_thr : 8;
    } packed;

    struct ZRMOT_DUR {
        uint8_t zrmot_dur : 8;
    } packed;

    struct FIFO_EN {
        bool temp_fifo_en : 1;
        bool xg_fifo_en : 1;
        bool yg_fifo_en : 1;
        bool zg_fifo_en : 1;
        bool accel_fifo_en : 1;
        bool slv2_fifo_en : 1;
        bool slv1_fifo_en : 1;
        bool slv0_fifo_en : 1;
    } packed;

    struct I2C_MST_CTRL {
        bool mult_mst_en : 1;
        bool wait_for_es : 1;
        bool slv3_fifo_en : 1;
        bool i2c_mst_p_nsr : 1;
        uint8_t i2c_mst_clk : 4;
    } packed;

    struct I2C_SLV_ADDR {
        bool i2c_slv_rw : 1;
        uint8_t i2c_slv_addr : 7;
    } packed;

    struct I2C_SLV_REG {
        uint8_t i2c_slv_reg : 8;
    } packed;

    struct I2C_SLV_CTRL {
        bool i2c_slv_en : 1;
        bool i2c_slv_byte_sw : 1;
        bool i2c_slv_reg_dis : 1;
        bool i2c_slv_grp : 1;
        uint8_t i2c_slv_len : 4;
    } packed;

    struct I2C_SLV4_ADDR {
        bool i2c_slv4_rw : 1;
        uint8_t i2c_slv4_addr : 7;
    } packed;

    struct I2C_SLV4_REG {
        uint8_t i2c_slv4_reg : 8;
    } packed;

    struct I2C_SLV4_DO {
        uint8_t i2c_slv4_do : 8;
    } packed;

    struct I2C_SLV4_CTRL {
        bool i2c_slv4_en : 1;
        bool i2c_slv4_int_en : 1;
        bool i2c_slv4_reg_dis : 1;
        uint8_t i2c_mst_dly : 5;
    } packed;

    struct I2C_SLV4_DI {
        uint8_t i2c_slv4_di : 8;
    } packed;

    struct I2C_MST_STATUS {
        bool pass_through : 1;
        bool i2c_slv4_done : 1;
        bool i2c_lost_arb : 1;
        bool i2c_slv4_nack : 1;
        bool i2c_slv3_nack : 1;
        bool i2c_slv2_nack : 1;
        bool i2c_slv1_nack : 1;
        bool i2c_slv0_nack : 1;
    } packed;

    struct INT_PIN_CFG {
        bool int_level : 1;
        bool int_open : 1;
        bool latch_int_en : 1;
        bool int_rd_clear : 1;
        bool fsync_int_level : 1;
        bool fsync_int_en : 1;
        bool i2c_bypass_en : 1;
        bool clock_output : 1;
    } packed;

    struct INT_ENABLE {
        bool ff_en : 1;
        bool mot_en : 1;
        bool zmot_en : 1;
        bool fifo_oflow_en : 1;
        bool i2c_mst_en : 1;
        bool pll_rdy_int_en : 1;
        bool dmp_int_en : 1;
        bool raw_rdy_int_en : 1;
    } packed;

    struct DMP_INT_STATUS {
        uint8_t : 2;
        bool dmp_int_5 : 1;
        bool dmp_int_4 : 1;
        bool dmp_int_3 : 1;
        bool dmp_int_2 : 1;
        bool dmp_int_1 : 1;
        bool dmp_int_0 : 1;
    } packed;

    struct TC {
        bool pwr_mode : 1;
        uint8_t offset : 6;
        bool otp_bnk_vld : 1;
    } packed;

    struct INT_STATUS {
        bool ff_int : 1;
        bool mot_int : 1;
        bool zmot_int : 1;
        bool fifo_oflow_int : 1;
        bool i2c_mst_int : 1;
        bool pll_rdy_int : 1;
        bool dmp_int : 1;
        bool raw_rdy_int : 1;
    } packed;

    struct ACCEL_XOUT {
        uint8_t accel_xout_h : 8;
        uint8_t accel_xout_l : 8;
    } packed;

    struct ACCEL_YOUT {
        uint8_t accel_yout_h : 8;
        uint8_t accel_yout_l : 8;
    } packed;

    struct ACCEL_ZOUT {
        uint8_t accel_zout_h : 8;
        uint8_t accel_zout_l : 8;
    } packed;

    struct ACCEL_OUT {
        ACCEL_XOUT accel_xout;
        ACCEL_YOUT accel_yout;
        ACCEL_ZOUT accel_zout;
    } packed;

    struct TEMP_OUT {
        uint8_t temp_out_h : 8;
        uint8_t temp_out_l : 8;
    } packed;

    struct GYRO_XOUT {
        uint8_t gyro_xout_h : 8;
        uint8_t gyro_xout_l : 8;
    } packed;

    struct GYRO_YOUT {
        uint8_t gyro_yout_h : 8;
        uint8_t gyro_yout_l : 8;
    } packed;

    struct GYRO_ZOUT {
        uint8_t gyro_zout_h : 8;
        uint8_t gyro_zout_l : 8;
    } packed;

    struct GYRO_OUT {
        GYRO_XOUT gyro_xout;
        GYRO_YOUT gyro_yout;
        GYRO_ZOUT gyro_zout;
    } packed;

    struct EXT_SENS_DATA {
        uint8_t ext_sens_data : 8;
    } packed;

    struct MOT_DETECT_STATUS {
        bool mot_xneg : 1;
        bool mot_xpos : 1;
        bool mot_yneg : 1;
        bool mot_ypos : 1;
        bool mot_zneg : 1;
        bool mot_zpos : 1;
        bool : 1;
        bool mot_zrmot : 1;
    } packed;

    struct I2C_SLV_DO {
        uint8_t i2c_slv_do : 8;
    } packed;

    struct I2C_MST_DELAY_CTRL {
        bool delay_es_shadow : 1;
        uint8_t : 2;
        bool i2c_slv4_dly_en : 1;
        bool i2c_slv3_dly_en : 1;
        bool i2c_slv2_dly_en : 1;
        bool i2c_slv1_dly_en : 1;
        bool i2c_slv0_dly_en : 1;
    } packed;

    struct SIGNAL_PATH_RESET {
        uint8_t : 5;
        bool gyro_reset : 1;
        bool accel_reset : 1;
        bool temp_reset : 1;
    } packed;

    struct MOT_DETECT_CTRL {
        uint8_t : 2;
        uint8_t accel_on_delay : 2;
        uint8_t ff_count : 2;
        uint8_t mot_count : 2;
    } packed;

    struct USER_CTRL {
        bool dmp_en : 1;
        bool fifo_en : 1;
        bool i2c_mst_en : 1;
        bool i2c_if_dis : 1;
        bool dmp_reset : 1;
        bool fifo_reset : 1;
        bool i2c_mst_reset : 1;
        bool sig_cond_reset : 1;
    } packed;

    struct PWR_MGMT_1 {
        bool device_reset : 1;
        bool sleep : 1;
        bool cycle : 1;
        bool : 1;
        bool temp_dis : 1;
        uint8_t clksel : 3;
    } packed;

    struct PWR_MGMT_2 {
        bool lp_wake_ctrl : 1;
        bool : 1;
        bool stby_xa : 1;
        bool stby_ya : 1;
        bool stby_za : 1;
        bool stby_xg : 1;
        bool stby_yg : 1;
        bool stby_yz : 1;
    } packed;

    struct BANK_SEL {
        bool : 1;
        bool prftch_en : 1;
        bool cfg_user_bank : 1;
        uint8_t mem_sel : 5;
    } packed;

    struct MEM_START_ADDR {
        uint8_t start_addr : 8;
    } packed;

    struct MEM_R_W {
        uint8_t mem_r_w : 8;
    } packed;

    struct DMP_CFG_1 {
        uint8_t dmp_cfg_1 : 8;
    } packed;

    struct DMP_CFG_2 {
        uint8_t dmp_cfg_2 : 8;
    } packed;

    struct FIFO_COUNT {
        uint8_t fifo_count_h : 8;
        uint8_t fifo_count_l : 8;
    } packed;

    struct FIFO_R_W {
        uint8_t fifo_r_w : 8;
    } packed;

    struct WHO_AM_I {
        bool : 1;
        uint8_t who_am_i : 6;
        bool : 1;
    } packed;

}; // namespace MPU6050

#endif // MPU6050_BITFIELDS_REGISTERS_HPP