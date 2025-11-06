//
// Created by fish on 2025/10/10.
//

#pragma once
#include <cstdint>

#include "bsp/can.h"

namespace motor {
    /*
     *  DJI Motor Driver
     *  GM6020:
     *      Feedback id: 0x205 - 0x20B
     *      Control id: 0x1FF/0x2FF (VOLTAGE), 0x1FE/0x2FE (CURRENT)
     *      VOLTAGE Range: -25000 ~ 25000
     *      CURRENT Range: -16384 ~ 16384
     *      Angle Range: 0 ~ 8191
     *      Speed Unit: rpm
     *  M3508 (C620):
     *      Feedback id: 0x201 - 0x208
     *      Control id: 0x200/0x1FF (CURRENT)
     *      CURRENT Range: -16384 ~ 16384 (-20A ~ 20A)
     *      Angle Range: 0 ~ 8191
     *      Speed Unit: rpm
     */
#define DJI_MOTOR_LIMIT 8
    class DJI {
    public:
        DJI() = delete;
        ~DJI() = default;

        enum control_mode_e {
            VOLTAGE, CURRENT
        };

        enum model_e {
            GM6020, M3508, M2006
        };

        struct param_t {
            uint8_t id;
            bsp_can_e port;
            control_mode_e mode;
        };

        struct feedback_t {
            // 电机原始反馈值
            struct {
                int16_t angle, speed, current;
                uint8_t temp;
            } raw;
            // 输出轴总圈数
            int round;
            // 输出轴单圈角度 (rad, [0, 2\pi))
            float angle;
            // 输出轴角速度 (rad/s)
            float speed;
            // 相电流 (A)
            float current;
            // 转矩 (Nm)
            float torque;
            uint32_t timestamp;
        };

        DJI(const char *name, const model_e &model, const param_t &param);
        DJI(const char *name, const model_e &model, const param_t &param, float ratio);

        static void decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len);

        void init();
        void clear();

        void enable() { this->enabled = true; }
        void disable() { clear(); this->enabled = false; }
        void update(float val);
        // void update_torque(float torque);

        float ratio = 0;
        char name[16] = { };
        bool enabled = false;
        feedback_t feedback = feedback_t();
        uint16_t ctrl_id = 0, feedback_id = 0;
    private:
        model_e model {};
        param_t param {};
        int16_t output = 0, lst_angle = 0;
    };
}
