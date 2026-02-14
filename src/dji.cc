#include "motor/dji.h"

#include "bsp/can.h"
#include "bsp/time.h"
#include "utils/logger.h"

#include "task.h"

#include <cstring>
#include <algorithm>

#include "cmsis_os2.h"

using namespace motor;

#define TASK_STACK_SIZE 512

// 电机默认减速比
#define GM6020_DEFAULT_RATIO 1
#define M3508_DEFAULT_RATIO (3591.f/187.f)
#define M2006_DEFAULT_RATIO (36.f/1.f)

// 电机默认转矩常数（24V下）
#define GM6020_TORQUE_CONSTANT 0.741f
#define M3508_TORQUE_CONSTANT 0.3f
#define M2006_TORQUE_CONSTANT 0.18f

// 映射和限幅
#define GM6020_VOLTAGE_LIMIT 25000.f
#define GM6020_CURRENT_LIMIT 16384.f
#define M3508_CURRENT_LIMIT 16384.f
#define M2006_CURRENT_LIMIT 10000.f
#define GM6020_CURRENT_LIMIT_REAL 3.f
#define M3508_CURRENT_LIMIT_REAL 20.f
#define M2006_CURRENT_LIMIT_REAL 10.f

// 电机功率默认拟合系数
#define GM6020_DEFAULT_POWER_K_0 0.5807780373237382f
#define GM6020_DEFAULT_POWER_K_1 0.000495794444214091f
#define GM6020_DEFAULT_POWER_K_2 27.17379907809995f
#define GM6020_DEFAULT_POWER_A 0.9334894382537696f

#define M3508_DEFAULT_POWER_K_0 1.1532294690949845f
#define M3508_DEFAULT_POWER_K_1 0.00003165414968530437f
#define M3508_DEFAULT_POWER_K_2 683.7802569066574f
#define M3508_DEFAULT_POWER_A 0.8267041157447231f

#define M2006_DEFAULT_POWER_K_0 0.f
#define M2006_DEFAULT_POWER_K_1 0.f
#define M2006_DEFAULT_POWER_K_2 0.f
#define M2006_DEFAULT_POWER_A 0.f

const float fpi = M_PI;

// FreeRTOS Task
static bool inited = false;
[[noreturn]] static void task(void *args);
static TaskHandle_t task_handle = nullptr;

// Device Class Ptr & Device Cnt (以 bsp_can_e 分类，保证一条 can 总线上的设备 id 不冲突)
#define ID_COUNT 5
static constexpr uint16_t ctrl_id_map[] = { 0x2ff, 0x1ff, 0x2fe, 0x1fe, 0x200 };
static uint8_t id_trans(uint16_t x) {
    for(uint8_t i = 0; i < ID_COUNT; i++) if(ctrl_id_map[i] == x) return i;
    BSP_ASSERT(false); return 0;
}
static dji* device_ptr[BSP_CAN_DEVICE_COUNT][DJI_MOTOR_LIMIT];
static uint8_t device_cnt[BSP_CAN_DEVICE_COUNT];
static bool ctrl_id_used[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1];
static uint8_t can_tx_buf[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1][8];

// 带过零的计算角度差，好用
static float calc_delta(float full, float current, float target) {
    float dt = target - current;
    if(2 * dt >  full) dt -= full;
    if(2 * dt < -full) dt += full;
    return dt;
}

dji::dji(const char *name, const model_e &model, const param_t &param) :
dji(name, model, param, model == GM6020 ? GM6020_DEFAULT_RATIO : model == M3508 ? M3508_DEFAULT_RATIO : M2006_DEFAULT_RATIO) {}

dji::dji(const char *name, const model_e &model, const param_t &param, float ratio) :
dji(name, model, param, ratio, {.k0 = 0, .k1 = 0, .k2 = 0, .a = 0}) {}

dji::dji(const char *name, const model_e &model, const param_t &param, float ratio, const power_param_t &power_param) : ratio(ratio), power_param(power_param), model(model), param(param) {
    BSP_ASSERT(ratio > 0.f);
    strcpy(this->name, name);

    switch (model) {
        case GM6020: {
            BSP_ASSERT(1 <= param.id and param.id <= 7);
            if (param.mode == VOLTAGE) {
                ctrl_id = param.id < 5 ? 0x1ff : 0x2ff;
            }
            if (param.mode == CURRENT) {
                ctrl_id = param.id < 5 ? 0x1fe : 0x2fe;
            }
            feedback_id = 0x204 + param.id;
            break;
        }
        case M3508: {
            BSP_ASSERT(1 <= param.id and param.id <= 8);
            BSP_ASSERT(param.mode == CURRENT);
            ctrl_id = param.id < 5 ? 0x200 : 0x1ff;
            feedback_id = 0x200 + param.id;
            break;
        }
        case M2006: {
            BSP_ASSERT(1 <= param.id and param.id <= 8);
            BSP_ASSERT(param.mode == CURRENT);
            ctrl_id = param.id < 5 ? 0x200 : 0x1ff;
            feedback_id = 0x200 + param.id;
            break;
        }
    }

    device_ptr[param.port][device_cnt[param.port] ++] = this;
    ctrl_id_used[param.port][id_trans(ctrl_id)] = true;
}

void dji::update(float val) {
    if (!enabled) return;
    switch (model) {
        case GM6020: {
            if (param.mode == VOLTAGE)
                output = static_cast<int16_t>(
                    std::clamp(val, -GM6020_VOLTAGE_LIMIT, GM6020_VOLTAGE_LIMIT)
                );
            else
                output = static_cast<int16_t>(
                    std::clamp(val, -GM6020_CURRENT_LIMIT, GM6020_CURRENT_LIMIT)
                );
            break;
        }
        case M3508: {
            output = static_cast<int16_t>(
                std::clamp(val, -M3508_CURRENT_LIMIT, M3508_CURRENT_LIMIT)
            );
            break;
        }
        case M2006: {
            output = static_cast<int16_t>(
                std::clamp(val, -M2006_CURRENT_LIMIT, M2006_CURRENT_LIMIT)
            );
            break;
        }
    }
    uint8_t cid = id_trans(ctrl_id), mid = param.id < 5 ? param.id : param.id - 4;
    can_tx_buf[param.port][cid][(mid - 1) << 1] = output >> 8;
    can_tx_buf[param.port][cid][(mid - 1) << 1 | 1] = output & 0xff;
}

void dji::clear() {
    update(0);
    memset(&feedback, 0, sizeof feedback);
}

static float calc_power(float k0, float k1, float k2, float a, float torque, float omega) {
    // torque: 减速箱前扭矩 (Nm) | omega: 减速箱前角速度 (rad/s)
    return k0 * torque * omega + k1 * omega * omega + k2 * torque * torque + a;
}

void dji::decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len) {
    if (!device_cnt[device] or len != 8) return;

    dji *p = nullptr;
    for(uint8_t i = 0; i < device_cnt[device]; i++) {
        if(device_ptr[device][i]->feedback_id == id) {
            p = device_ptr[device][i];
            break;
        }
    }
    if(p == nullptr or !p->enabled) return;

    auto &fb = p->feedback;

    fb.raw.angle = static_cast<int16_t>(data[0] << 8 | data[1]);
    fb.raw.speed = static_cast<int16_t>(data[2] << 8 | data[3]);
    fb.raw.current = static_cast<int16_t>(data[4] << 8 | data[5]);
    fb.raw.temp = data[6];

    // angle - rad
    fb.angle += calc_delta(8192, p->lst_angle, fb.raw.angle) / 4096.f * fpi / p->ratio;
    if (fb.angle < 0) fb.angle += 2 * fpi, fb.round --;
    if (fb.angle >= 2 * fpi) fb.angle -= 2 * fpi, fb.round ++;
    p->lst_angle = fb.raw.angle;
    // speed - rad/s
    fb.speed = static_cast<float>(fb.raw.speed) / 30.f * static_cast<float>(M_PI) / p->ratio;
    // current - A, torque - Nm
    switch (p->model) {
        case GM6020:
            fb.current = static_cast<float>(fb.raw.current) / GM6020_CURRENT_LIMIT * GM6020_CURRENT_LIMIT_REAL;
            fb.torque = fb.current * p->ratio / GM6020_DEFAULT_RATIO * GM6020_TORQUE_CONSTANT;
            break;
        case M3508:
            fb.current = static_cast<float>(fb.raw.current) / M3508_CURRENT_LIMIT * M3508_CURRENT_LIMIT_REAL;
            fb.torque = fb.current * p->ratio / M3508_DEFAULT_RATIO * M3508_TORQUE_CONSTANT;
            break;
        case M2006:
            fb.current = static_cast<float>(fb.raw.current) / M2006_CURRENT_LIMIT * M2006_CURRENT_LIMIT_REAL;
            fb.torque = fb.current * p->ratio / M2006_DEFAULT_RATIO * M2006_TORQUE_CONSTANT;
            break;
    }
    // power - W
    if (p->power_param.a == 0) {
        if (p->model == GM6020)
            fb.power = calc_power(GM6020_DEFAULT_POWER_K_0, GM6020_DEFAULT_POWER_K_1, GM6020_DEFAULT_POWER_K_2, GM6020_DEFAULT_POWER_A, fb.torque / p->ratio, fb.speed * p->ratio);
        if (p->model == M3508)
            fb.power = calc_power(M3508_DEFAULT_POWER_K_0, M3508_DEFAULT_POWER_K_1, M3508_DEFAULT_POWER_K_2, M3508_DEFAULT_POWER_A, fb.torque / p->ratio, fb.speed * p->ratio);
        if (p->model == M2006)
            fb.power = calc_power(M2006_DEFAULT_POWER_K_0, M2006_DEFAULT_POWER_K_1, M2006_DEFAULT_POWER_K_2, M2006_DEFAULT_POWER_A, fb.torque / p->ratio, fb.speed * p->ratio);
    } else {
        fb.power = calc_power(p->power_param.k0, p->power_param.k1, p->power_param.k2, p->power_param.a, fb.torque / p->ratio, fb.speed * p->ratio);
    }

    fb.timestamp = bsp_time_get_ms();
}

void dji::init() {
    logger::info("motor '%s' inited", name);
    if (!inited) {
        xTaskCreate(
            task,
            "motor::dji",
            TASK_STACK_SIZE,
            nullptr,
            osPriorityHigh,
            &task_handle
        );
        inited = true;
    }
    bsp_can_set_callback(param.port, feedback_id, decoder);
    enable();
}

static void task(void *args) {
    logger::info("module inited");
    for (;;) {
        for(uint8_t i = 0; i < BSP_CAN_DEVICE_COUNT; i++) {
            if(!device_cnt[i]) continue;
            for(uint8_t j = 0; j < ID_COUNT; j++) {
                if(ctrl_id_used[i][j]) {
                    bsp_can_send(static_cast<bsp_can_e>(i), ctrl_id_map[j], can_tx_buf[i][j], 8);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
