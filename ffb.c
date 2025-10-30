/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
・TinyUSB使用
・三相ハイサイドシグナル生成（三角波カウンタ、irq割り込みでwrap更新）
・UARTでエンコーダーにコマンド送信、直後にRXに来たデータを処理。
*/
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "bsp/board.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
// 設定：使用GPIOピンと周波数
#define PIN_U 16
#define PIN_V 18
#define PIN_W 17
#define DE_PIN 10            // MAX485 DE
#define RE_PIN 6             // MAX485 RE
#define UART_TX_PIN_uart0 12 // TXピン
#define UART_RX_PIN_uart0 13 // RXピン
#define UART_TX_PIN_uart1 8  // TXピン
#define UART_RX_PIN_uart1 9  // RXピン
#define LED_FFB_ACTIVE 4     // FFBアクティブLED
#define LED_FFB_MAGNITUDE 24 // FFBマグニチュードLED
// 三相PWM設定
#define CARRIER_FREQ_HZ 25000.0f // 25kHz キャリア
#define SINE_FREQ_HZ 0.0f        // 変調周波数
#define PHASE_MAX (1U << 31)
#define PHASE_RSL (1U << 11)
// UART設定
#define UART_ID_1 uart1
#define UART_BAUD_1 2500000

#define WHEEL_ANGLE 180 // degree
#define REDUCTION_RATIO 4

/*--------------共有変数------------*/
volatile int32_t angle_share = 0;     // エンコーダーで読み取った角度
volatile int32_t magnitude_share = 0; // FFB指令値
volatile int8_t rotateNum_share = 0;  // モーターの原点からの回転回数
volatile int8_t limitRot_share = 0;   // 回転制限 0 or 1
spin_lock_t *lock;
/*----------------------------------*/

volatile int16_t ffb_magnitude = 0;
volatile bool ffb_active = false;
volatile uint8_t current_block_idx = 4;
volatile bool effect_playing = false;
volatile int32_t angle_core0 = 0;
volatile int8_t rotateNum_core0 = 0;
volatile int8_t limitRot_core0 = 0;

/*-------------only use in CORE1-------------*/
// 位相変数（rad単位）、3相分はオフセットで扱う
volatile uint32_t phase = 0;
const uint32_t phase_offset = 57445188; // 9.63deg angleオフセット2090041344
uint32_t delta_phase;                   // 1キャリア周期ごとの位相ステップ
// PWM wrap 値（TOP）
uint32_t wrap_val;
// 各スライス番号・チャネル
uint slice_u, slice_v, slice_w;
uint chan_u, chan_v, chan_w;
float MR = 0.0f; // 変調率
float torque_max = 0.10f;
float sinValues[2 * PHASE_RSL];
float rdmValues[2 * PHASE_RSL];
int rotate_mode = 1;
uint8_t enc_rx_buffer[12];
uint8_t error = 0;
volatile int32_t angle = 0, pre_angle = 0; // 17bit treated as 360deg
volatile uint8_t direction_cmd = 0;        // 0:stop, 1:+, 2:-

volatile int8_t rotateNum_core1 = 0;
volatile int8_t limitRot_core1 = 0;
const uint32_t angle_offset = 7420;
volatile int32_t elec_angle = 0;
/*---------------------------------------------*/

// wrap IRQ ハンドラ（例としてU相のラップIRQを使い、3相同時更新）
void pwm_wrap_irq_handler()
{
    // IRQ フラグクリア（U相スライス）
    pwm_clear_irq(slice_u);

    // 位相更新
    phase = phase & (PHASE_MAX - 1);

    // 3相の振幅計算（浮動小数点 sinf)
    float su = 0, sv = 0, sw = 0;
    if (rotate_mode == 1)
    {
        // U相: θ
        su = sinValues[phase >> 20];
        // V相: θ + 120° = θ + 4π/3
        sv = sinValues[(phase >> 20) + 683 /*→(2^11 * 1/3)*/];
        // W相: θ + 240° = θ + 2π/3
        sw = sinValues[(phase >> 20) + 1365 /*→(2^11 * 2/3)*/];
    }
    else if (rotate_mode == -1)
    {
        // U相: θ
        su = sinValues[phase >> 20];
        // V相: θ + 240° = θ + 2π/3
        sv = sinValues[(phase >> 20) + 1365 /*→(2^11 * 2/3)*/];
        // W相: θ + 120° = θ + 4π/3
        sw = sinValues[(phase >> 20) + 683 /*→(2^11 * 1/3)*/];
    }
    // デューティ（0 ～ wrap_val）の計算: (sin*0.5 + 0.5) を乗算
    uint32_t du = (uint32_t)(MR * (su * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[phase >> 20]
    uint32_t dv = (uint32_t)(MR * (sv * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[(phase >> 20) + 683]
    uint32_t dw = (uint32_t)(MR * (sw * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[(phase >> 20) + 1365]

    // 比較レジスタ更新：次周期から反映
    pwm_set_chan_level(slice_u, chan_u, du);
    pwm_set_chan_level(slice_v, chan_v, dv);
    pwm_set_chan_level(slice_w, chan_w, dw);
}

void pwm_init_set()
{
    // PWM GPIO 設定
    gpio_set_function(PIN_U, GPIO_FUNC_PWM);
    gpio_set_function(PIN_V, GPIO_FUNC_PWM);
    gpio_set_function(PIN_W, GPIO_FUNC_PWM);
    slice_u = pwm_gpio_to_slice_num(PIN_U);
    chan_u = pwm_gpio_to_channel(PIN_U);
    slice_v = pwm_gpio_to_slice_num(PIN_V);
    chan_v = pwm_gpio_to_channel(PIN_V);
    slice_w = pwm_gpio_to_slice_num(PIN_W);
    chan_w = pwm_gpio_to_channel(PIN_W);

    // sysclk 周波数取得
    uint sysclk_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000;

    // clkdiv 設定（ここでは 1.0f、必要に応じ変更可）
    pwm_set_clkdiv(slice_u, 1.0f);
    pwm_set_clkdiv(slice_v, 1.0f);
    pwm_set_clkdiv(slice_w, 1.0f);

    // wrap (TOP) 計算: sysclk / キャリア周波数x2 - 1 (三角波キャリアにしたため)
    wrap_val = (uint32_t)(sysclk_hz / (CARRIER_FREQ_HZ * 2.0f)) - 1;
    pwm_set_wrap(slice_u, wrap_val);
    pwm_set_wrap(slice_v, wrap_val);
    pwm_set_wrap(slice_w, wrap_val);

    pwm_set_phase_correct(slice_u, true);
    pwm_set_phase_correct(slice_v, true);
    pwm_set_phase_correct(slice_w, true);

    // 初期デューティ: 0（出力OFF相当）
    pwm_set_chan_level(slice_u, chan_u, 0);
    pwm_set_chan_level(slice_v, chan_v, 0);
    pwm_set_chan_level(slice_w, chan_w, 0);

    // pwm_set_output_polarity(slice_u, false, true);
    // pwm_set_output_polarity(slice_v, false, true);
    // pwm_set_output_polarity(slice_w, false, true);

    // 正弦位相ステップ Δθ = 2π * SINE_FREQ_HZ / CARRIER_FREQ_HZ
    // delta_phase = (float)PHASE_MAX * (float)SINE_FREQ_HZ / (float)CARRIER_FREQ_HZ;

    // wrap IRQ を U相スライスで有効化
    pwm_clear_irq(slice_u);
    pwm_set_irq_enabled(slice_u, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // PWM 有効化（フリーラン開始）
    pwm_set_enabled(slice_u, true);
    pwm_set_enabled(slice_v, true);
    pwm_set_enabled(slice_w, true);
}

int convert_angle(int8_t rotateNum, int angle_17bit)
{ // {(17bit int) and (multi rotate data)} into 16bit int
    int angle_conv;
    if (rotateNum == 0)
    {
        angle_conv = -32767 * (float)(360.f * ((float)angle_17bit / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }
    else if (rotateNum <= -1)
    {
        angle_conv = -32767 * (float)(360.f * ((float)(angle_17bit + (65535 + 65535) * abs(rotateNum)) / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }
    else if (rotateNum >= 1)
    {
        angle_conv = -32767 * (float)(360.f * ((float)(angle_17bit - (65535 + 65535) * abs(rotateNum)) / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }

    if (angle_conv > 32767)
    {
        angle_conv = 32767, limitRot_core0 = 1;
    }
    else if (angle_conv < -32767)
    {
        angle_conv = -32767, limitRot_core0 = -1;
    }
    else
    {
        limitRot_core0 = 0;
    }
    return angle_conv;
}

typedef struct
{
    int16_t x;
    int16_t y;
} __attribute__((packed)) hid_report_t;

void send_hid_report()
{
    static hid_report_t report = {0};
    report.x = -convert_angle(rotateNum_core0, angle_core0);
    report.y = 0;

    if (tud_hid_ready())
    {
        tud_hid_report(/*report_id*/ 0x01, &report, sizeof(report));
    }
}

// UART function
void uart1_send(uint8_t cmd)
{
    // 送信モードに切り替え
    gpio_put(RE_PIN, 1); // 受信無効
    gpio_put(DE_PIN, 1); // 送信有効
    //  送信前にRX FIFOをクリアしておく（古いデータ・ノイズ排除）
    while (uart_is_readable(UART_ID_1))
    {
        (void)uart_getc(UART_ID_1);
    }
    // UART送信
    // uart_write_blocking は内部でFIFOに書き込み、そのまま戻る
    uart_write_blocking(UART_ID_1, &cmd, 1);

    // 送信完了待ち: 1バイト/2.5Mbps ≈ 3.2μs。確実に送出完了させるため、数十μs待機
    // UARTハードウェアレジスタで直にBUSYを確認する方法もあるが、簡易的にsleepを入れる
    sleep_us(3);
    // 受信モードに戻す
    gpio_put(RE_PIN, 0);
    gpio_put(DE_PIN, 0);
}
// エンコーダー受信用
void uart1_receive()
{
    uint8_t const expected_bytes = 12;
    int idx = 0;
    uint64_t start = get_absolute_time();
    uint8_t pre_buffer[12] = {0};
    for (int i = 0; i < expected_bytes; i++)
    {
        pre_buffer[i] = enc_rx_buffer[i];
    }

    while (absolute_time_diff_us(start, get_absolute_time()) < 200)
    { // 受信タイムアウト
        while (uart_is_readable(UART_ID_1) && idx < expected_bytes)
        {
            enc_rx_buffer[idx++] = uart_getc(UART_ID_1);
        }
        if (idx >= expected_bytes)
            break;
    }
    if (idx < expected_bytes)
    {
        // printf("%d\n",idx);
        for (int i = 0; i < expected_bytes; i++)
        {
            enc_rx_buffer[i] = pre_buffer[i];
        }
    }
}
// エンコーダーデコード
int decode_encoder_angle(uint8_t abs0, uint8_t abs1, uint8_t abs2)
{ // 17bitデータの復号
    int32_t pos = ((abs2 & 0x7F) << 16) + (abs1 << 8) + abs0 - angle_offset;
    if (pos > 131072 / 2)
    {
        pos -= 131072;
    }
    if (pos < -131072 / 2)
    {
        pos += 131072;
    }
    // printf("%d\n",pos);
    return pos;
}

void uart_init_set()
{
    // UART1初期化
    uart_init(UART_ID_1, UART_BAUD_1);
    gpio_set_function(UART_TX_PIN_uart1, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_uart1, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID_1, true);
    // DE/REピン設定
    gpio_init(DE_PIN);
    gpio_set_dir(DE_PIN, GPIO_OUT);
    gpio_put(DE_PIN, 0);
    gpio_init(RE_PIN);
    gpio_set_dir(RE_PIN, GPIO_OUT);
    gpio_put(RE_PIN, 0);
    // リセットコマンド
    uart1_send(0xBA);
    sleep_ms(1);
    uart1_send(0xC2);
    sleep_ms(1);
    uart1_send(0x62);
    sleep_ms(1);
    uart1_send(0xEA);
    sleep_ms(1);
}

// core_1 モーター制御関係
void core1_main()
{
    // モーター制御用数値設定
    phase = phase_offset;
    angle = angle_offset;
    int pre_elec = 0;
    float torque = 0.0f;
    int32_t local_magnitude = 0, pre_local_magnitude = 0;
    float Kd = 0.0f, alpha = 0.001f, beta = 0.01f;
    int delta_angle = 0, pre_delta_angle = 0, a = 0;

    pwm_init_set();

    while (true)
    {
        uart1_send(0x1A); // エンコーダーコマンド送信
        sleep_us(100);
        uart1_receive(); // エンコーダーデータ受信

        pre_angle = angle;
        pre_delta_angle = delta_angle;
        pre_local_magnitude = local_magnitude;

        angle = decode_encoder_angle(enc_rx_buffer[3], enc_rx_buffer[4], enc_rx_buffer[5]); // -65535~65535
        delta_angle = angle - pre_angle;
        if (delta_angle > 65535)
        {
            rotateNum_core1++;
            delta_angle = 0;
        }
        else if (delta_angle < -65535)
        {
            rotateNum_core1--;
            delta_angle = 0;
        }

        // 排他制御して読み込み
        uint32_t irq = save_and_disable_interrupts();
        spin_lock_unsafe_blocking(lock);
        local_magnitude = magnitude_share; // fetch
        limitRot_core1 = limitRot_share;   // fetch
        angle_share = angle;               // pass
        rotateNum_share = rotateNum_core1; // pass
        spin_unlock_unsafe(lock);
        restore_interrupts(irq);

        // 電気角算出
        if (angle >= 0 && angle < 32768)
        {
            elec_angle = angle;
        }
        else if (angle >= 32768 && angle < 65536)
        {
            elec_angle = angle - 32768;
        }
        else if (angle >= -32768 && angle < 0)
        {
            elec_angle = 32768 + angle;
        }
        else if (angle >= -65536 && angle < -32768)
        {
            elec_angle = 65536 + angle;
        }
        else
        {
            error = 1;
        }

        pre_elec = elec_angle;

        // 回転限界処理、粘性抵抗
        delta_angle = pre_delta_angle * (1.0f - alpha) + delta_angle * alpha; // ωフィルタ
        local_magnitude = pre_local_magnitude * (1.0f - beta) + local_magnitude * beta;
        torque = -((float)delta_angle / 65536.f * Kd) + (torque_max * (float)local_magnitude / 10000.f);
        // if(rotateNum_core1!=0){MR = 0.07f;}else{MR = fabs(torque);} // (0.10f * fabs(angle) / 65536.f) +
        MR = fabs(torque);
        if (limitRot_core1 > 0)
        {
            a = -8192;
        }
        else if (limitRot_core1 < 0)
        {
            a = 8192;
        }
        else
        {
            a = 0;
            if (torque > 0)
            {
                a = -8192;
            }
            else if (torque < 0)
            {
                a = 8192;
            }
        }

        elec_angle = (elec_angle + a); // 8192 = 90deg

        // 位相計算
        if (elec_angle < 0)
        {
            elec_angle = 32768 + elec_angle;
        }
        elec_angle = elec_angle & ((1 << 15) - 1);
        phase = phase_offset + (float)(elec_angle) / (float)(1 << 15) * (float)PHASE_MAX;
    }
}
// core_0 USBデータ送信・メインループ
int main()
{
    stdio_init_all();
    // 三相基本波配列計算
    for (int x = 0; x < 2 * PHASE_RSL; x++)
    {
        float s = (float)(2.0f * M_PI * (float)x) / (float)PHASE_RSL;
        sinValues[x] = sin(s);
        // printf(x + "\n");
    }
    for (int x = 0; x < 2 * PHASE_RSL; x++)
    {
        rdmValues[x] = (float)(rand() % 100 - 50) / 1000.0f;
    }

    board_init();
    tusb_init();
    uart_init_set();

    lock = spin_lock_instance(0);
    multicore_reset_core1();
    sleep_ms(100);
    multicore_launch_core1(core1_main);

    gpio_init(LED_FFB_ACTIVE);
    gpio_init(LED_FFB_MAGNITUDE);
    gpio_set_dir(LED_FFB_ACTIVE, GPIO_OUT);
    gpio_set_dir(LED_FFB_MAGNITUDE, GPIO_OUT);

    while (true)
    {
        tud_task(); // USBタスク処理

        // 排他制御して読み込み
        uint32_t irq = save_and_disable_interrupts();
        spin_lock_unsafe_blocking(lock);
        magnitude_share = -ffb_magnitude;
        limitRot_share = limitRot_core0;
        angle_core0 = angle_share;
        rotateNum_core0 = rotateNum_share;
        spin_unlock_unsafe(lock);
        restore_interrupts(irq);

        send_hid_report(); // ゲームパッド状態送信
        sleep_ms(5);       // 約200Hzで送信
        gpio_put(LED_FFB_ACTIVE, ffb_active);
        gpio_put(LED_FFB_MAGNITUDE, ffb_magnitude != 0);
        if (!ffb_active)
        {
            ffb_magnitude = 0;
        }
    }
}

typedef struct __attribute__((packed))
{
    uint8_t id; /* 0x01 */
    uint8_t block_idx;
    uint8_t effect_type;
    uint16_t duration_ms;
    uint16_t trig_rep_int_ms;
    uint16_t sample_prd_ms;
    uint8_t gain;
    uint8_t trig_button;
    uint8_t axes_enable; /* bit0=X, bit1=Y */
    uint8_t dir_enable;  /* bit0=1 */
    uint8_t direction_x; /* 0‑255 (角度) */
    uint8_t direction_y;
    uint16_t ts_offset1;
    uint16_t ts_offset2;
} __attribute__((packed)) set_effect_rpt_t;
/* -------- PID State / Block‑Load (ID 0x02) -------- */
typedef struct __attribute__((packed))
{
    // uint8_t  id;              // = 0x02
    uint8_t effectBlockIndex; /* = 1 だけ使う */
    uint8_t loadStatus;       /* 0 = OK, 1 = FULL */
    uint8_t poolAvailableLo;  /* 0xFF → 32 kB 空き */
    uint8_t poolAvailableHi;
    uint8_t actuatorsEnabled; /* bit0 */
    uint8_t safetySwitch;     /* 0 固定 */
    uint8_t effectPlaying;    /* bit0 */
} __attribute__((packed)) pid_state_rpt_t;

/* -------- Block‑Load Status (ID 0x06) --------
 *  byte0: 0x06 (Report ID)
 *  byte1: effectBlockIndex (1 を固定で OK)
 *  byte2: blockLoadStatus  (1 = Success, 2 = Full, 3 = Error)
 *  byte3: ramPoolAvailableLo   (0xFF = 32 kB 空き)
 *  byte4: ramPoolAvailableHi   (0x7F)
 */
typedef struct __attribute__((packed))
{
    // uint8_t  id;          // = 0x05
    uint16_t effect_idx; // block index the FW just allocated
    uint8_t rom_flag;    // 0 = RAM
    uint8_t status;      // 0 = success
} pid_var_blockload_rpt_t;
typedef struct __attribute__((packed))
{
    // uint8_t id;              /* = 0x06 */
    uint8_t effectBlockIndex; /* 1 */
    uint8_t loadStatus;       /* 1 */
    uint8_t poolAvailLo;      /* 0xFF */
    uint8_t poolAvailHi;      /* 0x7F */
} pid_blockload_status_t;
typedef struct __attribute__((packed))
{
    // uint8_t  id;             // = 0x07
    uint16_t ram_pool_size; // 0x8000 → little‑endian: 00 80
    uint8_t effect_max;     // 8
    uint8_t caps;           // bit0=1, bit1=1 → 0x03
} pid_pool_rpt_t;           // sizeof = 5

// デバイスに関する情報
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t rid, hid_report_type_t type, uint8_t *buffer, uint16_t reqlen)
{
    if (type != HID_REPORT_TYPE_FEATURE)
        return 0;

    // if (rid == 0x02) { // && reqlen >= sizeof(pid_state_rpt_t)
    //     pid_state_rpt_t* r = (pid_state_rpt_t*)buffer;
    //     //r->id               = 0x02;
    //     r->effectBlockIndex = current_block_idx;           // Block ID
    //     r->loadStatus       = 0;           // 0 = Success
    //     r->poolAvailableLo  = 0xFF;        // 32 kB 空き
    //     r->poolAvailableHi  = 0x7F;
    //     r->actuatorsEnabled = effect_playing;
    //     r->safetySwitch     = 0;
    //     r->effectPlaying    = ffb_active;
    //     return 8;
    // }

    if (rid == 0x06 && reqlen >= 4)
    {
        pid_blockload_status_t rpt = {
            //.id               = 0x06,
            .effectBlockIndex = current_block_idx, // 前回の idx
            .loadStatus = 1,                       // (1=SUCCESS,2=FULL,3=ERROR…)
            .poolAvailLo = 0xFF,
            .poolAvailHi = 0x7F};
        memcpy(buffer, &rpt, sizeof(rpt));
        return sizeof(rpt) + 1;
    }

    if (rid == 0x07)
    {
        pid_pool_rpt_t rpt = {
            //.id            = 0x07,
            .ram_pool_size = 0x8000, /* 32 kB  */
            .effect_max = 8,         /* 同時 8 効果 */
            .caps = 0x03             /* bit0:DeviceManaged, bit1:SharedParams */
        };
        memcpy(buffer, &rpt, sizeof(rpt));
        return sizeof(rpt) + 1;
    }

    /* 未対応 → STALL (=0) */
    else
    {
        return 0;
    }
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t type, uint8_t const *buffer, uint16_t bufsize)
{
    // ここでPCからの出力（FFBなど）も受け取れる
    // if (type != HID_REPORT_TYPE_OUTPUT) return;

    switch (buffer[0])
    {
    case 0x01:
        if (bufsize > 18)
        {
            current_block_idx = buffer[1];
        }
        break;
    case 0x04:
        if (bufsize >= 3)
        {
            ffb_magnitude = (int16_t)(buffer[4] | (buffer[5] << 8)); // read offset value as magnitude
        }
        break;
    case 0x05:
        if (bufsize >= 2)
        {
            current_block_idx = buffer[1];
            uint8_t LSB = buffer[2];
            uint8_t MSB = buffer[3];
            ffb_magnitude = (int16_t)(buffer[2] | (buffer[3] << 8));
        }
        break;
    case 0x0A:
        if (bufsize < 3)
            break;
        uint8_t idx = buffer[1];
        uint8_t op = buffer[2]; // 3=All Stop 1=Start
        if (idx != current_block_idx)
            break; /* ×: 未知ハンドルなら無視 */
        ffb_active = (op == 1);
        break;
    default:
        break;
    }
    return;
}
