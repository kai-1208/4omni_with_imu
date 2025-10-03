#include "mbed.h"
#include "BNO055Uart.hpp"
#include "key.hpp"
#include "serial_read.hpp"

#define M_PI 3.14159265358979323846

BufferedSerial pc(USBTX, USBRX, 115200);
BNO055Uart imu(PA_9, PA_10);

serial_unit serial(pc);

float yaw_offset = 0.0f;        // ヨー角の初期オフセット
double relative_yaw = 0.0;      // IMUから取得した相対ヨー角

double Lx = 0.0, Ly = 0.0, Rx = 0.0;

CAN can(PA_11, PA_12, (int)1e6);

int speed_change = 100;

/**
 * @brief imuからヨー角を取得
 */
void imu_yaw_get() {
    if (imu.update()) {
        // オイラー角（yaw, pitch, roll）を取得
        BNO055Uart::EulerAngles angles = imu.getEuler();

        double current_yaw = angles.yaw - yaw_offset;

        // 角度が-180度から180度の範囲に収まるように正規化
        while (current_yaw > 180.0) {
            current_yaw -= 360.0;
        }
        while (current_yaw < -180.0) {
            current_yaw += 360.0;
        }

        relative_yaw = current_yaw;
    } else {
        printf("Failed to update sensor data.\n");
    }
}


// 以下のコメントアウトは、ps4controller_topic-communicationを使う場合のコード

// /**
//  * @brief ps4コントローラーの入力値を受信
//  */
// void read_until_pipe(char *output_buf, int output_buf_size) {
//     char buf[20];
//     int output_buf_index = 0;
//     while (1) {
//         if (pc.readable()) {
//             ssize_t num = pc.read(buf, sizeof(buf) - 1);
//             buf[num] = '\0';
//             for (int i = 0; i < num; i++) {
//                 if (buf[i] == '|') {
//                     output_buf[output_buf_index] = '\0';
//                     return;
//                 } else if (buf[i] != '\n' && output_buf_index < output_buf_size - 1) {
//                     output_buf[output_buf_index++] = buf[i];
//                 }
//             }
//         }
//         if (output_buf_index >= output_buf_size - 1) {
//             output_buf[output_buf_index] = '\0';
//             return;
//         }
//     }
// }

// /**
//  * @brief output_bufから必要なデータを取得
//  */
// void output_buf_get(char *buffer, double *output, const char *phrase) {
//     int phrase_len = strlen(phrase);
//     if (strncmp(buffer, phrase, phrase_len) == 0) {
//         char *data_pointer = buffer + phrase_len;
//         *output = atof(data_pointer);
//     }
// }

/**
 * @brief lx, ly, rxの値を取得
 */
void move_aa(std::string msg) {
    msg.erase(0, 2);
    std::vector<double> joys_d = to_numbers(msg);
    std::vector<float> joys(joys_d.begin(), joys_d.end());
    for (auto &joy : joys) {
        if (joy > -0.08 && joy < 0.08) {
            joy = 0.0;
        }
    }
    
    // joysの値をグローバル変数に代入
    if (joys.size() >= 3) {
        Lx = joys[0];
        Ly = joys[1];
        Rx = joys[2];
    }
}

/**
 * @brief 4輪オムニの出力計算およびcanで送信
 */
void omni_control(double lx, double ly, double rx, double yaw) {
    double yaw_rad = yaw * M_PI / 180.0;

    double vx = ly * cos(yaw_rad) - lx * sin(yaw_rad);
    double vy = ly * sin(yaw_rad) + lx * cos(yaw_rad);
    double omega = rx;

    double wheel_speeds[4];
    wheel_speeds[0] = vx - vy - omega; // 右前
    wheel_speeds[1] = vx + vy + omega; // 左前
    wheel_speeds[2] = vx + vy - omega; // 左後
    wheel_speeds[3] = vx - vy + omega; // 右後

    int16_t pwm_outputs[4];
    for (int i = 0; i < 4; ++i) {
        pwm_outputs[i] = static_cast<int16_t>(wheel_speeds[i] * speed_change);
    }

    CANMessage msg(0x200, (uint8_t*)pwm_outputs, 8);
    can.write(msg);

    // CANバスにメッセージを送信
    if (can.write(msg)) {
        printf("%d, %d, %d, %d\n", pwm_outputs[0], pwm_outputs[1], pwm_outputs[2], pwm_outputs[3]);
    } else {
        printf("CAN Send Failed\n");
    }
}

int main() {
    // BNO055の初期化
    if (imu.begin()) {
        printf("BNO055 Initialized!\n");
        ThisThread::sleep_for(100ms); // センサーが安定するまで少し待つ

        imu.update();
        yaw_offset = imu.getEuler().yaw;
        printf("Yaw offset set to: %.2f\n", yaw_offset);
    } else {
        printf("Failed to initialize BNO055.\n");
        while(1);
    }
    
    // char output_buf[20];

    while (1) {
        serial_read();
        imu_yaw_get();
        // printf("Relative Yaw: %7.2f\n", relative_yaw);

        // ps4controller_topic-communicationを使う場合のコード

        // read_until_pipe(output_buf, sizeof(output_buf));
        // output_buf_get(output_buf, &Lx, "L3_x:");
        // output_buf_get(output_buf, &Ly, "L3_y:");
        // output_buf_get(output_buf, &Rx, "R3_x:");

        printf("Lx: %f, Ly: %f, Rx: %f, Yaw: %f\n", Lx, Ly, Rx, relative_yaw);

        omni_control(Lx, Ly, Rx, relative_yaw);

        ThisThread::sleep_for(20ms);
    }
}