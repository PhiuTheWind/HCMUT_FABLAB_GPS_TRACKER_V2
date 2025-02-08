#ifndef YAW_H
#define YAW_H

class KalmanFilter {
  private:
    float angle;       // Góc được ước tính
    float bias;        // Bias của gyroscope
    float rate;        // Tốc độ góc

    float P[2][2];     // Ma trận không chắc chắn
    float Q_angle;     // Nhiễu góc
    float Q_bias;      // Nhiễu bias
    float R_measure;   // Nhiễu đo lường

  public:
    // Constructor: khởi tạo các giá trị mặc định
    KalmanFilter() {
        angle = 0.0;
        bias = 0.0;
        rate = 0.0;

        P[0][0] = 0.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 0.0;

        Q_angle = 0.001;    // Nhiễu của góc
        Q_bias = 0.003;     // Nhiễu của bias
        R_measure = 0.03;   // Nhiễu của đo lường
    }

    // Hàm tính toán góc bằng Kalman Filter
    float getAngle(float newAngle, float newRate, float dt) {
        // Dự đoán
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Cập nhật
        float S = P[0][0] + R_measure;
        float K[2]; // Kalman gain
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle; // Sai lệch giữa đo lường và dự đoán
        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

#endif // YAW_H
