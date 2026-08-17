#pragma once

#include <cmath>

namespace actuators::fin_kinematics {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// AS5047Dの1回転角を、前回sampleからの最短差分として求める。
// producerが連続してsampleを取得し、隣接valid sample間で実回転が180度未満であることを前提とする。
inline double unwrapEncoderDelta(double current_raw_rad,
                                 double previous_raw_rad) {
  return std::remainder(current_raw_rad - previous_raw_rad, kTwoPi);
}

// 再接続後の1回転角を、切断前の連続角に最も近いmulti-turn branchへ復元する。
inline double nearestEquivalentAngle(double raw_rad,
                                     double reference_unwrapped_rad) {
  return raw_rad +
         std::round((reference_unwrapped_rad - raw_rad) / kTwoPi) * kTwoPi;
}

// 実機取付ではAS5047D正方向と定義したFin正方向が逆向きである。
// motor polarityと同じFin座標系に揃えるため、encoder軸角を符号反転して
// total gear ratioで出力軸Fin角へ変換する。
inline double encoderToFinRadians(double encoder_rad, double gear_ratio) {
  return -encoder_rad / gear_ratio;
}

} // namespace actuators::fin_kinematics
