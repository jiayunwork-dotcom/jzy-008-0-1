/**
 * 旋转矩阵 -> 钉死约定的内禀 ZYX 欧拉角（外动轴 yaw-pitch-roll）。
 *
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *
 * 对应的旋转矩阵：
 *   [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr ]
 *   [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr ]
 *   [ -sp,    cp*sr,             cp*cr            ]
 *
 * 由此：
 *   pitch = asin(-R[2][0])（钳制到 [-1,1]）
 *   当 |cp| > 1e-9（非万向锁）：
 *     yaw  = atan2(R[1][0], R[0][0])
 *     roll = atan2(R[2][1], R[2][2])
 *   当接近万向锁（pitch = ±π/2）：
 *     取 yaw = 0，roll 由 R[0][1] 项解出（明确的钉死分支，不再二义）。
 */
import { EULER_CONVENTION } from './constants';

export interface EulerAngles {
  roll: number;
  pitch: number;
  yaw: number;
  convention: typeof EULER_CONVENTION;
}

export function rotationToEuler(R: number[][]): EulerAngles {
  const gimbalThreshold = 1e-9;
  const sinPitch = clampUnit(-R[2][0]);
  const pitch = Math.asin(sinPitch);
  const cosPitch = Math.cos(pitch);

  let roll: number;
  let yaw: number;
  if (Math.abs(cosPitch) > gimbalThreshold) {
    yaw = Math.atan2(R[1][0], R[0][0]);
    roll = Math.atan2(R[2][1], R[2][2]);
  } else {
    // 万向锁：钉死取 yaw = 0。
    yaw = 0;
    if (sinPitch < 0) {
      // pitch = +π/2：R[0][1] = sr, R[0][2] = cr
      roll = Math.atan2(R[0][1], R[0][2]);
    } else {
      // pitch = -π/2：R[0][1] = -sr, R[0][2] = cr
      roll = Math.atan2(-R[0][1], R[0][2]);
    }
  }

  return { roll, pitch, yaw, convention: EULER_CONVENTION };
}

function clampUnit(x: number): number {
  if (x > 1) return 1;
  if (x < -1) return -1;
  return x;
}
