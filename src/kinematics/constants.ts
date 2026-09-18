/**
 * 全服务统一的数值容差与运动学约定常量。
 * 容差是“钉死”的：所有正演复核、逆解回贴校验、矩阵一致性比较都引用这里。
 */

/** 矩阵连乘一致性比较容差（元素级最大绝对误差）。 */
export const MATRIX_TOLERANCE = 1e-10;

/** 位置（米/长度单位）正演回贴容差。 */
export const POSITION_TOLERANCE = 1e-8;

/** 一般标量浮点比较容差。 */
export const EPSILON = 1e-12;

/**
 * acos/asin 的合法自变量钳制范围。1 + 小余量，避免浮点误差把 1.0000000002 判成 NaN。
 */
export const TRIG_CLAMP = 1 + 1e-9;

/**
 * 钉死的姿态阅读约定：内禀 Z-Y-X 欧拉角 = 外动轴 yaw(Z) -> pitch(Y) -> roll(X)，
 * 即 R = Rz(yaw) * Ry(pitch) * Rx(roll)。全服务只给出这一种欧拉角。
 */
export const EULER_CONVENTION = 'intrinsic ZYX (Rz(yaw)*Ry(pitch)*Rx(roll))' as const;

/** 全服务唯一允许的 DH 约定描述。 */
export const DH_CONVENTION = {
  name: 'standard-dh' as const,
  /**
   * 标准 DH 单杆变换，固定连乘顺序（右到左为作用顺序）：
   *   A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
   */
  formula: 'A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)' as const,
  order: ['Rz(theta)', 'Tz(d)', 'Tx(a)', 'Rx(alpha)'] as const,
  angleUnit: 'radian' as const,
  /** 转动关节自由变量是关节角 theta；移动关节自由变量是连杆偏距 d。 */
  revoluteFreeVariable: 'theta' as const,
  prismaticFreeVariable: 'd' as const,
  euler: EULER_CONVENTION,
};
