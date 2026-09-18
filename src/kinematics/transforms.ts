/**
 * 标准 DH 单杆变换。
 *
 * 全服务唯一允许的约定（钉死）：
 *   A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
 *
 * 实现上显式构造四个原语矩阵并按该固定顺序连乘，而不是只写闭式结果，
 * 这样“约定是什么”在代码中可直接审计，也便于测试逐原语复核。
 * 所有角度入参必须是弧度。
 */
import { Matrix4 } from './types';
import { multiply } from './math';

/** 绕 z 轴旋转 theta（弧度）。 */
export function rotZ(theta: number): Matrix4 {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

/** 沿 z 轴平移 d。 */
export function transZ(d: number): Matrix4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, d],
    [0, 0, 0, 1],
  ];
}

/** 沿 x 轴平移 a。 */
export function transX(a: number): Matrix4 {
  return [
    [1, 0, 0, a],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

/** 绕 x 轴旋转 alpha（弧度）。 */
export function rotX(alpha: number): Matrix4 {
  const c = Math.cos(alpha);
  const s = Math.sin(alpha);
  return [
    [1, 0, 0, 0],
    [0, c, -s, 0],
    [0, s, c, 0],
    [0, 0, 0, 1],
  ];
}

/**
 * 标准 DH 连杆变换：A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)。
 * 顺序钉死，不接受任何“修改 DH / Craig DH”之类的切换参数。
 */
export function linkTransform(theta: number, d: number, a: number, alpha: number): Matrix4 {
  return multiply(multiply(multiply(rotZ(theta), transZ(d)), transX(a)), rotX(alpha)) as Matrix4;
}

/**
 * 闭式标准 DH 矩阵，仅用于与显式连乘结果交叉复核（测试中使用）。
 */
export function linkTransformClosedForm(theta: number, d: number, a: number, alpha: number): Matrix4 {
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}
