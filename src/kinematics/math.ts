/**
 * 齐次变换矩阵的基础运算。
 * 全部为无状态纯函数：不使用任何模块级可变中间量，
 * 从根本上避免相邻请求之间残留矩阵互相污染。
 */
import { Matrix4, Vec3 } from './types';

/** 4x4 单位阵（每次新建，调用方可以安全修改返回值）。 */
export function identity(): Matrix4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

/** 深拷贝一个 4x4 矩阵。 */
export function clone(m: Matrix4): Matrix4 {
  return m.map((row) => row.slice());
}

/** 通用 n×n 矩阵乘法 C = A * B（此处只用于 4x4，但保持通用便于复核）。 */
export function multiply(A: number[][], B: number[][]): number[][] {
  const n = A.length;
  if (B.length !== n || A[0].length !== n || B[0].length !== n) {
    throw new Error('matrix multiply: 方阵维度不一致');
  }
  const C: number[][] = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let sum = 0;
      for (let k = 0; k < n; k++) {
        sum += A[i][k] * B[k][j];
      }
      C[i][j] = sum;
    }
  }
  return C;
}

/** 4x4 连乘：chainMultiply([A1, A2, A3]) = A1 * A2 * A3。空表返回单位阵。 */
export function chainMultiply(matrices: Matrix4[]): Matrix4 {
  return matrices.reduce<Matrix4>((acc, m) => multiply(acc, m) as Matrix4, identity());
}

/** 取齐次变换的平移部分 [tx, ty, tz]。 */
export function translationOf(m: Matrix4): Vec3 {
  return [m[0][3], m[1][3], m[2][3]];
}

/** 取齐次变换左上 3x3 旋转矩阵。 */
export function rotationOf(m: Matrix4): number[][] {
  return [
    [m[0][0], m[0][1], m[0][2]],
    [m[1][0], m[1][1], m[1][2]],
    [m[2][0], m[2][1], m[2][2]],
  ];
}

/** 向量欧氏距离。 */
export function distance(a: Vec3 | number[], b: Vec3 | number[]): number {
  let sum = 0;
  for (let i = 0; i < a.length; i++) {
    const d = a[i] - b[i];
    sum += d * d;
  }
  return Math.sqrt(sum);
}

/**
 * 两个 4x4 矩阵的元素级最大绝对误差（用于“总变换必须等于逐杆连乘”的复核）。
 */
export function maxAbsDiff(A: Matrix4, B: Matrix4): number {
  let max = 0;
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      max = Math.max(max, Math.abs(A[i][j] - B[i][j]));
    }
  }
  return max;
}

/**
 * 判断一个 4x4 数组形状是否合法：恰好 4 行，每行恰好 4 个 number。
 * 有限性检查在调用处完成（以便给出可区分错误）。
 */
export function isMatrix4Shape(v: unknown): v is Matrix4 {
  return (
    Array.isArray(v) &&
    v.length === 4 &&
    v.every((row) => Array.isArray(row) && row.length === 4 && row.every((x) => typeof x === 'number'))
  );
}
