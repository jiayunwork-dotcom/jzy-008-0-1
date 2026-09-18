/**
 * 开链连杆链乘与正向运动学。
 *
 * 末端位姿 = A_1 * A_2 * ... * A_n（按从基座到末端顺序连乘），
 * 再右乘固定工具变换 T_tool（缺省单位阵）：T = (A_1...A_n) * T_tool。
 *
 * 每次调用都在函数内部新建全部中间矩阵，不读取也不写入任何模块级变量，
 * 因此相邻两次请求不可能因上次连乘的中间矩阵残留而互相污染。
 */
import {
  ForwardKinematicsResult,
  LinkResult,
  MaterializedLink,
  Matrix4,
  Vec3,
} from './types';
import { chainMultiply, identity, multiply, rotationOf, translationOf } from './math';
import { linkTransform } from './transforms';
import { rotationToEuler } from './euler';
import { maxAbsDiff } from './math';
import { MATRIX_TOLERANCE } from './constants';

export interface ForwardKinematicsOutput extends ForwardKinematicsResult {
  /** 复核诊断：逐杆独立连乘得到的总变换与主循环累积结果的最大元素误差。 */
  chainCheckMaxError: number;
}

/**
 * 计算一次完整正演。
 *
 * @param links 已物化的连杆（角度为弧度），按基座到末端顺序排列
 * @param tool  固定工具变换（4x4），缺省由调用方传单位阵
 */
export function forwardKinematics(links: MaterializedLink[], tool: Matrix4): ForwardKinematicsOutput {
  // 每个连杆的独立变换，全部新建。
  const transforms: Matrix4[] = links.map((l) => linkTransform(l.theta, l.d, l.a, l.alpha));

  // 前缀累积：cumulative[i] = A_1 * ... * A_i；cumulative[0] 对应基座（单位阵）。
  const linkResults: LinkResult[] = [];
  const axisOrigins: Vec3[] = [];
  let cumulative: Matrix4 = identity();

  for (let i = 0; i < links.length; i++) {
    cumulative = multiply(cumulative, transforms[i]) as Matrix4;
    const origin = translationOf(cumulative);
    axisOrigins.push(origin);
    linkResults.push({
      index: links[i].index,
      jointType: links[i].jointType,
      transform: transforms[i],
      origin,
      cumulative,
    });
  }

  // 独立复核路径：不使用主循环的 cumulative，直接对全部单杆变换重新连乘。
  // “服务给出的总变换必须等于逐杆连乘”以此处的差值为证据。
  const totalTransform = chainMultiply(transforms);
  const chainCheckMaxError = maxAbsDiff(cumulative, totalTransform);
  if (chainCheckMaxError > MATRIX_TOLERANCE) {
    // 纯算术上不应发生；一旦发生说明实现有缺陷，直接抛出而不是静默返回错误位姿。
    throw new Error(
      `正向运动学内部一致性复核失败：累积变换与逐杆连乘之差 ${chainCheckMaxError} 超过容差 ${MATRIX_TOLERANCE}`,
    );
  }

  // 工具变换在最右侧：T_0e = T_0n * T_tool。
  const endEffectorTransform = multiply(totalTransform, tool) as Matrix4;

  const position = translationOf(endEffectorTransform);
  const rotation = rotationOf(endEffectorTransform);
  const euler = rotationToEuler(rotation);

  // 骨架折线：基座原点 + 各轴原点（末端工具不改变骨架关节点）。
  const skeleton: Vec3[] = [[0, 0, 0], ...axisOrigins.map((p) => [...p] as Vec3)];

  return {
    linkCount: links.length,
    totalTransform,
    endEffectorTransform,
    position,
    rotation,
    euler: { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw },
    links: linkResults,
    axisOrigins,
    skeleton,
    tool,
    chainCheckMaxError,
  };
}
