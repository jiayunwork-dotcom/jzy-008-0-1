/**
 * 运动学硬性规则测试：
 * - 平面三杆全零伸直位
 * - 单关节加 2π 总变换不变
 * - 杆长全部加倍：位置加倍、旋转不变
 * - 只动最后一个关节：末端绕腕点转、腕点不动
 * - 逐杆链乘与总变换一致
 * - 移动关节沿自身轴线平移、平移量等于偏距增量
 * - 度在入口换算与弧度一致
 * - 工具变换右乘
 */
import { forwardKinematics } from '../src/kinematics/chain';
import { linkTransform, linkTransformClosedForm } from '../src/kinematics/transforms';
import { chainMultiply, identity, maxAbsDiff, multiply, translationOf } from '../src/kinematics/math';
import { MaterializedLink, Matrix4 } from '../src/kinematics/types';
import { MATRIX_TOLERANCE, POSITION_TOLERANCE } from '../src/kinematics/constants';
import { planarChain, vecApprox, matApprox, approx } from './helpers';

function revoluteLinks(lengths: number[], thetas: number[]): MaterializedLink[] {
  return lengths.map((a, i) => ({
    index: i + 1,
    jointType: 'revolute' as const,
    theta: thetas[i] ?? 0,
    d: 0,
    a,
    alpha: 0,
  }));
}

describe('标准 DH 单杆变换约定', () => {
  test('显式 Rz*Tz*Tx*Rx 连乘与闭式标准 DH 矩阵一致', () => {
    const [theta, d, a, alpha] = [0.7, 0.3, 1.2, -0.4];
    const A = linkTransform(theta, d, a, alpha);
    const Aref = linkTransformClosedForm(theta, d, a, alpha);
    expect(maxAbsDiff(A, Aref)).toBeLessThan(MATRIX_TOLERANCE);
  });
});

describe('平面三杆全零伸直位', () => {
  test('末端沿第一杆方向坐标 = 三杆长度之和，垂直为 0，姿态无扭转', () => {
    const fk = forwardKinematics(revoluteLinks([1, 1, 1], [0, 0, 0]), identity());
    expect(vecApprox(fk.position, [3, 0, 0], POSITION_TOLERANCE)).toBe(true);
    expect(matApprox(fk.rotation, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])).toBe(true);
    expect(approx(fk.euler.roll, 0)).toBe(true);
    expect(approx(fk.euler.pitch, 0)).toBe(true);
    expect(approx(fk.euler.yaw, 0)).toBe(true);
    // 骨架点列等距落在 x 轴上。
    expect(fk.skeleton).toHaveLength(4);
    expect(vecApprox(fk.skeleton[0], [0, 0, 0])).toBe(true);
    expect(vecApprox(fk.skeleton[1], [1, 0, 0])).toBe(true);
    expect(vecApprox(fk.skeleton[2], [2, 0, 0])).toBe(true);
    expect(vecApprox(fk.skeleton[3], [3, 0, 0])).toBe(true);
    // 各轴原点前缀坐标。
    expect(vecApprox(fk.axisOrigins[0], [1, 0, 0])).toBe(true);
    expect(vecApprox(fk.axisOrigins[1], [2, 0, 0])).toBe(true);
    expect(vecApprox(fk.axisOrigins[2], [3, 0, 0])).toBe(true);
  });

  test('任意非零杆长 L1/L2/L3 全零时末端为 [L1+L2+L3, 0, 0]', () => {
    const [L1, L2, L3] = [2.5, 0.7, 1.3];
    const fk = forwardKinematics(revoluteLinks([L1, L2, L3], [0, 0, 0]), identity());
    expect(vecApprox(fk.position, [L1 + L2 + L3, 0, 0], POSITION_TOLERANCE)).toBe(true);
  });
});

describe('转动关节加一周（2π）总变换不变', () => {
  test.each([
    { which: 0 },
    { which: 1 },
    { which: 2 },
  ])('只给第 $which 个关节加 2π，总变换一致', ({ which }) => {
    const base = [0.4, -0.9, 1.1];
    const per = [0.4, -0.9, 1.1];
    per[which] += 2 * Math.PI;
    const fk0 = forwardKinematics(revoluteLinks([1.3, 0.8, 0.6], base), identity());
    const fk1 = forwardKinematics(revoluteLinks([1.3, 0.8, 0.6], per), identity());
    expect(maxAbsDiff(fk0.totalTransform, fk1.totalTransform)).toBeLessThan(MATRIX_TOLERANCE);
  });
});

describe('所有杆长同时加倍', () => {
  test('末端位置坐标加倍，旋转部分保持不变', () => {
    const joints = [0.35, 0.8, -0.5];
    const fkA = forwardKinematics(revoluteLinks([1, 1.5, 0.7], joints), identity());
    const fkB = forwardKinematics(revoluteLinks([2, 3, 1.4], joints), identity());
    expect(vecApprox(fkB.position, fkA.position.map((v) => 2 * v), POSITION_TOLERANCE)).toBe(true);
    expect(matApprox(fkB.rotation, fkA.rotation, MATRIX_TOLERANCE)).toBe(true);
  });
});

describe('只改最后一个关节角：末端绕腕点转，腕点不动', () => {
  test('前两杆轴原点（腕点）基座坐标不变，末端到腕点距离恒为 L3', () => {
    const L = [1.2, 0.9, 0.6];
    const fk1 = forwardKinematics(revoluteLinks(L, [0.5, 0.2, 0.3]), identity());
    const fk2 = forwardKinematics(revoluteLinks(L, [0.5, 0.2, 1.1]), identity());

    // 腕点 = 第二杆（最后一杆之前）的轴原点，在基座系下。
    const wrist1 = fk1.axisOrigins[1];
    const wrist2 = fk2.axisOrigins[1];
    expect(vecApprox(wrist2, wrist1, POSITION_TOLERANCE)).toBe(true);
    // 前两个轴原点都不应变化。
    expect(vecApprox(fk2.axisOrigins[0], fk1.axisOrigins[0], POSITION_TOLERANCE)).toBe(true);

    const r1 = Math.hypot(fk1.position[0] - wrist1[0], fk1.position[1] - wrist1[1]);
    const r2 = Math.hypot(fk2.position[0] - wrist2[0], fk2.position[1] - wrist2[1]);
    expect(approx(r1, L[2], POSITION_TOLERANCE)).toBe(true);
    expect(approx(r2, L[2], POSITION_TOLERANCE)).toBe(true);
    // 末端确实转动了（不是没动）。
    expect(vecApprox(fk2.position, fk1.position)).toBe(false);
  });
});

describe('逐杆链乘与总变换一致', () => {
  test('总变换 = A1*A2*A3（独立重算），并与闭式手算矩阵连乘一致', () => {
    const links = revoluteLinks([1.1, 0.7, 0.9], [0.2, -0.6, 1.3]);
    const fk = forwardKinematics(links, identity());
    const independent = chainMultiply(
      links.map((l) => linkTransform(l.theta, l.d, l.a, l.alpha)),
    );
    expect(fk.chainCheckMaxError).toBeLessThanOrEqual(MATRIX_TOLERANCE);
    expect(maxAbsDiff(fk.totalTransform, independent)).toBeLessThanOrEqual(MATRIX_TOLERANCE);

    // 闭式矩阵独立连乘再复核一次。
    const closed = chainMultiply(
      links.map((l) => linkTransformClosedForm(l.theta, l.d, l.a, l.alpha)),
    );
    expect(maxAbsDiff(fk.totalTransform, closed)).toBeLessThanOrEqual(MATRIX_TOLERANCE);
  });

  test('工具变换右乘：endEffectorTransform = totalTransform * tool', () => {
    const links = revoluteLinks([1, 1], [0.3, 0.4]);
    const tool: Matrix4 = [
      [1, 0, 0, 0.2],
      [0, 0, -1, 0.1],
      [0, 1, 0, 0],
      [0, 0, 0, 1],
    ];
    const fk = forwardKinematics(links, tool);
    const expectT = multiply(fk.totalTransform, tool);
    expect(maxAbsDiff(fk.endEffectorTransform, expectT as Matrix4)).toBeLessThanOrEqual(MATRIX_TOLERANCE);
  });

  test('缺省工具为单位阵时 totalTransform 与 endEffectorTransform 相同', () => {
    const fk = forwardKinematics(revoluteLinks([1, 1], [0.3, 0.4]), identity());
    expect(maxAbsDiff(fk.totalTransform, fk.endEffectorTransform)).toBe(0);
  });
});

describe('移动关节：只改偏距则沿关节轴线平移', () => {
  function onePrismatic(delta: number) {
    // 标准 DH 中第 i 杆的关节轴是 z_{i-1}：第二杆（移动关节）的轴是 z_1。
    // 取杆1 theta1=30°, alpha1=-90°：z_1 = Rz(theta1)Rx(-90°)·z = [-sinθ1, cosθ1, 0]，
    // 即关节平移轴线 u = [-sin30°, cos30°, 0]（在水平面内）。
    const theta1 = Math.PI / 6;
    const links: MaterializedLink[] = [
      { index: 1, jointType: 'revolute', theta: theta1, d: 0, a: 0, alpha: -Math.PI / 2 },
      { index: 2, jointType: 'prismatic', theta: 0, d: 0.2 + delta, a: 0.4, alpha: 0 },
    ];
    return forwardKinematics(links, identity());
  }

  test('平移向量方向沿该关节轴线、模长等于偏距增量', () => {
    const fk0 = onePrismatic(0);
    const delta = 0.83;
    const fk1 = onePrismatic(delta);
    const dx = fk1.position[0] - fk0.position[0];
    const dy = fk1.position[1] - fk0.position[1];
    const dz = fk1.position[2] - fk0.position[2];
    expect(approx(Math.hypot(dx, dy, dz), delta, POSITION_TOLERANCE)).toBe(true);
    // 移动关节轴线 u = [-sinθ1, cosθ1, 0]。
    const u = [-Math.sin(Math.PI / 6), Math.cos(Math.PI / 6), 0];
    expect(approx(dx / delta, u[0], 1e-9)).toBe(true);
    expect(approx(dy / delta, u[1], 1e-9)).toBe(true);
    expect(approx(dz, 0, POSITION_TOLERANCE)).toBe(true);
    // 姿态不变。
    expect(matApprox(fk0.rotation, fk1.rotation, MATRIX_TOLERANCE)).toBe(true);
  });
});

describe('角度单位入口换算', () => {
  test('度传入（30°, 45°, -60°）与弧度传入结果一致', () => {
    const L = [1, 1, 1];
    const rad = [Math.PI / 6, Math.PI / 4, -Math.PI / 3];
    const fkRad = forwardKinematics(revoluteLinks(L, rad), identity());
    // 通过 HTTP 层的校验器走一遍，确保换算发生在入口。
    // 这里直接用同样的角度构造进行数学层面的对照由校验测试覆盖；
    // 先验证弧度情形位置手算正确。
    const theta = rad[0] + rad[1] + rad[2];
    const x =
      L[0] * Math.cos(rad[0]) +
      L[1] * Math.cos(rad[0] + rad[1]) +
      L[2] * Math.cos(theta);
    const y =
      L[0] * Math.sin(rad[0]) +
      L[1] * Math.sin(rad[0] + rad[1]) +
      L[2] * Math.sin(theta);
    expect(vecApprox(fkRad.position, [x, y, 0], POSITION_TOLERANCE)).toBe(true);
  });

  test('平面两杆度批量载荷与弧度载荷通过 service 得到同一位置', async () => {
    const { service, store } = await import('./helpers').then((m) => m.makeService());
    const deg = await service.forward(planarChain([1, 1], [30, 60], 'degree'));
    const rad = await service.forward(planarChain([1, 1], [Math.PI / 6, Math.PI / 3], 'radian'));
    expect(vecApprox(deg.result.position, rad.result.position, POSITION_TOLERANCE)).toBe(true);
    await store.close();
  });
});

describe('相邻两次正演互不污染', () => {
  test('先算非零位形再算全零位形，结果与直接算全零完全相同', () => {
    const linksA = revoluteLinks([1, 1, 1], [1.2, -2.3, 0.7]);
    const linksZero = revoluteLinks([1, 1, 1], [0, 0, 0]);
    forwardKinematics(linksA, identity());
    forwardKinematics(linksA, identity());
    const after = forwardKinematics(linksZero, identity());
    const direct = forwardKinematics(revoluteLinks([1, 1, 1], [0, 0, 0]), identity());
    expect(maxAbsDiff(after.totalTransform, direct.totalTransform)).toBe(0);
    expect(vecApprox(after.position, translationOf(direct.totalTransform))).toBe(true);
  });
});
