/**
 * 平面两杆逆解测试：
 * - 正常点返回肘上/肘下两支，各自正演回贴目标
 * - 超出可达圆环返回 no-solution
 * - 伸直共线 / 等长折叠共线返回 singular
 * - 分支标签与几何位置一致
 */
import { planar2RInverse } from '../src/kinematics/inverse';
import { POSITION_TOLERANCE } from '../src/kinematics/constants';
import { approx, vecApprox } from './helpers';

describe('平面两杆逆解', () => {
  const L1 = 1;
  const L2 = 1;

  test('可达点返回两支，且每支正演回贴目标（容差内）', () => {
    const target: [number, number] = [1.2, 0.5];
    const r = planar2RInverse({ L1, L2, target });
    expect(r.status).toBe('solved');
    expect(r.solutions).toHaveLength(2);
    expect(r.solutions.map((s) => s.branch).sort()).toEqual(['elbow-down', 'elbow-up']);
    for (const s of r.solutions) {
      expect(s.residual).toBeLessThanOrEqual(POSITION_TOLERANCE);
      expect(vecApprox([s.fkPosition[0], s.fkPosition[1]], target, POSITION_TOLERANCE)).toBe(true);
      expect(Number.isFinite(s.joints[0]) && Number.isFinite(s.joints[1])).toBe(true);
    }
  });

  test('两支解确实不同（非共线时分支可区分）', () => {
    const r = planar2RInverse({ L1, L2, target: [0.6, 0.8] });
    const [a, b] = r.solutions;
    expect(Math.abs(a.joints[1] - b.joints[1])).toBeGreaterThan(1e-6);
  });

  test('肘上支的肘关节位于基座-目标连线之上（y 朝上）', () => {
    const target: [number, number] = [1.4, 0.3];
    const r = planar2RInverse({ L1, L2, target });
    const up = r.solutions.find((s) => s.branch === 'elbow-up')!;
    const ex = L1 * Math.cos(up.joints[0]);
    const ey = L1 * Math.sin(up.joints[0]);
    // 叉积 (target - base) x (elbow - base)：肘上为正
    const cross = target[0] * ey - target[1] * ex;
    expect(cross).toBeGreaterThan(0);
    expect(approx(ex, 0)).toBe(false);
  });

  test.each([
    { target: [3, 0] as [number, number], label: '超出外缘' },
    { target: [-2.5, 0.1] as [number, number], label: '斜向外超界' },
  ])('$label 返回 no-solution 且不给假角度', ({ target }) => {
    const r = planar2RInverse({ L1, L2, target });
    expect(r.status).toBe('no-solution');
    expect(r.solutions).toHaveLength(0);
    expect(r.reason).toBeTruthy();
  });

  test('等长臂、目标在基座原点（r=0）为折叠奇异，返回 singular 无分支', () => {
    const r = planar2RInverse({ L1, L2, target: [0, 0] });
    expect(r.status).toBe('singular');
    expect(r.reason).toContain('分支不可区分');
  });

  test('完全伸直共线（r=L1+L2）为奇异，返回一支重合解并标 singular', () => {
    const r = planar2RInverse({ L1: 1, L2: 0.8, target: [1.8, 0] });
    expect(r.status).toBe('singular');
    expect(r.solutions).toHaveLength(1);
    const s = r.solutions[0];
    expect(s.residual).toBeLessThanOrEqual(POSITION_TOLERANCE);
    expect(Math.abs(s.joints[1])).toBeLessThan(1e-9);
  });

  test('折叠共线（r=|L1-L2|>0）为奇异', () => {
    const r = planar2RInverse({ L1: 1.2, L2: 0.7, target: [0.5, 0] });
    expect(r.status).toBe('singular');
    expect(r.solutions).toHaveLength(1);
    expect(Math.abs(Math.abs(r.solutions[0].joints[1]) - Math.PI)).toBeLessThan(1e-9);
  });

  test('不等长臂落入内孔（r<|L1-L2|）返回 no-solution', () => {
    const r = planar2RInverse({ L1: 1.5, L2: 0.4, target: [0.2, 0] });
    expect(r.status).toBe('no-solution');
    expect(r.solutions).toHaveLength(0);
  });

  test('多点位扫描：每个可达点两支都回贴目标', () => {
    const pts: Array<[number, number]> = [
      [0.3, 0.3],
      [1.5, 0],
      [1.0, -1.0],
      [-0.5, 0.9],
      [0.2, -1.2],
    ];
    for (const target of pts) {
      const r = planar2RInverse({ L1: 1.1, L2: 0.9, target });
      expect(r.status).toBe('solved');
      for (const s of r.solutions) {
        expect(s.residual).toBeLessThanOrEqual(POSITION_TOLERANCE);
      }
    }
  });
});
