/**
 * 平面两转动关节臂（2R）逆解。
 *
 * 两杆标准 DH（平面内 alpha=0, d=0）：
 *   杆1：a = L1，自由变量 theta1
 *   杆2：a = L2，自由变量 theta2（相对第一杆的连杆角）
 * 正演：x = L1 cos(theta1) + L2 cos(theta1+theta2)
 *       y = L1 sin(theta1) + L2 sin(theta1+theta2)
 *
 * 几何解：
 *   r^2 = x^2 + y^2
 *   cos(theta2) = (r^2 - L1^2 - L2^2) / (2 L1 L2)
 *   phi = atan2(y, x)，beta = atan2(L2 sin(theta2), L1 + L2 cos(theta2))
 *   theta1 = phi - beta
 *
 * 分支（数学坐标 y 朝上、theta 逆时针为正）：
 *   theta2 = -acos(...) 为肘上 (elbow-up)
 *   theta2 = +acos(...) 为肘下 (elbow-down)
 *
 * 每一支解都用本服务同一套标准 DH 正演引擎回算位置并核对残差，
 * 残差不达标则绝不作为有效解返回。
 */
import { InverseKinematicsResult, InverseSolution, MaterializedLink, Vec3 } from './types';
import { POSITION_TOLERANCE, TRIG_CLAMP } from './constants';
import { distance, identity } from './math';
import { forwardKinematics } from './chain';

export interface Planar2RParams {
  /** 第一杆长度（必须为正）。 */
  L1: number;
  /** 第二杆长度（必须为正）。 */
  L2: number;
  /** 末端目标平面位置。 */
  target: [number, number];
}

/** 判定“共线奇异”时 cos(theta2) 距 ±1 的阈值（钉死）。 */
const SINGULAR_THRESHOLD = 1e-10;

export function planar2RInverse(p: Planar2RParams): InverseKinematicsResult {
  const { L1, L2 } = p;
  const [x, y] = p.target;
  const result: InverseKinematicsResult = {
    status: 'solved',
    target: [x, y],
    solutions: [],
    reason: null,
  };

  const r = Math.hypot(x, y);
  const outer = L1 + L2;
  const inner = Math.abs(L1 - L2);

  // 超出可达圆环（外缘之外或内缘之内）。留出一个位置容差带，带内按奇异处理。
  if (r > outer + POSITION_TOLERANCE || r < inner - POSITION_TOLERANCE) {
    result.status = 'no-solution';
    result.reason =
      `目标超出平面两杆可达圆环：到基座距离 r=${r} 不在 ` +
      `[|L1-L2|=${inner}, L1+L2=${outer}] 之内（容差 ${POSITION_TOLERANCE}）`;
    return result;
  }

  // cos(theta2)，钳制到 [-1,1] 以吸收边界处的浮点误差。
  let c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 > TRIG_CLAMP || c2 < -TRIG_CLAMP) {
    // 能走到这里说明可达性判据有漏洞，直接按无解处理而不是产出 NaN。
    result.status = 'no-solution';
    result.reason = `cos(theta2)=${c2} 超出定义域，目标不可达`;
    return result;
  }
  c2 = Math.max(-1, Math.min(1, c2));

  // 两杆共线：肘上/肘下两支无法区分 → 奇异。
  if (Math.abs(Math.abs(c2) - 1) <= SINGULAR_THRESHOLD) {
    if (r <= POSITION_TOLERANCE) {
      // 等长折叠且目标在基座原点：theta1 任意均可达到目标，分支彻底不可区分。
      result.status = 'singular';
      result.reason = '目标位于基座原点且两杆等长折叠：关节角有无穷多组，分支不可区分';
      return result;
    }
    const theta2 = c2 > 0 ? 0 : Math.PI;
    const phi = Math.atan2(y, x);
    const beta = Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));
    const theta1 = phi - beta;
    const verified = verify([theta1, theta2], L1, L2, [x, y]);
    result.status = 'singular';
    result.reason =
      c2 > 0
        ? '两杆完全伸直共线：肘上与肘下两支重合，分支无法区分'
        : '两杆折叠共线：肘上与肘下两支重合，分支无法区分';
    result.solutions = [verified];
    return result;
  }

  const phi = Math.atan2(y, x);
  const s2mag = Math.sqrt(1 - c2 * c2);

  // 两支：s2 < 0 肘上，s2 > 0 肘下。
  const branches: Array<{ branch: InverseSolution['branch']; s2: number }> = [
    { branch: 'elbow-up', s2: -s2mag },
    { branch: 'elbow-down', s2: s2mag },
  ];

  for (const { branch, s2 } of branches) {
    const theta2 = Math.atan2(s2, c2);
    const beta = Math.atan2(L2 * s2, L1 + L2 * c2);
    const theta1 = phi - beta;
    result.solutions.push(verify([theta1, theta2], L1, L2, [x, y], branch));
  }

  // 钉死规则：任何一支正演回贴不上目标，就不能以 solved 返回。
  const bad = result.solutions.find((s) => s.residual > POSITION_TOLERANCE);
  if (bad) {
    result.status = 'no-solution';
    result.solutions = [];
    result.reason = `逆解分支 ${bad.branch} 正演回贴残差 ${bad.residual} 超过容差 ${POSITION_TOLERANCE}`;
  }

  return result;
}

/** 用服务自身的标准 DH 正演引擎回算一支解的末端位置与残差。 */
function verify(
  joints: [number, number],
  L1: number,
  L2: number,
  target: [number, number],
  branch: InverseSolution['branch'] = 'elbow-up',
): InverseSolution {
  const [theta1, theta2] = joints;
  const links: MaterializedLink[] = [
    { index: 1, jointType: 'revolute', theta: theta1, d: 0, a: L1, alpha: 0 },
    { index: 2, jointType: 'revolute', theta: theta2, d: 0, a: L2, alpha: 0 },
  ];
  const fk = forwardKinematics(links, identity());
  const fkPosition: Vec3 = fk.position;
  return {
    branch,
    joints: [theta1, theta2],
    fkPosition,
    residual: distance([fkPosition[0], fkPosition[1]], [target[0], target[1]]),
  };
}
