/**
 * 运动学层使用的内部数据类型。HTTP 层的原始入参类型见 src/validation/types.ts。
 */

/** 关节类型：revolute=转动关节（自由变量 theta），prismatic=移动关节（自由变量 d）。 */
export type JointType = 'revolute' | 'prismatic';

/** 全服务角度单位，内部计算统一为弧度。 */
export type AngleUnit = 'radian' | 'degree';

/** 4x4 齐次变换 / 旋转矩阵均用行优先的二维数组表示。 */
export type Matrix4 = number[][];

/** 三维点 / 向量。 */
export type Vec3 = [number, number, number];

/**
 * 完成“物化”的连杆：theta/d/a/alpha 四个 DH 参数都已是确定数值（角度为弧度）。
 * jointType 仅用于输出说明，链乘本身只依赖四个参数。
 */
export interface MaterializedLink {
  /** 从 1 开始的连杆序号。 */
  index: number;
  jointType: JointType;
  /** 绕 z 轴关节角（弧度）。 */
  theta: number;
  /** 沿 z 轴连杆偏距。 */
  d: number;
  /** 沿 x 轴连杆长度。 */
  a: number;
  /** 绕 x 轴扭角（弧度）。 */
  alpha: number;
}

/** 一次正演单连杆结果。 */
export interface LinkResult {
  index: number;
  jointType: JointType;
  /** 该杆相对前一杆的齐次变换 A_i。 */
  transform: Matrix4;
  /** 该杆坐标系原点在基座坐标系下的坐标（前缀连乘平移列）。 */
  origin: Vec3;
  /** 从基座到该杆坐标系的前缀累积变换。 */
  cumulative: Matrix4;
}

export interface ForwardKinematicsResult {
  /** 杆件数。 */
  linkCount: number;
  /** 基座到末端的总变换 = A_1 * A_2 * ... * A_n（未含工具变换）。 */
  totalTransform: Matrix4;
  /** 末端实际总变换 = totalTransform * tool。 */
  endEffectorTransform: Matrix4;
  /** 末端在基座系下的位置（平移部分）。 */
  position: Vec3;
  /** 末端姿态旋转矩阵（3x3）。 */
  rotation: number[][];
  /** 钉死约定下的内禀 ZYX 欧拉角 [roll, pitch, yaw]（弧度）。 */
  euler: { roll: number; pitch: number; yaw: number };
  /** 逐杆结果。 */
  links: LinkResult[];
  /** 各轴原点在基座系下的坐标，按连杆顺序（与 links[i].origin 相同）。 */
  axisOrigins: Vec3[];
  /**
   * 骨架点列：基座原点 [0,0,0] 开头，其后依次为各轴原点。
   * 按顺序相连即从基座到末端的骨架折线。
   */
  skeleton: Vec3[];
  /** 实际参与计算的固定工具变换（缺省为单位阵）。 */
  tool: Matrix4;
}

/** 平面两杆逆解的单支解。 */
export interface InverseSolution {
  branch: 'elbow-up' | 'elbow-down';
  /** 关节角 [theta1, theta2]，弧度。theta2 为相对第一杆的连杆角。 */
  joints: [number, number];
  /** 用该支解正演回去得到的末端位置。 */
  fkPosition: Vec3;
  /** 正演位置与目标的距离残差（必须 <= POSITION_TOLERANCE）。 */
  residual: number;
}

export type InverseKinematicsStatus = 'solved' | 'no-solution' | 'singular';

export interface InverseKinematicsResult {
  status: InverseKinematicsStatus;
  /** 目标点 [x, y]。 */
  target: [number, number];
  /** 两支解（有解且非奇异时恰好两支；奇异时两支重合为一支）。 */
  solutions: InverseSolution[];
  /** status 为 solved 时为 null。 */
  reason: string | null;
}
