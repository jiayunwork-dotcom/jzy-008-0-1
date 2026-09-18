/**
 * HTTP 层使用的“原始入参”类型（全部为 unknown，由校验层负责收窄）。
 */

export interface RawLink {
  jointType?: unknown;
  /** 连杆长度 a（沿 x）。 */
  a?: unknown;
  /** 扭角 alpha（绕 x），单位服从整次请求的 angleUnit。 */
  alpha?: unknown;
  /**
   * 连杆偏距 d（沿 z）：
   * - 转动关节：d 为固定结构参数（缺省 0）；
   * - 移动关节：d 是自由变量，由关节矢量给出，此处的值仅作基准偏距（缺省 0）。
   */
  d?: unknown;
  /**
   * 关节角 theta（绕 z）：
   * - 移动关节：theta 为固定结构参数（缺省 0）；
   * - 转动关节：theta 是自由变量，由关节矢量给出。
   */
  theta?: unknown;
  /** 单根杆声明允许负杆长；也可用请求级 allowNegativeLengths 统一放行。 */
  allowNegativeLength?: unknown;
  /** 若单根杆自带角度单位，必须与请求级 angleUnit 一致，否则 ANGLE_UNIT_CONFLICT。 */
  angleUnit?: unknown;
}

/** 关节矢量元素：裸数值，或显式标出它驱动哪个自由变量的对象。 */
export type RawJoint =
  | unknown
  | {
      value?: unknown;
      /** 显式声明该关节驱动的自由变量；与关节类型对不上则拒绝。 */
      freeVariable?: unknown;
    };

export interface RawForwardRequest {
  armId?: unknown;
  /** 批量场景下单组可选的名称。 */
  name?: unknown;
  /** 整次请求统一角度单位；缺省 radian。全服务内部只按弧度计算。 */
  angleUnit?: unknown;
  links?: unknown;
  joints?: unknown;
  /** 固定工具变换（4x4），右乘在末端；缺省单位阵。 */
  tool?: unknown;
  /** 请求级统一放行负杆长。 */
  allowNegativeLengths?: unknown;
}

export interface RawBatchCase {
  name?: unknown;
  armId?: unknown;
  angleUnit?: unknown;
  links?: unknown;
  joints?: unknown;
  tool?: unknown;
  allowNegativeLengths?: unknown;
}

export interface RawBatchRequest {
  angleUnit?: unknown;
  cases?: unknown;
}

export interface RawInverseRequest {
  L1?: unknown;
  L2?: unknown;
  target?: unknown;
}

/** 校验物化后的正演请求（角度均已换算为弧度）。 */
export interface ValidatedForwardRequest {
  armId: string | null;
  angleUnit: 'radian' | 'degree';
  links: import('../kinematics/types').MaterializedLink[];
  tool: import('../kinematics/types').Matrix4;
}

export interface ValidatedBatchCase extends RawBatchCase {
  name: string | null;
}

export interface HistoryQuery {
  type?: string;
  status?: 'ok' | 'error';
  armId?: string;
  since?: number;
  until?: number;
  limit: number;
  offset: number;
}
