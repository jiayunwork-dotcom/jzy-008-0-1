/**
 * HTTP 原始入参 → 运动学层可用的物化请求。
 *
 * 校验在任何三角函数与矩阵运算之前完成，且集中在本文件：
 * 缺字段、类型错误、非有限值、负杆长、关节矢量长度不符、
 * 关节类型与自由变量对不上、角度单位矛盾、工具矩阵非法等
 * 都会抛出带杆号/参数名的 ValidationError，绝不带病计算。
 */
import {
  AngleUnit,
  JointType,
  MaterializedLink,
  Matrix4,
  Vec3,
} from '../kinematics/types';
import { isMatrix4Shape } from '../kinematics/math';
import { ValidationError, linkError } from './errors';
import { ArmRegistry } from './armRegistry';
import {
  RawBatchRequest,
  RawForwardRequest,
  RawInverseRequest,
  RawLink,
} from './types';

const VALID_ANGLE_UNITS: AngleUnit[] = ['radian', 'degree'];
const VALID_JOINT_TYPES: JointType[] = ['revolute', 'prismatic'];

function isFiniteNumber(v: unknown): v is number {
  return typeof v === 'number' && Number.isFinite(v);
}

function requireFinite(
  v: unknown,
  linkIndex: number,
  parameter: string,
  code: 'NON_FINITE_VALUE' | 'INVALID_TYPE' | 'MISSING_FIELD' = 'NON_FINITE_VALUE',
  extraMsg?: string,
): number {
  if (v === undefined || v === null) {
    throw linkError('MISSING_FIELD', linkIndex, parameter, `第 ${linkIndex} 根杆缺少参数 '${parameter}'`);
  }
  if (typeof v !== 'number' || !Number.isFinite(v)) {
    throw linkError(
      code,
      linkIndex,
      parameter,
      `第 ${linkIndex} 根杆参数 '${parameter}' 必须是有限数值，收到 ${describe(v)}。${extraMsg ?? ''}`,
    );
  }
  return v;
}

/** 可选结构量：缺省 0；若提供则必须是有限数。 */
function optionalFinite(v: unknown, linkIndex: number, parameter: string): number {
  if (v === undefined) return 0;
  return requireFinite(v, linkIndex, parameter);
}

function describe(v: unknown): string {
  if (typeof v === 'number') return Number.isFinite(v) ? String(v) : `数值 ${v}`;
  if (v === null) return 'null';
  if (Array.isArray(v)) return '数组';
  return typeof v;
}

function parseAngleUnit(v: unknown): AngleUnit {
  if (v === undefined) return 'radian';
  if (typeof v !== 'string' || !VALID_ANGLE_UNITS.includes(v as AngleUnit)) {
    throw new ValidationError({
      code: 'INVALID_ANGLE_UNIT',
      parameter: 'angleUnit',
      message: `angleUnit 只能是 'radian' 或 'degree'，收到 ${describe(v)}`,
    });
  }
  return v as AngleUnit;
}

function toRadians(value: number, unit: AngleUnit): number {
  // 度在入口一次性换算；此后整个服务只有弧度。
  return unit === 'degree' ? (value * Math.PI) / 180 : value;
}

/**
 * 校验并物化一次正演请求。
 * registry 用于在跨请求之间识别同一 armId 的结构变更/逆序。
 */
export function validateForward(
  raw: RawForwardRequest,
  registry: ArmRegistry,
  caseIndex?: number,
): { links: MaterializedLink[]; tool: Matrix4; armId: string | null; angleUnit: AngleUnit } {
  const err = (e: ValidationError): ValidationError => {
    if (caseIndex !== undefined) {
      return new ValidationError({
        code: e.code,
        message: e.message,
        linkIndex: e.linkIndex,
        parameter: e.parameter,
        caseIndex,
        details: e.details,
      });
    }
    return e;
  };

  try {
    if (raw === null || typeof raw !== 'object' || Array.isArray(raw)) {
      throw new ValidationError({ code: 'INVALID_REQUEST', message: '请求体必须是 JSON 对象' });
    }

    const angleUnit = parseAngleUnit(raw.angleUnit);

    // links
    if (raw.links === undefined) {
      throw new ValidationError({ code: 'MISSING_FIELD', parameter: 'links', message: "缺少必填字段 'links'" });
    }
    if (!Array.isArray(raw.links)) {
      throw new ValidationError({
        code: 'INVALID_TYPE',
        parameter: 'links',
        message: `'links' 必须是连杆对象数组，收到 ${describe(raw.links)}`,
      });
    }
    const rawLinks = raw.links as RawLink[];
    if (rawLinks.length < 2) {
      throw new ValidationError({
        caseIndex,
        code: 'LINK_COUNT_INVALID',
        parameter: 'links',
        message: `开链杆件数不得少于 2，收到 ${rawLinks.length} 根`,
      });
    }

    // joints
    if (raw.joints === undefined) {
      throw new ValidationError({
        code: 'MISSING_FIELD',
        parameter: 'joints',
        message: "缺少必填字段 'joints'（关节矢量）",
      });
    }
    if (!Array.isArray(raw.joints)) {
      throw new ValidationError({
        code: 'INVALID_TYPE',
        parameter: 'joints',
        message: `'joints' 必须是长度等于杆件数的数组，收到 ${describe(raw.joints)}`,
      });
    }
    const joints = raw.joints as unknown[];
    if (joints.length !== rawLinks.length) {
      throw new ValidationError({
        caseIndex,
        code: 'JOINT_VECTOR_LENGTH_MISMATCH',
        parameter: 'joints',
        message: `关节矢量长度 ${joints.length} 与杆件数 ${rawLinks.length} 不一致`,
        details: { jointsLength: joints.length, linkCount: rawLinks.length },
      });
    }

    const allowNegativeLengths = raw.allowNegativeLengths === true;

    const links: MaterializedLink[] = rawLinks.map((rl, i) => {
      const index = i + 1;
      if (rl === null || typeof rl !== 'object' || Array.isArray(rl)) {
        throw linkError('INVALID_TYPE', index, 'link', `第 ${index} 根杆必须是 DH 参数对象`);
      }

      // 单根杆自带角度单位必须与请求级一致（不允许一部分杆用度、一部分用弧度）。
      if (rl.angleUnit !== undefined) {
        let perLinkUnit: AngleUnit;
        try {
          perLinkUnit = parseAngleUnit(rl.angleUnit);
        } catch (e) {
          if (e instanceof ValidationError) {
            throw linkError(
              'INVALID_ANGLE_UNIT',
              index,
              'alpha',
              `第 ${index} 根杆 angleUnit 非法：${e.message}`,
            );
          }
          throw e;
        }
        if (perLinkUnit !== angleUnit) {
          throw linkError(
            'ANGLE_UNIT_CONFLICT',
            index,
            'angleUnit',
            `第 ${index} 根杆声明 angleUnit='${perLinkUnit}'，与请求级 angleUnit='${angleUnit}' 矛盾：` +
              `全服务只允许一次请求统一一种角度单位。`,
            { requestUnit: angleUnit, linkUnit: perLinkUnit },
          );
        }
      }

      // jointType
      const jointTypeRaw = rl.jointType;
      if (jointTypeRaw === undefined) {
        throw linkError('MISSING_FIELD', index, 'jointType', `第 ${index} 根杆缺少 'jointType'`);
      }
      if (typeof jointTypeRaw !== 'string' || !VALID_JOINT_TYPES.includes(jointTypeRaw as JointType)) {
        throw linkError(
          'INVALID_JOINT_TYPE',
          index,
          'jointType',
          `第 ${index} 根杆 jointType 只能是 'revolute' 或 'prismatic'，收到 ${describe(jointTypeRaw)}`,
        );
      }
      const jointType = jointTypeRaw as JointType;

      // 关节矢量元素：裸数值，或 { value, freeVariable }。
      const jointRaw = joints[i];
      let jointValue: number;
      let declaredFreeVariable: string | undefined;
      if (
        jointRaw !== null &&
        typeof jointRaw === 'object' &&
        !Array.isArray(jointRaw) &&
        ('value' in (jointRaw as Record<string, unknown>) || 'freeVariable' in (jointRaw as Record<string, unknown>))
      ) {
        const jo = jointRaw as { value?: unknown; freeVariable?: unknown };
        jointValue = requireFinite(jo.value, index, `joints[${i}]`);
        if (jo.freeVariable !== undefined) {
          if (typeof jo.freeVariable !== 'string' || !['theta', 'd'].includes(jo.freeVariable)) {
            throw linkError(
              'JOINT_TYPE_FREE_VARIABLE_MISMATCH',
              index,
              `joints[${i}].freeVariable`,
              `第 ${index} 根杆的关节自由变量声明只能是 'theta' 或 'd'，收到 ${describe(jo.freeVariable)}`,
            );
          }
          declaredFreeVariable = jo.freeVariable;
        }
      } else {
        jointValue = requireFinite(jointRaw, index, `joints[${i}]`);
      }

      const expectedFreeVariable = jointType === 'revolute' ? 'theta' : 'd';
      if (declaredFreeVariable !== undefined && declaredFreeVariable !== expectedFreeVariable) {
        throw linkError(
          'JOINT_TYPE_FREE_VARIABLE_MISMATCH',
          index,
          `joints[${i}].freeVariable`,
          `第 ${index} 根杆是 ${jointType} 关节，自由变量必须是 '${expectedFreeVariable}'，` +
            `但关节元素声明驱动 '${declaredFreeVariable}'。`,
          { jointType, declaredFreeVariable, expectedFreeVariable },
        );
      }

      // 杆长 a
      const aRaw = rl.a;
      if (aRaw === undefined) {
        throw linkError('MISSING_FIELD', index, 'a', `第 ${index} 根杆缺少连杆长度 'a'`);
      }
      const a = requireFinite(aRaw, index, 'a');
      if (a < 0 && !(allowNegativeLengths || rl.allowNegativeLength === true)) {
        throw linkError(
          'NEGATIVE_LINK_LENGTH',
          index,
          'a',
          `第 ${index} 根杆连杆长度 a=${a} 为负且未声明允许（置 allowNegativeLength:true 或该杆 allowNegativeLength:true 可显式放行）`,
          { a },
        );
      }

      // 固定结构量
      const alphaDeg = optionalFinite(rl.alpha, index, 'alpha');
      const dBase = optionalFinite(rl.d, index, 'd');
      const thetaBase = optionalFinite(rl.theta, index, 'theta');

      // 自由变量归位 + 角度在入口一次性换算为弧度。
      let theta: number;
      let d: number;
      if (jointType === 'revolute') {
        theta = toRadians(jointValue, angleUnit) + toRadians(thetaBase, angleUnit);
        d = dBase;
      } else {
        theta = toRadians(thetaBase, angleUnit);
        d = dBase + jointValue; // 移动关节自由变量是偏距（长度量，不随角度单位换算）
      }
      const alpha = toRadians(alphaDeg, angleUnit);

      return { index, jointType, theta, d, a, alpha };
    });

    const tool = validateTool(raw.tool);

    let armId: string | null = null;
    if (raw.armId !== undefined) {
      if (typeof raw.armId !== 'string' || raw.armId.length === 0) {
        throw new ValidationError({
          code: 'INVALID_TYPE',
          parameter: 'armId',
          message: `armId 必须是非空字符串，收到 ${describe(raw.armId)}`,
        });
      }
      armId = raw.armId;
    }

    registry.check(armId, links);

    return { links, tool, armId, angleUnit };
  } catch (e) {
    if (e instanceof ValidationError) throw err(e);
    throw e;
  }
}

function validateTool(toolRaw: unknown): Matrix4 {
  if (toolRaw === undefined) {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ];
  }
  if (!isMatrix4Shape(toolRaw)) {
    throw new ValidationError({
      code: 'INVALID_MATRIX',
      parameter: 'tool',
      message: '工具变换 tool 必须是 4x4 数值矩阵（4 行，每行 4 个数）',
    });
  }
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      if (!Number.isFinite(toolRaw[i][j])) {
        throw new ValidationError({
          code: 'NON_FINITE_VALUE',
          parameter: `tool[${i}][${j}]`,
          message: `工具变换 tool[${i}][${j}]=${toolRaw[i][j]} 不是有限值`,
        });
      }
    }
  }
  return toolRaw as Matrix4;
}

/** 批量请求顶层校验（cases 存在且为数组）；逐组合法性在 service 中用 validateForward 判。 */
export function validateBatch(raw: RawBatchRequest): { angleUnit: AngleUnit | undefined } {
  if (raw === null || typeof raw !== 'object' || Array.isArray(raw)) {
    throw new ValidationError({ code: 'INVALID_REQUEST', message: '请求体必须是 JSON 对象' });
  }
  if (raw.cases === undefined) {
    throw new ValidationError({ code: 'MISSING_FIELD', parameter: 'cases', message: "缺少必填字段 'cases'" });
  }
  if (!Array.isArray(raw.cases) || raw.cases.length === 0) {
    throw new ValidationError({
      code: 'INVALID_TYPE',
      parameter: 'cases',
      message: "'cases' 必须是非空数组，每组是一次正演请求",
    });
  }
  let angleUnit: AngleUnit | undefined;
  if (raw.angleUnit !== undefined) angleUnit = parseAngleUnit(raw.angleUnit);
  return { angleUnit };
}

/** 平面两杆逆解入参校验。 */
export function validateInverse(raw: RawInverseRequest): { L1: number; L2: number; target: [number, number] } {
  if (raw === null || typeof raw !== 'object' || Array.isArray(raw)) {
    throw new ValidationError({ code: 'INVALID_REQUEST', message: '请求体必须是 JSON 对象' });
  }
  const scalar = (name: 'L1' | 'L2'): number => {
    const v = (raw as Record<string, unknown>)[name];
    if (v === undefined) {
      throw new ValidationError({ code: 'MISSING_FIELD', parameter: name, message: `缺少必填字段 '${name}'` });
    }
    if (!isFiniteNumber(v)) {
      throw new ValidationError({
        code: 'INVERSE_INVALID_PARAMS',
        parameter: name,
        message: `'${name}' 必须是有限数值，收到 ${describe(v)}`,
      });
    }
    if (v <= 0) {
      throw new ValidationError({
        code: 'INVERSE_INVALID_PARAMS',
        parameter: name,
        message: `'${name}' 必须为正的杆长，收到 ${v}`,
      });
    }
    return v;
  };
  const L1 = scalar('L1');
  const L2 = scalar('L2');

  const targetRaw = raw.target;
  if (targetRaw === undefined) {
    throw new ValidationError({ code: 'MISSING_FIELD', parameter: 'target', message: "缺少必填字段 'target'" });
  }
  if (
    !Array.isArray(targetRaw) ||
    targetRaw.length !== 2 ||
    !targetRaw.every((x) => typeof x === 'number')
  ) {
    throw new ValidationError({
      code: 'INVALID_TYPE',
      parameter: 'target',
      message: "'target' 必须是两个数 [x, y]",
    });
  }
  const target = targetRaw as [number, number];
  if (!Number.isFinite(target[0]) || !Number.isFinite(target[1])) {
    throw new ValidationError({
      code: 'NON_FINITE_VALUE',
      parameter: 'target',
      message: `'target' 含非有限值：[${target[0]}, ${target[1]}]`,
    });
  }
  return { L1, L2, target };
}

/** 把任意值规范成基座系 Vec3（供算例等使用）。 */
export function vec3(x: number, y: number, z: number): Vec3 {
  return [x, y, z];
}
