/**
 * 运动学核算编排层：校验 → 纯函数计算 → 结构化响应 → 历史持久化。
 *
 * 计算部分全部调用 src/kinematics 的无状态纯函数，每次请求独立物化，
 * 因而并发请求之间不会共享任何中间矩阵。
 */
import { ForwardKinematicsOutput } from '../kinematics/chain';
import { forwardKinematics } from '../kinematics/chain';
import { planar2RInverse } from '../kinematics/inverse';
import { identity } from '../kinematics/math';
import { DH_CONVENTION, MATRIX_TOLERANCE, POSITION_TOLERANCE } from '../kinematics/constants';
import { ArmRegistry } from '../validation/armRegistry';
import { validateBatch, validateForward, validateInverse } from '../validation/validator';
import { ValidationError } from '../validation/errors';
import {
  RawBatchRequest,
  RawForwardRequest,
  RawInverseRequest,
} from '../validation/types';
import {
  HistoryRequestType,
  HistoryStatus,
  HistoryStore,
  StoredHistoryRecord,
} from '../storage/types';

export interface ServiceContext {
  store: HistoryStore;
  registry: ArmRegistry;
  /** 单调递增的服务内请求序号（原子：JS 单线程，自增即可）。 */
  nextRequestId(): number;
}

export class KinematicsService {
  constructor(private readonly ctx: ServiceContext) {}

  /** 单臂正演。 */
  async forward(raw: RawForwardRequest): Promise<ForwardSuccessBody> {
    const started = nowMs();
    const requestId = this.ctx.nextRequestId();
    let status: HistoryStatus = 'ok';
    let errorCode: string | null = null;
    let linkCount: number | null = null;
    let body: ForwardSuccessBody;

    try {
      const req = validateForward(raw, this.ctx.registry);
      const fk = forwardKinematics(req.links, req.tool);
      linkCount = fk.linkCount;
      body = serializeFk(fk, req.armId, req.angleUnit, requestId);
    } catch (e) {
      status = 'error';
      const ve = asValidationError(e);
      errorCode = ve.code;
      await this.persist('forward', { status, errorCode, linkCount, raw, errorBody: ve.toJSON(), started });
      throw ve;
    }

    await this.persist('forward', { status, errorCode, linkCount, raw, successBody: body, started });
    return body;
  }

  /** 骨架点列（与正演同一次链乘，只返回骨架相关字段）。 */
  async skeleton(raw: RawForwardRequest): Promise<SkeletonSuccessBody> {
    const started = nowMs();
    const requestId = this.ctx.nextRequestId();
    let status: HistoryStatus = 'ok';
    let errorCode: string | null = null;
    let linkCount: number | null = null;
    let body: SkeletonSuccessBody;

    try {
      const req = validateForward(raw, this.ctx.registry);
      const fk = forwardKinematics(req.links, req.tool);
      linkCount = fk.linkCount;
      body = {
        ok: true,
        requestId,
        armId: req.armId,
        angleUnit: req.angleUnit,
        linkCount: fk.linkCount,
        endPosition: fk.position,
        axisOrigins: fk.axisOrigins,
        skeleton: fk.skeleton,
        chainCheckMaxError: fk.chainCheckMaxError,
      };
    } catch (e) {
      status = 'error';
      const ve = asValidationError(e);
      errorCode = ve.code;
      await this.persist('skeleton', { status, errorCode, linkCount, raw, errorBody: ve.toJSON(), started });
      throw ve;
    }

    await this.persist('skeleton', { status, errorCode, linkCount, raw, successBody: body, started });
    return body;
  }

  /** 批量正演：逐组独立校验计算，某组非法不影响其余组。 */
  async batch(raw: RawBatchRequest): Promise<BatchSuccessBody> {
    const started = nowMs();
    const requestId = this.ctx.nextRequestId();

    let topAngleUnit: 'radian' | 'degree' | undefined;
    try {
      topAngleUnit = validateBatch(raw).angleUnit;
    } catch (e) {
      const ve = asValidationError(e);
      await this.persist('batch', {
        status: 'error',
        errorCode: ve.code,
        caseCount: Array.isArray(raw?.cases) ? raw.cases.length : null,
        raw,
        errorBody: ve.toJSON(),
        started,
      });
      throw ve;
    }

    const cases = raw.cases as RawForwardRequest[];
    const results: BatchCaseResult[] = [];
    let successCount = 0;
    let failureCount = 0;
    // 注意：每个 case 必须独立注册核对——非法 case 抛错时不能影响已成功 case 的登记，
    // 因此校验中失败的 arm 登记本来就没有写入（registry.check 只在成功物化后才到达）。
    cases.forEach((c, i) => {
      const name = typeof c?.name === 'string' ? (c.name as string) : null;
      try {
        const merged: RawForwardRequest =
          topAngleUnit !== undefined && c.angleUnit === undefined ? { ...c, angleUnit: topAngleUnit } : c;
        const req = validateForward(merged, this.ctx.registry, i);
        const fk = forwardKinematics(req.links, req.tool);
        const body = serializeFk(fk, req.armId, req.angleUnit, this.ctx.nextRequestId());
        successCount++;
        results.push({ index: i, name, ok: true, result: body.result });
      } catch (e) {
        failureCount++;
        const ve = asValidationError(e, i);
        results.push({ index: i, name, ok: false, error: ve.toJSON() });
      }
    });

    const body: BatchSuccessBody = {
      ok: true,
      requestId,
      total: cases.length,
      successCount,
      failureCount,
      angleUnit: topAngleUnit ?? 'radian',
      results,
    };

    await this.persist('batch', {
      status: failureCount > 0 ? 'error' : 'ok',
      errorCode: failureCount > 0 ? 'BATCH_PARTIAL_OR_ALL_FAILED' : null,
      caseCount: cases.length,
      raw,
      successBody: body,
      started,
    });
    return body;
  }

  /** 平面两杆逆解。 */
  async inverse(raw: RawInverseRequest): Promise<InverseSuccessBody> {
    const started = nowMs();
    const requestId = this.ctx.nextRequestId();
    let body: InverseSuccessBody;
    try {
      const { L1, L2, target } = validateInverse(raw);
      const ik = planar2RInverse({ L1, L2, target });
      body = {
        ok: true,
        requestId,
        target: ik.target,
        L1,
        L2,
        status: ik.status,
        reason: ik.reason,
        solutions: ik.solutions,
        tolerance: POSITION_TOLERANCE,
      };
    } catch (e) {
      const ve = asValidationError(e);
      await this.persist('inverse', {
        status: 'error',
        errorCode: ve.code,
        raw,
        errorBody: ve.toJSON(),
        started,
      });
      throw ve;
    }

    await this.persist('inverse', {
      status: body.status === 'solved' ? 'ok' : 'error',
      errorCode: body.status === 'solved' ? null : `IK_${body.status.toUpperCase()}`,
      raw,
      successBody: body,
      started,
    });
    return body;
  }

  /** 历史查询。 */
  async history(filter: {
    type?: string;
    status?: string;
    armId?: string;
    since?: unknown;
    until?: unknown;
    limit?: unknown;
    offset?: unknown;
  }): Promise<{ total: number; limit: number; offset: number; records: StoredHistoryRecord[] }> {
    const parseEpoch = (v: unknown, name: string): number | undefined => {
      if (v === undefined) return undefined;
      const n = Number(v);
      if (!Number.isFinite(n)) {
        throw new ValidationError({
          code: 'INVALID_TYPE',
          parameter: name,
          message: `查询参数 '${name}' 必须是 epoch 毫秒数值`,
        });
      }
      return n;
    };
    const limit = clampInt(filter.limit, 50, 1, 500, 'limit');
    const offset = clampInt(filter.offset, 0, 0, 1_000_000, 'offset');
    const f = {
      type: filter.type,
      status: filter.status,
      armId: filter.armId,
      sinceEpochMs: parseEpoch(filter.since, 'since'),
      untilEpochMs: parseEpoch(filter.until, 'until'),
      limit,
      offset,
    };
    if (f.type && !['forward', 'skeleton', 'batch', 'inverse'].includes(f.type)) {
      throw new ValidationError({
        code: 'INVALID_TYPE',
        parameter: 'type',
        message: "type 只能是 forward | skeleton | batch | inverse",
      });
    }
    if (f.status && !['ok', 'error'].includes(f.status)) {
      throw new ValidationError({
        code: 'INVALID_TYPE',
        parameter: 'status',
        message: "status 只能是 ok | error",
      });
    }
    const [records, total] = await Promise.all([
      this.ctx.store.query(f),
      this.ctx.store.count({
        type: f.type,
        status: f.status,
        armId: f.armId,
        sinceEpochMs: f.sinceEpochMs,
        untilEpochMs: f.untilEpochMs,
      }),
    ]);
    return { total, limit, offset, records };
  }

  /** 预置算例：平面三杆、杆长 1/1/1、全零关节角。 */
  presetPlanar3R(): { description: string; input: RawForwardRequest; expected: unknown } {
    const input: RawForwardRequest = {
      armId: 'preset-planar-3r',
      angleUnit: 'radian',
      links: [
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
      ],
      joints: [0, 0, 0],
    };
    return {
      description:
        '平面三转动关节臂，L1=L2=L3=1，关节角全零（弧度）。手算伸直位：末端位置 [3,0,0]，姿态无扭转（单位旋转）。',
      input,
      expected: {
        position: [3, 0, 0],
        rotation: identity().slice(0, 3).map((r) => r.slice(0, 3)),
        euler: { roll: 0, pitch: 0, yaw: 0 },
        skeleton: [
          [0, 0, 0],
          [1, 0, 0],
          [2, 0, 0],
          [3, 0, 0],
        ],
      },
    };
  }

  private async persist(
    requestType: HistoryRequestType,
    p: {
      status: HistoryStatus;
      errorCode: string | null;
      linkCount?: number | null;
      caseCount?: number | null;
      raw: unknown;
      successBody?: unknown;
      errorBody?: unknown;
      started: number;
    },
  ): Promise<void> {
    await this.ctx.store.insert({
      requestType,
      status: p.status,
      armId: armIdOf(p.raw),
      request: p.raw,
      response: p.status === 'ok' ? p.successBody : { error: p.errorBody },
      linkCount: p.linkCount ?? null,
      caseCount: p.caseCount ?? null,
      errorCode: p.errorCode,
      durationMs: nowMs() - p.started,
    });
  }
}

function serializeFk(
  fk: ForwardKinematicsOutput,
  armId: string | null,
  angleUnit: 'radian' | 'degree',
  requestId: number,
): ForwardSuccessBody {
  return {
    ok: true,
    requestId,
    armId,
    angleUnit,
    convention: DH_CONVENTION.name,
    result: {
      linkCount: fk.linkCount,
      totalTransform: fk.totalTransform,
      endEffectorTransform: fk.endEffectorTransform,
      position: fk.position,
      rotation: fk.rotation,
      euler: fk.euler,
      links: fk.links.map((l) => ({
        index: l.index,
        jointType: l.jointType,
        transform: l.transform,
        origin: l.origin,
      })),
      axisOrigins: fk.axisOrigins,
      skeleton: fk.skeleton,
      tool: fk.tool,
      chainCheckMaxError: fk.chainCheckMaxError,
      chainCheckTolerance: MATRIX_TOLERANCE,
    },
  };
}

/**
 * 把捕获到的异常统一成 ValidationError 再向上抛（HTTP 层据此返回 4xx）。
 * 若已有 caseIndex 则补齐（批量场景）；非校验类异常包装为 INTERNAL 语义的校验错误不可取，
 * 这里保留原消息但用 INVALID_REQUEST，由错误中间件区分 500。
 */
function asValidationError(e: unknown, caseIndex?: number): ValidationError {
  if (e instanceof ValidationError) {
    if (caseIndex !== undefined && e.caseIndex === undefined) {
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
  }
  const message = e instanceof Error ? e.message : String(e);
  return new ValidationError({
    code: 'INVALID_REQUEST',
    message: `计算失败：${message}`,
    ...(caseIndex !== undefined ? { caseIndex } : {}),
  });
}

function armIdOf(raw: unknown): string | null {
  if (raw && typeof raw === 'object') {
    const id = (raw as { armId?: unknown }).armId;
    if (typeof id === 'string') return id;
  }
  return null;
}

function clampInt(v: unknown, dflt: number, min: number, max: number, name: string): number {
  if (v === undefined) return dflt;
  const n = Number(v);
  if (!Number.isInteger(n)) {
    throw new ValidationError({
      code: 'INVALID_TYPE',
      parameter: name,
      message: `查询参数 '${name}' 必须是整数`,
    });
  }
  return Math.max(min, Math.min(max, n));
}

function nowMs(): number {
  const hr = process.hrtime.bigint();
  return Number(hr / 1_000_000n);
}

export interface ForwardSuccessBody {
  ok: true;
  requestId: number;
  armId: string | null;
  angleUnit: 'radian' | 'degree';
  convention: string;
  result: {
    linkCount: number;
    totalTransform: number[][];
    endEffectorTransform: number[][];
    position: [number, number, number];
    rotation: number[][];
    euler: { roll: number; pitch: number; yaw: number };
    links: Array<{
      index: number;
      jointType: 'revolute' | 'prismatic';
      transform: number[][];
      origin: [number, number, number];
    }>;
    axisOrigins: [number, number, number][];
    skeleton: [number, number, number][];
    tool: number[][];
    chainCheckMaxError: number;
    chainCheckTolerance: number;
  };
}

export interface SkeletonSuccessBody {
  ok: true;
  requestId: number;
  armId: string | null;
  angleUnit: 'radian' | 'degree';
  linkCount: number;
  endPosition: [number, number, number];
  axisOrigins: [number, number, number][];
  skeleton: [number, number, number][];
  chainCheckMaxError: number;
}

export type BatchCaseResult =
  | { index: number; name: string | null; ok: true; result: ForwardSuccessBody['result'] }
  | { index: number; name: string | null; ok: false; error: ReturnType<ValidationError['toJSON']> };

export interface BatchSuccessBody {
  ok: true;
  requestId: number;
  total: number;
  successCount: number;
  failureCount: number;
  angleUnit: 'radian' | 'degree';
  results: BatchCaseResult[];
}

export interface InverseSuccessBody {
  ok: true;
  requestId: number;
  target: [number, number];
  L1: number;
  L2: number;
  status: 'solved' | 'no-solution' | 'singular';
  reason: string | null;
  solutions: Array<{
    branch: 'elbow-up' | 'elbow-down';
    joints: [number, number];
    fkPosition: [number, number, number];
    residual: number;
  }>;
  tolerance: number;
}
