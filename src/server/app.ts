/**
 * Express 应用：路由与统一错误处理。
 * 应用只负责 HTTP 适配，所有计算都在 service / kinematics 层。
 */
import express, { Express, Request, Response, NextFunction } from 'express';
import { KinematicsService } from '../services/kinematicsService';
import { ValidationError } from '../validation/errors';
import { HistoryStore } from '../storage/types';
import { ArmRegistry } from '../validation/armRegistry';
import { DH_CONVENTION, EULER_CONVENTION, MATRIX_TOLERANCE, POSITION_TOLERANCE } from '../kinematics/constants';
import { Metrics } from './metrics';

export interface CreatedApp {
  app: Express;
  service: KinematicsService;
  metrics: Metrics;
}

export function createApp(store: HistoryStore): CreatedApp {
  const app = express();
  app.use(express.json({ limit: '5mb' }));

  const metrics = new Metrics();
  let requestSeq = 0;
  const service = new KinematicsService({
    store,
    registry: new ArmRegistry(),
    nextRequestId: () => ++requestSeq,
  });

  // 请求计数与结束指标：必须在路由之前注册，保证每个请求（含错误）都能触发 finish 钩子。
  app.use((req, res, next) => {
    metrics.requestStarted();
    res.on('finish', () => {
      if (req.path.startsWith('/api') || req.path.startsWith('/health') || req.path.startsWith('/metrics')) {
        metrics.requestFinished(res.statusCode);
      }
    });
    next();
  });

  /** 约定回显接口：当前 DH 约定与角度单位（钉死，不可配置）。 */
  app.get('/api/v1/convention', (_req, res) => {
    res.json({
      dhConvention: DH_CONVENTION,
      angleUnit: {
        internal: 'radian',
        accepted: ['radian', 'degree'],
        rule: 'degree 仅在请求入口一次性换算为弧度后参与三角函数；同一请求所有连杆必须使用同一单位',
      },
      eulerConvention: EULER_CONVENTION,
      toolTransform: 'T_end = (A_1...A_n) * T_tool，T_tool 缺省为单位阵',
      tolerance: { matrix: MATRIX_TOLERANCE, position: POSITION_TOLERANCE },
      joints: {
        revolute: '自由变量为关节角 theta（弧度/按 angleUnit 换算）',
        prismatic: '自由变量为连杆偏距 d（长度增量）',
      },
    });
  });

  /** 预置算例回显（不计算，仅给出可直接提交给正演接口的载荷与手算期望值）。 */
  app.get('/api/v1/presets/planar3r', (_req, res) => {
    res.json(service.presetPlanar3R());
  });

  app.post('/api/v1/fk', asyncHandler(async (req, res) => {
    const body = await service.forward(req.body);
    res.json(body);
  }));

  app.post('/api/v1/skeleton', asyncHandler(async (req, res) => {
    const body = await service.skeleton(req.body);
    res.json(body);
  }));

  app.post('/api/v1/batch', asyncHandler(async (req, res) => {
    const body = await service.batch(req.body);
    // 批量的 HTTP 状态恒为 200：逐组成败在 results 内表达（部分失败不影响其余组）。
    res.json(body);
  }));

  app.post('/api/v1/inverse', asyncHandler(async (req, res) => {
    const body = await service.inverse(req.body);
    // 逆解 status 为 no-solution/singular 属于正常核算结果，HTTP 仍返回 200，
    // 由响应体内的 status 区分；只有入参非法才是 4xx。
    res.json(body);
  }));

  app.get('/api/v1/history', asyncHandler(async (req, res) => {
    const body = await service.history(req.query as never);
    res.json({ ok: true, ...body });
  }));

  /** 存活探针。 */
  app.get('/health/live', (_req, res) => {
    res.json({ status: 'alive', uptimeSeconds: metrics.uptimeSeconds() });
  });

  /** 就绪探针：存储可查询即就绪。 */
  app.get('/health/ready', asyncHandler(async (_req, res) => {
    const n = await store.count();
    res.json({ status: 'ready', storage: 'ok', historyRecords: n });
  }));

  /** 运行指标（JSON）。 */
  app.get('/metrics/json', (_req, res) => {
    res.json(metrics.snapshot());
  });

  /** 运行指标（Prometheus 文本）。 */
  app.get('/metrics', (_req, res) => {
    res.type('text/plain; version=0.0.4').send(metrics.prometheus());
  });

  // JSON 解析错误（请求体不是合法 JSON）。
  app.use((err: unknown, req: Request, res: Response, next: NextFunction) => {
    if (err instanceof SyntaxError && 'body' in (err as unknown as Record<string, unknown>)) {
      respondError(res, 400, {
        error: true,
        code: 'INVALID_REQUEST',
        message: `请求体不是合法 JSON：${err.message}`,
      });
 } else {
      next(err);
    }
  });

  // 统一错误处理。
  app.use((err: unknown, _req: Request, res: Response, _next: NextFunction) => {
    if (err instanceof ValidationError) {
      respondError(res, 400, err.toJSON());
      return;
    }
    const message = err instanceof Error ? err.message : String(err);
    respondError(res, 500, {
      error: true,
      code: 'INTERNAL_ERROR',
      message: `服务内部错误：${message}`,
    });
  });

  return { app, service, metrics };
}

/** 把 async 路由抛出的拒绝交给错误中间件。 */
function asyncHandler(
  handler: (req: Request, res: Response, next: NextFunction) => Promise<void>,
) {
  return (req: Request, res: Response, next: NextFunction): void => {
    handler(req, res, next).catch(next);
  };
}

function respondError(res: Response, status: number, body: unknown): void {
  res.status(status).json(body);
}
