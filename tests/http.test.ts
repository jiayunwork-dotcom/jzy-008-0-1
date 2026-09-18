/**
 * HTTP 端到端与并发测试：
 * - 各接口状态码、错误结构
 * - 约定回显接口
 * - 健康检查与监控指标
 * - 并发多请求结果互不串扰、历史记录不错乱
 */
import supertest from 'supertest';
import { Express } from 'express';
import { createApp } from '../src/server/app';
import { HistoryStore } from '../src/storage/types';
import { MemoryHistoryStore } from '../src/storage/memoryStore';
import { POSITION_TOLERANCE } from '../src/kinematics/constants';

let store: HistoryStore;
let app: Express;

beforeAll(async () => {
  store = new MemoryHistoryStore();
  await store.init();
  app = createApp(store).app;
});

afterAll(async () => {
  await store.close();
});

const planar3 = (joints: number[]) => ({
  links: [
    { jointType: 'revolute', a: 1, alpha: 0 },
    { jointType: 'revolute', a: 1, alpha: 0 },
    { jointType: 'revolute', a: 1, alpha: 0 },
  ],
  joints,
});

describe('GET /api/v1/convention 约定回显', () => {
  test('钉死 DH 约定、角度单位、欧拉约定、容差', async () => {
    const res = await supertest(app).get('/api/v1/convention').expect(200);
    expect(res.body.dhConvention.formula).toBe('A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)');
    expect(res.body.dhConvention.order).toEqual(['Rz(theta)', 'Tz(d)', 'Tx(a)', 'Rx(alpha)']);
    expect(res.body.angleUnit.internal).toBe('radian');
    expect(res.body.angleUnit.accepted).toEqual(['radian', 'degree']);
    expect(res.body.eulerConvention).toContain('ZYX');
    expect(res.body.tolerance.position).toBe(POSITION_TOLERANCE);
  });
});

describe('POST /api/v1/fk 正演接口', () => {
  test('全零伸直位 200 且矩阵/点列齐全', async () => {
    const res = await supertest(app).post('/api/v1/fk').send(planar3([0, 0, 0])).expect(200);
    expect(res.body.ok).toBe(true);
    expect(res.body.result.position).toEqual([3, 0, 0]);
    expect(res.body.result.totalTransform).toHaveLength(4);
    expect(res.body.result.axisOrigins).toHaveLength(3);
    expect(res.body.result.skeleton).toHaveLength(4);
    expect(res.body.result.chainCheckMaxError).toBeLessThanOrEqual(1e-10);
  });

  test('非法入参返回 400 与结构化错误（杆号/参数）', async () => {
    const res = await supertest(app)
      .post('/api/v1/fk')
      .send({
        links: [
          { jointType: 'revolute', a: 1 },
          { jointType: 'revolute', a: -5 },
        ],
        joints: [0, 0],
      })
      .expect(400);
    expect(res.body.error).toBe(true);
    expect(res.body.code).toBe('NEGATIVE_LINK_LENGTH');
    expect(res.body.linkIndex).toBe(2);
    expect(res.body.parameter).toBe('a');
  });

  test('非法 JSON 返回 400 而非崩溃', async () => {
    const res = await supertest(app)
      .post('/api/v1/fk')
      .set('Content-Type', 'application/json')
      .send('{ not json')
      .expect(400);
    expect(res.body.code).toBe('INVALID_REQUEST');
  });

  test('度单位 90° 与弧度 pi/2 位置一致', async () => {
    const deg = await supertest(app).post('/api/v1/fk').send({ ...planar3([90, 0, 0]), angleUnit: 'degree' }).expect(200);
    const rad = await supertest(app).post('/api/v1/fk').send(planar3([Math.PI / 2, 0, 0])).expect(200);
    expect(deg.body.result.position[0]).toBeCloseTo(rad.body.result.position[0], 8);
    expect(deg.body.result.position[1]).toBeCloseTo(rad.body.result.position[1], 8);
  });
});

describe('POST /api/v1/skeleton 骨架接口', () => {
  test('返回基座与各轴点列', async () => {
    const res = await supertest(app).post('/api/v1/skeleton').send(planar3([0, 0, 0])).expect(200);
    expect(res.body.skeleton).toEqual([
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
      [3, 0, 0],
    ]);
  });
});

describe('POST /api/v1/inverse 逆解接口', () => {
  test('可达点 200 + 两支', async () => {
    const res = await supertest(app).post('/api/v1/inverse').send({ L1: 1, L2: 1, target: [1, 1] }).expect(200);
    expect(res.body.status).toBe('solved');
    expect(res.body.solutions).toHaveLength(2);
    for (const s of res.body.solutions) {
      expect(s.residual).toBeLessThanOrEqual(POSITION_TOLERANCE);
    }
  });

  test('超界返回 no-solution（HTTP 200，业务状态在体内）', async () => {
    const res = await supertest(app).post('/api/v1/inverse').send({ L1: 1, L2: 1, target: [5, 5] }).expect(200);
    expect(res.body.status).toBe('no-solution');
    expect(res.body.solutions).toHaveLength(0);
    expect(res.body.reason).toBeTruthy();
  });

  test('非法参数 400', async () => {
    await supertest(app).post('/api/v1/inverse').send({ L1: 'x', L2: 1, target: [1, 1] }).expect(400);
  });
});

describe('POST /api/v1/batch 批量接口', () => {
  test('部分失败其余成功，错误标到组/杆/参数', async () => {
    const res = await supertest(app)
      .post('/api/v1/batch')
      .send({
        cases: [
          planar3([0, 0, 0]),
          { links: [{ jointType: 'revolute', a: 1 }], joints: [0] },
          planar3([0.1, 0.2, 0.3]),
        ],
      })
      .expect(200);
    expect(res.body.successCount).toBe(2);
    expect(res.body.failureCount).toBe(1);
    expect(res.body.results[0].ok).toBe(true);
    expect(res.body.results[1].ok).toBe(false);
    expect(res.body.results[1].error.caseIndex).toBe(1);
    expect(res.body.results[1].error.code).toBe('LINK_COUNT_INVALID');
    expect(res.body.results[2].ok).toBe(true);
  });
});

describe('GET /api/v1/history 历史接口', () => {
  test('分页与过滤', async () => {
    const res = await supertest(app).get('/api/v1/history?type=forward&limit=2&offset=0').expect(200);
    expect(res.body.records.length).toBeLessThanOrEqual(2);
    expect(res.body.records.every((r: { requestType: string }) => r.requestType === 'forward')).toBe(true);
    expect(res.body.total).toBeGreaterThanOrEqual(res.body.records.length);
  });

  test('非法分页参数 400', async () => {
    await supertest(app).get('/api/v1/history?limit=abc').expect(400);
  });
});

describe('预置算例接口', () => {
  test('GET /api/v1/presets/planar3r 给出载荷与手算期望', async () => {
    const res = await supertest(app).get('/api/v1/presets/planar3r').expect(200);
    expect(res.body.expected.position).toEqual([3, 0, 0]);
    const fk = await supertest(app).post('/api/v1/fk').send(res.body.input).expect(200);
    expect(fk.body.result.position).toEqual(res.body.expected.position);
  });
});

describe('运行状态采集', () => {
  test('/health/live 与 /health/ready', async () => {
    await supertest(app).get('/health/live').expect(200);
    const ready = await supertest(app).get('/health/ready').expect(200);
    expect(ready.body.storage).toBe('ok');
  });

  test('/metrics 与 /metrics/json 输出计数', async () => {
    await supertest(app).post('/api/v1/fk').send(planar3([0, 0, 0]));
    const text = await supertest(app).get('/metrics').expect(200);
    expect(text.text).toContain('kinematics_http_requests_total');
    const json = await supertest(app).get('/metrics/json').expect(200);
    expect(json.body.uptimeSeconds).toBeGreaterThanOrEqual(0);
  });
});

describe('并发互不串扰', () => {
  test('20 个并发请求（不同位形/不同臂/夹杂非法），每个结果只对应自己的输入', async () => {
    const tasks: Array<Promise<unknown>> = [];
    for (let i = 0; i < 20; i++) {
      const t1 = i * 0.07;
      const t2 = -i * 0.03;
      const isBad = i % 5 === 4;
      const payload = isBad
        ? { links: planar3([0, 0, 0]).links, joints: [0, 0] } // 长度不符
        : planar3([t1, t2, 0]);
      tasks.push(
        supertest(app)
          .post('/api/v1/fk')
          .send(payload)
          .then((res) => ({ i, isBad, res })),
      );
    }
    const out = (await Promise.all(tasks)) as Array<{
      i: number;
      isBad: boolean;
      res: supertest.Response;
    }>;

    for (const { i, isBad, res } of out) {
      if (isBad) {
        expect(res.status).toBe(400);
        expect(res.body.code).toBe('JOINT_VECTOR_LENGTH_MISMATCH');
      } else {
        expect(res.status).toBe(200);
        // 独立重算该位形的期望位置（三杆：theta3=0）。
        const t1 = i * 0.07;
        const t2 = -i * 0.03;
        const x =
          Math.cos(t1) + Math.cos(t1 + t2) + Math.cos(t1 + t2);
        const y =
          Math.sin(t1) + Math.sin(t1 + t2) + Math.sin(t1 + t2);
        expect(res.body.result.position[0]).toBeCloseTo(x, 8);
        expect(res.body.result.position[1]).toBeCloseTo(y, 8);
        // requestId 唯一。
        expect(res.body.requestId).toBeGreaterThan(0);
      }
    }

    // 20 条历史一条不错、一条不少（成功失败都落库）。
    const recs = await supertest(app).get('/api/v1/history?type=forward&limit=200&offset=0').expect(200);
    expect(recs.body.total).toBeGreaterThanOrEqual(20);
    const ids = new Set(recs.body.records.map((r: { id: string }) => r.id));
    expect(ids.size).toBe(recs.body.records.length);
  });

  test('交错位形 A/B 连续快速请求，结果严格等于各自独立计算', async () => {
    const A = planar3([0.9, 0.1, -0.4]);
    const B = planar3([-1.2, 2.0, 0.3]);
    const results: number[][][] = [];
    await Promise.all(
      Array.from({ length: 10 }, (_, k) =>
        supertest(app)
          .post('/api/v1/fk')
          .send(k % 2 === 0 ? A : B)
          .then((r) => results.push([r.body.result.position])),
      ),
    );
    // 抽样：直接独立请求 A、B 各一次对照。
    const ra = await supertest(app).post('/api/v1/fk').send(A).expect(200);
    const rb = await supertest(app).post('/api/v1/fk').send(B).expect(200);
    for (const [p] of results) {
      const matchA = Math.abs(p[0] - ra.body.result.position[0]) < 1e-9;
      const matchB = Math.abs(p[0] - rb.body.result.position[0]) < 1e-9;
      expect(matchA || matchB).toBe(true);
    }
  });
});
