/**
 * 服务编排测试：
 * - 批量正演某组非法时指出第几组/哪根杆/哪个参数，其余组照常成功
 * - 预置平面三杆算例与手算伸直位一致
 * - 每次请求都持久化（成功/失败都可查），且可按条件过滤
 * - 工具变换经 service 也生效
 */
import { makeService, planarChain, vecApprox } from './helpers';
import { ValidationError } from '../src/validation/errors';
import { POSITION_TOLERANCE } from '../src/kinematics/constants';

describe('批量正演', () => {
  test('部分组非法：标出 caseIndex/linkIndex/parameter，合法组正常返回', async () => {
    const { service, store } = await makeService();
    const body = await service.batch({
      cases: [
        planarChain([1, 1], [0, 0]),
        planarChain([1, 1, 1], [0, 0, 0]),
        {
          // 非法：关节矢量长度不符
          links: [
            { jointType: 'revolute', a: 1 },
            { jointType: 'revolute', a: 1 },
          ],
          joints: [0],
        },
        {
          // 非法：第 2 根杆 a 为负
          links: [
            { jointType: 'revolute', a: 1 },
            { jointType: 'revolute', a: -2 },
          ],
          joints: [0, 0],
        },
        planarChain([1, 1], [Math.PI / 2, 0]),
      ],
    });

    expect(body.total).toBe(5);
    expect(body.successCount).toBe(3);
    expect(body.failureCount).toBe(2);

    expect(body.results[0].ok).toBe(true);
    expect(body.results[1].ok).toBe(true);
    expect(body.results[4].ok).toBe(true);
    if (body.results[4].ok) {
      expect(vecApprox(body.results[4].result.position, [0, 2, 0], POSITION_TOLERANCE)).toBe(true);
    }

    expect(body.results[2].ok).toBe(false);
    if (!body.results[2].ok) {
      expect(body.results[2].error.caseIndex).toBe(2);
      expect(body.results[2].error.code).toBe('JOINT_VECTOR_LENGTH_MISMATCH');
    }

    expect(body.results[3].ok).toBe(false);
    if (!body.results[3].ok) {
      expect(body.results[3].error.caseIndex).toBe(3);
      expect(body.results[3].error.linkIndex).toBe(2);
      expect(body.results[3].error.parameter).toBe('a');
      expect(body.results[3].error.code).toBe('NEGATIVE_LINK_LENGTH');
    }

    await store.close();
  });

  test('顶层 cases 缺失/为空抛 ValidationError', async () => {
    const { service, store } = await makeService();
    await expect(service.batch({ cases: [] })).rejects.toBeInstanceOf(ValidationError);
    await expect(service.batch({} as never)).rejects.toBeInstanceOf(ValidationError);
    await store.close();
  });

  test('批量顶层 angleUnit 对未单独声明单位的组生效（度）', async () => {
    const { service, store } = await makeService();
    const links = [
      { jointType: 'revolute' as const, a: 1, alpha: 0 },
      { jointType: 'revolute' as const, a: 1, alpha: 0 },
    ];
    const body = await service.batch({
      angleUnit: 'degree',
      cases: [
        { links, joints: [90, 0] },
        { links, joints: [180, 0] },
      ],
    });
    expect(body.successCount).toBe(2);
    // 扭角缺省 0：90° 伸直朝上，180° 伸直朝左（θ 正为逆时针）。
    if (body.results[0].ok) expect(vecApprox(body.results[0].result.position, [0, 2, 0], POSITION_TOLERANCE)).toBe(true);
    if (body.results[1].ok) expect(vecApprox(body.results[1].result.position, [-2, 0, 0], POSITION_TOLERANCE)).toBe(true);
    await store.close();
  });
});

describe('预置算例（平面三杆伸直位）', () => {
  test('直接计算预置载荷得到手算 [3,0,0]，与 expected 一致', async () => {
    const { service, store } = await makeService();
    const preset = service.presetPlanar3R();
    const fk = await service.forward(preset.input);
    const expected = preset.expected as {
      position: number[];
      skeleton: number[][];
    };
    expect(vecApprox(fk.result.position, expected.position as [number, number, number], POSITION_TOLERANCE)).toBe(true);
    expect(fk.result.rotation).toEqual([
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ]);
    expect(fk.result.skeleton).toEqual(expected.skeleton);
    await store.close();
  });
});

describe('历史持久化', () => {
  test('成功与失败请求都落库，并可按 type/status/armId 过滤', async () => {
    const { service, store } = await makeService();

    await service.forward({ ...planarChain([1, 1], [0, 0]), armId: 'hist-a' });
    await service.forward({ ...planarChain([1, 1, 1], [0.1, 0.2, 0.3]), armId: 'hist-b' });
    await expect(
      service.forward({ links: [{ jointType: 'revolute', a: 1 }], joints: [0] }),
    ).rejects.toBeInstanceOf(ValidationError);
    const inv = await service.inverse({ L1: 1, L2: 1, target: [3, 0] });
    expect(inv.status).toBe('no-solution');

    const all = await service.history({ limit: 100, offset: 0 });
    expect(all.total).toBe(4);
    // 最新在前。
    expect(all.records[0].requestType).toBe('inverse');

    const fwd = await service.history({ type: 'forward', limit: 100, offset: 0 });
    expect(fwd.total).toBe(3);
    expect(fwd.records.every((r) => r.requestType === 'forward')).toBe(true);

    const errs = await service.history({ status: 'error', limit: 100, offset: 0 });
    // 非法正演 + no-solution 的逆解（核算结果非 solved 记为 error）
    expect(errs.total).toBe(2);

    const aRecs = await service.history({ armId: 'hist-a', limit: 100, offset: 0 });
    expect(aRecs.total).toBe(1);
    expect(aRecs.records[0].armId).toBe('hist-a');

    // 请求/响应快照内容可追溯（按 id 倒序取该臂自己的记录最稳妥）。
    const bRecs = await service.history({ armId: 'hist-b', limit: 1, offset: 0 });
    const latest = bRecs.records[0];
    expect((latest.request as { joints?: number[] }).joints).toEqual([0.1, 0.2, 0.3]);
    expect((latest.response as { ok: boolean }).ok).toBe(true);
    expect(latest.linkCount).toBe(3);

    await store.close();
  });

  test('历史记录内容完整（请求快照/响应快照/杆数/耗时/时间戳）', async () => {
    const { service, store } = await makeService();
    await service.forward({ ...planarChain([1, 1], [0, 0]), armId: 'persist-me' });
    const recs = await service.history({ armId: 'persist-me', limit: 10, offset: 0 });
    expect(recs.total).toBe(1);
    const r = recs.records[0];
    expect(r.request).toBeTruthy();
    expect((r.request as { armId: string }).armId).toBe('persist-me');
    expect((r.response as { ok: boolean }).ok).toBe(true);
    expect(r.linkCount).toBe(2);
    expect(r.durationMs).toBeGreaterThanOrEqual(0);
    expect(Number.isFinite(Date.parse(r.createdAt))).toBe(true);
    expect(r.id).toBeTruthy();
    await store.close();
  });

  test('批量整体也写一条历史，caseCount 等于组数', async () => {
    const { service, store } = await makeService();
    await service.batch({
      cases: [planarChain([1, 1], [0, 0]), planarChain([1, 1], [0.1, 0.2])],
    });
    const batchRecs = await service.history({ type: 'batch', limit: 10, offset: 0 });
    expect(batchRecs.total).toBe(1);
    expect(batchRecs.records[0].caseCount).toBe(2);
    expect(batchRecs.records[0].status).toBe('ok');
    await store.close();
  });
});
