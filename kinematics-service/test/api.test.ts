import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import request from 'supertest';
import type { Express } from 'express';
import { createApp } from '../src/app.js';
import { SqliteHistoryRepository } from '../src/store/sqlite.js';
import { forwardKinematics } from '../src/kinematics/forward.js';
import { validateFkRequest } from '../src/validation/validate.js';

const planar3rBody = (joints: number[], a = [1.0, 0.7, 0.4]) => ({
  angleUnit: 'rad',
  links: a.map((len) => ({ jointType: 'revolute', a: len, alpha: 0, d: 0 })),
  joints,
});

let app: Express;
let store: SqliteHistoryRepository;

beforeAll(async () => {
  store = new SqliteHistoryRepository(':memory:');
  await store.init();
  app = createApp(store);
});

afterAll(async () => {
  await store.close();
});

describe('HTTP API', () => {
  it('GET /health reports ok for monitoring', async () => {
    const res = await request(app).get('/health');
    expect(res.status).toBe(200);
    expect(res.body.status).toBe('ok');
    expect(res.body.db).toBe('up');
  });

  it('GET /api/convention echoes the pinned DH convention and angle unit', async () => {
    const res = await request(app).get('/api/convention');
    expect(res.status).toBe(200);
    expect(res.body.dh).toBe('standard');
    expect(res.body.angleUnit).toBe('rad');
    expect(res.body.compositionOrder).toEqual(['RotZ(theta)', 'TransZ(d)', 'TransX(a)', 'RotX(alpha)']);
  });

  it('GET /api/example/planar3r matches the hand-computed straight-line pose', async () => {
    const res = await request(app).get('/api/example/planar3r');
    expect(res.status).toBe(200);
    expect(res.body.matchesExpectation).toBe(true);
    expect(res.body.fk.position[0]).toBeCloseTo(2.1, 12);
    expect(res.body.fk.position[1]).toBeCloseTo(0, 12);
    expect(res.body.fk.rotation).toEqual([
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ]);
  });

  it('POST /api/fk returns transform, position, euler, origins and skeleton', async () => {
    const res = await request(app).post('/api/fk').send(planar3rBody([0.2, 0.3, 0.4]));
    expect(res.status).toBe(200);
    expect(res.body.transform).toHaveLength(4);
    expect(res.body.position).toHaveLength(3);
    expect(res.body.euler.convention).toBe('ZYX');
    expect(res.body.jointOrigins).toHaveLength(3);
    expect(res.body.skeleton).toHaveLength(4);
    expect(res.body.armSignature).toMatch(/^[0-9a-f]{64}$/);
  });

  it('POST /api/fk accepts degrees and converts once', async () => {
    const deg = await request(app).post('/api/fk').send({
      angleUnit: 'deg',
      links: [
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
      ],
      joints: [90, 0],
    });
    const rad = await request(app).post('/api/fk').send({
      angleUnit: 'rad',
      links: [
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
      ],
      joints: [Math.PI / 2, 0],
    });
    expect(deg.status).toBe(200);
    for (let k = 0; k < 3; k++) expect(deg.body.position[k]).toBeCloseTo(rad.body.position[k], 9);
  });

  it('POST /api/skeleton returns the polyline', async () => {
    const res = await request(app).post('/api/skeleton').send(planar3rBody([0, 0, 0]));
    expect(res.status).toBe(200);
    expect(res.body.skeleton[0]).toEqual([0, 0, 0]);
    expect(res.body.skeleton[3][0]).toBeCloseTo(2.1, 12);
  });

  it('POST /api/ik/planar2r returns two verified branches', async () => {
    const res = await request(app).post('/api/ik/planar2r').send({ x: 1.1, y: 0.6, a1: 1.0, a2: 0.7 });
    expect(res.status).toBe(200);
    expect(res.body.solutions).toHaveLength(2);
    for (const s of res.body.solutions) expect(s.fkError).toBeLessThanOrEqual(1e-9);
  });

  it('POST /api/ik/planar2r returns 422 UNREACHABLE / SINGULAR, never fake angles', async () => {
    const far = await request(app).post('/api/ik/planar2r').send({ x: 5, y: 0, a1: 1, a2: 0.7 });
    expect(far.status).toBe(422);
    expect(far.body.error.code).toBe('UNREACHABLE');
    const collinear = await request(app).post('/api/ik/planar2r').send({ x: 1.7, y: 0, a1: 1, a2: 0.7 });
    expect(collinear.status).toBe(422);
    expect(collinear.body.error.code).toBe('SINGULAR');
  });

  it('POST /api/fk rejects illegal input with a structured, located error', async () => {
    const bad = planar3rBody([0, 0, 0]);
    bad.links[1].a = -1;
    const res = await request(app).post('/api/fk').send(bad);
    expect(res.status).toBe(400);
    expect(res.body.error.code).toBe('NEGATIVE_LINK_LENGTH');
    expect(res.body.error.details.linkIndex).toBe(1);
    expect(res.body.error.details.parameter).toBe('a');
  });

  it('POST /api/fk/batch: one bad group fails in place, the rest succeed', async () => {
    const res = await request(app).post('/api/fk/batch').send({
      items: [
        planar3rBody([0, 0, 0]),
        { angleUnit: 'rad', links: [{ jointType: 'revolute', a: 1, alpha: 0 }], joints: [0] }, // too few links
        planar3rBody([0.5, 0.5, 0.5]),
        { ...planar3rBody([0, 0, 0]), joints: [0, 0] }, // joint length mismatch
      ],
    });
    expect(res.status).toBe(200);
    expect(res.body.total).toBe(4);
    expect(res.body.succeeded).toBe(2);
    const [r0, r1, r2, r3] = res.body.results;
    expect(r0.ok).toBe(true);
    expect(r2.ok).toBe(true);
    expect(r1.ok).toBe(false);
    expect(r1.error.code).toBe('TOO_FEW_LINKS');
    expect(r1.index).toBe(1);
    expect(r3.ok).toBe(false);
    expect(r3.error.code).toBe('JOINT_LENGTH_MISMATCH');
    expect(r3.index).toBe(3);
    // good results still carry full FK output
    expect(r0.result.position[0]).toBeCloseTo(2.1, 12);
  });

  it('history persists requests and is queryable by kind and ok', async () => {
    const before = await store.count();
    await request(app).post('/api/fk').send(planar3rBody([0.1, 0.2, 0.3]));
    await request(app).post('/api/fk').send({ garbage: true });
    await request(app).post('/api/ik/planar2r').send({ x: 1, y: 0.5, a1: 1, a2: 0.7 });
    expect(await store.count()).toBe(before + 3);

    const all = await request(app).get('/api/history?limit=50');
    expect(all.status).toBe(200);
    expect(all.body.count).toBeGreaterThanOrEqual(3);

    const fkOnly = await request(app).get('/api/history?kind=fk&ok=false');
    expect(fkOnly.body.records.every((r: { kind: string; ok: boolean }) => r.kind === 'fk' && r.ok === false)).toBe(true);
    expect(fkOnly.body.records.length).toBeGreaterThanOrEqual(1);

    const ikOnly = await request(app).get('/api/history?kind=ik');
    expect(ikOnly.body.records.every((r: { kind: string }) => r.kind === 'ik')).toBe(true);
    // records round-trip the request payload
    const rec = ikOnly.body.records[0];
    expect(rec.request.a1).toBe(1);
    expect(rec.response.solutions).toHaveLength(2);
  });

  it('concurrent requests do not interfere and history stays consistent', async () => {
    const N = 30;
    const jobs = Array.from({ length: N }, (_, i) => {
      const joints = [0.01 * i, -0.02 * i, 0.03 * i];
      const expected = forwardKinematics(validateFkRequest(planar3rBody(joints)).links);
      return { joints, expected };
    });
    const before = await store.count();
    const responses = await Promise.all(
      jobs.map((j) => request(app).post('/api/fk').send(planar3rBody(j.joints))),
    );
    responses.forEach((res, i) => {
      expect(res.status).toBe(200);
      for (let k = 0; k < 3; k++) {
        expect(res.body.position[k]).toBeCloseTo(jobs[i].expected.position[k], 12);
      }
    });
    expect(await store.count()).toBe(before + N);
    const ids = new Set((await store.query({ limit: before + N + 100 })).map((r) => r.id));
    expect(ids.size).toBe(before + N);
  });
});
