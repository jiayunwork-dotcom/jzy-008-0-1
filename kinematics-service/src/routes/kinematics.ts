import { randomUUID } from 'node:crypto';
import { Router } from 'express';
import { KinematicsError } from '../errors.js';
import { forwardKinematics } from '../kinematics/forward.js';
import { inversePlanar2R } from '../kinematics/inverse2r.js';
import { runDemo } from '../example.js';
import { validateFkRequest } from '../validation/validate.js';
import type { HistoryRecord, HistoryRepository, RecordKind } from '../store/repository.js';

async function persist(store: HistoryRepository, kind: RecordKind, ok: boolean, request: unknown, response: unknown): Promise<void> {
  const record: HistoryRecord = {
    id: randomUUID(),
    kind,
    ok,
    createdAt: new Date().toISOString(),
    request,
    response,
  };
  await store.save(record);
}

export function kinematicsRouter(store: HistoryRepository): Router {
  const r = Router();

  // --- single-arm forward kinematics ---------------------------------------
  r.post('/fk', async (req, res, next) => {
    try {
      const chain = validateFkRequest(req.body);
      const result = forwardKinematics(chain.links, chain.tool);
      const body = { ...result, armSignature: chain.signature };
      await persist(store, 'fk', true, req.body, body);
      res.json(body);
    } catch (err) {
      if (err instanceof KinematicsError) await persist(store, 'fk', false, req.body, err.toJSON());
      next(err);
    }
  });

  // --- skeleton polyline -----------------------------------------------------
  r.post('/skeleton', async (req, res, next) => {
    try {
      const chain = validateFkRequest(req.body);
      const result = forwardKinematics(chain.links, chain.tool);
      const body = { skeleton: result.skeleton, jointOrigins: result.jointOrigins, armSignature: chain.signature };
      await persist(store, 'skeleton', true, req.body, body);
      res.json(body);
    } catch (err) {
      if (err instanceof KinematicsError) await persist(store, 'skeleton', false, req.body, err.toJSON());
      next(err);
    }
  });

  // --- planar 2R inverse kinematics ------------------------------------------
  r.post('/ik/planar2r', async (req, res, next) => {
    try {
      const { x, y, a1, a2 } = (req.body ?? {}) as Record<string, unknown>;
      for (const [name, v] of Object.entries({ x, y, a1, a2 })) {
        if (v === undefined) throw new KinematicsError('MISSING_FIELD', `Missing required field "${name}"`, { parameter: name });
      }
      const result = inversePlanar2R(x as number, y as number, a1 as number, a2 as number);
      await persist(store, 'ik', true, req.body, result);
      res.json(result);
    } catch (err) {
      if (err instanceof KinematicsError) await persist(store, 'ik', false, req.body, err.toJSON());
      next(err);
    }
  });

  // --- batch forward kinematics: one bad item never sinks the rest -----------
  r.post('/fk/batch', async (req, res, next) => {
    try {
      const items = (req.body as { items?: unknown })?.items;
      if (!Array.isArray(items) || items.length === 0) {
        throw new KinematicsError('MISSING_FIELD', 'Request body must contain a non-empty "items" array of FK requests', { parameter: 'items' });
      }
      const results = items.map((item, index) => {
        try {
          const chain = validateFkRequest(item);
          const result = forwardKinematics(chain.links, chain.tool);
          return { index, ok: true as const, result: { ...result, armSignature: chain.signature } };
        } catch (err) {
          const e = err instanceof KinematicsError
            ? err.toJSON()
            : { code: 'INTERNAL', message: String(err), details: {} };
          return { index, ok: false as const, error: e };
        }
      });
      const body = { total: results.length, succeeded: results.filter((x) => x.ok).length, results };
      await persist(store, 'fk_batch', true, req.body, body);
      res.json(body);
    } catch (err) {
      next(err);
    }
  });

  // --- history query -----------------------------------------------------------
  r.get('/history', async (req, res, next) => {
    try {
      const { kind, ok, since, until, limit } = req.query as Record<string, string | undefined>;
      const records = await store.query({
        kind: kind as RecordKind | undefined,
        ok: ok === undefined ? undefined : ok === 'true',
        since,
        until,
        limit: limit ? Number(limit) : undefined,
      });
      res.json({ count: records.length, records });
    } catch (err) {
      next(err);
    }
  });

  // --- pinned convention echo ---------------------------------------------------
  r.get('/convention', (_req, res) => {
    res.json({
      dh: 'standard',
      compositionOrder: ['RotZ(theta)', 'TransZ(d)', 'TransX(a)', 'RotX(alpha)'],
      angleUnit: 'rad',
      acceptedAngleUnits: ['rad', 'deg'],
      angleConversion: 'deg inputs are converted to rad exactly once at request entry',
      freeVariable: { revolute: 'theta (joint angle)', prismatic: 'd (link offset)' },
      euler: 'ZYX (yaw, pitch, roll), radians',
      toolTransform: 'optional fixed 4x4, right-multiplied; defaults to identity',
    });
  });

  // --- ready-to-call preset example ---------------------------------------------
  r.get('/example/planar3r', (_req, res) => {
    res.json(runDemo());
  });

  return r;
}
