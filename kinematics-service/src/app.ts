import express, { type Express } from 'express';
import { KinematicsError } from './errors.js';
import { kinematicsRouter } from './routes/kinematics.js';
import type { HistoryRepository } from './store/repository.js';

export function createApp(store: HistoryRepository): Express {
  const app = express();
  app.use(express.json({ limit: '1mb' }));

  const startedAt = Date.now();

  app.get('/health', async (_req, res) => {
    let db = 'up';
    try {
      await store.count();
    } catch {
      db = 'down';
    }
    res.status(db === 'up' ? 200 : 503).json({
      status: db === 'up' ? 'ok' : 'degraded',
      db,
      dialect: store.dialect,
      uptimeSeconds: Math.round((Date.now() - startedAt) / 1000),
      timestamp: new Date().toISOString(),
    });
  });

  app.use('/api', kinematicsRouter(store));

  // JSON parse errors and anything else lands here as a readable error body.
  app.use((err: unknown, _req: express.Request, res: express.Response, _next: express.NextFunction) => {
    if (err instanceof KinematicsError) {
      const status = err.code === 'UNREACHABLE' || err.code === 'SINGULAR' ? 422 : 400;
      res.status(status).json({ error: err.toJSON() });
      return;
    }
    if (err instanceof SyntaxError && 'body' in (err as object)) {
      res.status(400).json({ error: { code: 'INVALID_JSON', message: 'Request body is not valid JSON', details: {} } });
      return;
    }
    res.status(500).json({ error: { code: 'INTERNAL', message: err instanceof Error ? err.message : String(err), details: {} } });
  });

  return app;
}
