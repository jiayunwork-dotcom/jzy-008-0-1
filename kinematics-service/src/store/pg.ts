import pg from 'pg';
import type { HistoryFilter, HistoryRecord, HistoryRepository } from './repository.js';

/** Postgres-backed repository (used when DATABASE_URL is set, e.g. docker-compose). */
export class PgHistoryRepository implements HistoryRepository {
  readonly dialect = 'postgres';
  private pool: pg.Pool;

  constructor(connectionString: string) {
    this.pool = new pg.Pool({ connectionString });
  }

  async init(): Promise<void> {
    await this.pool.query(`
      CREATE TABLE IF NOT EXISTS history (
        id TEXT PRIMARY KEY,
        kind TEXT NOT NULL,
        ok BOOLEAN NOT NULL,
        created_at TIMESTAMPTZ NOT NULL,
        request JSONB NOT NULL,
        response JSONB NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_history_kind ON history(kind);
      CREATE INDEX IF NOT EXISTS idx_history_created ON history(created_at);
    `);
  }

  async save(record: HistoryRecord): Promise<void> {
    await this.pool.query(
      'INSERT INTO history (id, kind, ok, created_at, request, response) VALUES ($1, $2, $3, $4, $5, $6)',
      [record.id, record.kind, record.ok, record.createdAt, JSON.stringify(record.request), JSON.stringify(record.response)],
    );
  }

  async query(filter: HistoryFilter): Promise<HistoryRecord[]> {
    const where: string[] = [];
    const args: unknown[] = [];
    if (filter.kind) { args.push(filter.kind); where.push(`kind = $${args.length}`); }
    if (filter.ok !== undefined) { args.push(filter.ok); where.push(`ok = $${args.length}`); }
    if (filter.since) { args.push(filter.since); where.push(`created_at >= $${args.length}`); }
    if (filter.until) { args.push(filter.until); where.push(`created_at <= $${args.length}`); }
    args.push(filter.limit ?? 100);
    const sql = `SELECT id, kind, ok, created_at AS "createdAt", request, response FROM history ${where.length ? 'WHERE ' + where.join(' AND ') : ''} ORDER BY created_at DESC, id DESC LIMIT $${args.length}`;
    const res = await this.pool.query(sql, args);
    return res.rows.map((r) => ({ ...r, createdAt: new Date(r.createdAt).toISOString() }));
  }

  async count(): Promise<number> {
    const res = await this.pool.query('SELECT COUNT(*)::int AS n FROM history');
    return res.rows[0].n as number;
  }

  async close(): Promise<void> {
    await this.pool.end();
  }
}
