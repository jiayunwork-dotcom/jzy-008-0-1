import Database from 'better-sqlite3';
import type { HistoryFilter, HistoryRecord, HistoryRepository } from './repository.js';

/** SQLite-backed repository (default for local runs and tests). */
export class SqliteHistoryRepository implements HistoryRepository {
  readonly dialect = 'sqlite';
  private db: Database.Database;

  constructor(path: string) {
    this.db = new Database(path);
    this.db.pragma('journal_mode = WAL');
  }

  async init(): Promise<void> {
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS history (
        id TEXT PRIMARY KEY,
        kind TEXT NOT NULL,
        ok INTEGER NOT NULL,
        created_at TEXT NOT NULL,
        request TEXT NOT NULL,
        response TEXT NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_history_kind ON history(kind);
      CREATE INDEX IF NOT EXISTS idx_history_created ON history(created_at);
    `);
  }

  async save(record: HistoryRecord): Promise<void> {
    this.db
      .prepare('INSERT INTO history (id, kind, ok, created_at, request, response) VALUES (?, ?, ?, ?, ?, ?)')
      .run(record.id, record.kind, record.ok ? 1 : 0, record.createdAt, JSON.stringify(record.request), JSON.stringify(record.response));
  }

  async query(filter: HistoryFilter): Promise<HistoryRecord[]> {
    const where: string[] = [];
    const args: unknown[] = [];
    if (filter.kind) { where.push('kind = ?'); args.push(filter.kind); }
    if (filter.ok !== undefined) { where.push('ok = ?'); args.push(filter.ok ? 1 : 0); }
    if (filter.since) { where.push('created_at >= ?'); args.push(filter.since); }
    if (filter.until) { where.push('created_at <= ?'); args.push(filter.until); }
    const sql = `SELECT * FROM history ${where.length ? 'WHERE ' + where.join(' AND ') : ''} ORDER BY created_at DESC, id DESC LIMIT ?`;
    args.push(filter.limit ?? 100);
    const rows = this.db.prepare(sql).all(...args) as Array<{ id: string; kind: string; ok: number; created_at: string; request: string; response: string }>;
    return rows.map((r) => ({
      id: r.id,
      kind: r.kind as HistoryRecord['kind'],
      ok: r.ok === 1,
      createdAt: r.created_at,
      request: JSON.parse(r.request),
      response: JSON.parse(r.response),
    }));
  }

  async count(): Promise<number> {
    const row = this.db.prepare('SELECT COUNT(*) AS n FROM history').get() as { n: number };
    return row.n;
  }

  async close(): Promise<void> {
    this.db.close();
  }
}
