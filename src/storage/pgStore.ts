/**
 * 基于 node-postgres (pg.Pool) 的历史存储，供 Docker Compose 中的真实 Postgres 使用。
 */
import { Pool } from 'pg';
import {
  HistoryFilter,
  HistoryRecord,
  HistoryStore,
  StoredHistoryRecord,
} from './types';
import { buildWhere, insertParams, INSERT_SQL, mapRow, SCHEMA_SQL } from './schema';

export class PgHistoryStore implements HistoryStore {
  private readonly pool: Pool;

  constructor(connectionString?: string) {
    this.pool = new Pool(
      connectionString
        ? { connectionString }
        : {
            host: process.env.PGHOST || 'localhost',
            port: process.env.PGPORT ? Number(process.env.PGPORT) : 5432,
            database: process.env.PGDATABASE || 'kinematics',
            user: process.env.PGUSER || 'kinematics',
            password: process.env.PGPASSWORD || 'kinematics',
          },
    );
    this.pool.on('error', (err) => {
      // 空闲连接错误不应让进程崩溃。
      console.error('[storage] pg pool idle client error:', err.message);
    });
  }

  async init(): Promise<void> {
    await this.pool.query(SCHEMA_SQL);
  }

  async insert(record: HistoryRecord): Promise<StoredHistoryRecord> {
    const res = await this.pool.query(INSERT_SQL, insertParams(record));
    const row = res.rows[0];
    const saved = mapRow({
      ...{
        request_type: record.requestType,
        status: record.status,
        arm_id: record.armId ?? null,
        request_json: record.request ?? null,
        response_json: record.response ?? null,
        link_count: record.linkCount ?? null,
        case_count: record.caseCount ?? null,
        error_code: record.errorCode ?? null,
        duration_ms: record.durationMs,
      },
      id: row.id,
      created_at: row.created_at,
    });
    return saved;
  }

  async query(filter: HistoryFilter): Promise<StoredHistoryRecord[]> {
    const { clause, params } = buildWhere(filter);
    const p = params.slice();
    p.push(filter.limit);
    p.push(filter.offset);
    const sql =
      `SELECT * FROM kinematics_history ${clause} ORDER BY id DESC LIMIT $${p.length - 1} OFFSET $${p.length}`;
    const res = await this.pool.query(sql, p);
    return res.rows.map((r) => mapRow(r));
  }

  async count(filter?: Omit<HistoryFilter, 'limit' | 'offset'>): Promise<number> {
    const { clause, params } = buildWhere(filter ?? {});
    const res = await this.pool.query(`SELECT COUNT(*)::text AS count FROM kinematics_history ${clause}`, params);
    return Number(res.rows[0].count);
  }

  async close(): Promise<void> {
    await this.pool.end();
  }
}
