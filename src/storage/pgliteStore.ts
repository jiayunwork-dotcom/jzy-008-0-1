/**
 * 基于 PGlite 的历史存储：嵌入式 Postgres（WASM），数据落盘到本地目录，
 * 无需外部数据库进程即可独立部署，也是自动化测试使用的实现。
 * 查询接口与 node-postgres 对齐（都返回 { rows }）。
 */
import { PGlite } from '@electric-sql/pglite';
import {
  HistoryFilter,
  HistoryRecord,
  HistoryStore,
  StoredHistoryRecord,
} from './types';
import { buildWhere, insertParams, INSERT_SQL, mapRow, SCHEMA_SQL } from './schema';

export class PGliteHistoryStore implements HistoryStore {
  private db: PGlite;

  constructor(dataDir: string) {
    this.db = new PGlite(dataDir);
  }

  async init(): Promise<void> {
    await this.db.exec(SCHEMA_SQL);
  }

  async insert(record: HistoryRecord): Promise<StoredHistoryRecord> {
    const res = await this.db.query<{ id: number; created_at: Date }>(INSERT_SQL, insertParams(record));
    const row = res.rows[0];
    return mapRow({
      id: row.id,
      created_at: row.created_at,
      request_type: record.requestType,
      status: record.status,
      arm_id: record.armId ?? null,
      request_json: record.request ?? null,
      response_json: record.response ?? null,
      link_count: record.linkCount ?? null,
      case_count: record.caseCount ?? null,
      error_code: record.errorCode ?? null,
      duration_ms: record.durationMs,
    });
  }

  async query(filter: HistoryFilter): Promise<StoredHistoryRecord[]> {
    const { clause, params } = buildWhere(filter);
    const p = params.slice();
    p.push(filter.limit);
    p.push(filter.offset);
    const sql =
      `SELECT * FROM kinematics_history ${clause} ORDER BY id DESC LIMIT $${p.length - 1} OFFSET $${p.length}`;
    const res = await this.db.query(sql, p);
    return res.rows.map((r) => mapRow(r as never));
  }

  async count(filter?: Omit<HistoryFilter, 'limit' | 'offset'>): Promise<number> {
    const { clause, params } = buildWhere(filter ?? {});
    const res = await this.db.query<{ count: string }>(
      `SELECT COUNT(*)::text AS count FROM kinematics_history ${clause}`,
      params,
    );
    return Number(res.rows[0].count);
  }

  async close(): Promise<void> {
    await this.db.close();
  }
}
