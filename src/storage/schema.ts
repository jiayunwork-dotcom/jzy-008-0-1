/**
 * 建表语句与行映射，两个 Postgres 实现（pg / PGlite）共用。
 */
import { HistoryRecord, StoredHistoryRecord } from './types';

export const SCHEMA_SQL = `
CREATE TABLE IF NOT EXISTS kinematics_history (
  id BIGSERIAL PRIMARY KEY,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
  request_type TEXT NOT NULL,
  status TEXT NOT NULL,
  arm_id TEXT,
  request_json JSONB NOT NULL,
  response_json JSONB NOT NULL,
  link_count INTEGER,
  case_count INTEGER,
  error_code TEXT,
  duration_ms DOUBLE PRECISION NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_kin_history_type ON kinematics_history(request_type);
CREATE INDEX IF NOT EXISTS idx_kin_history_status ON kinematics_history(status);
CREATE INDEX IF NOT EXISTS idx_kin_history_arm ON kinematics_history(arm_id);
CREATE INDEX IF NOT EXISTS idx_kin_history_created ON kinematics_history(created_at);
`;

export const INSERT_SQL = `
INSERT INTO kinematics_history
  (request_type, status, arm_id, request_json, response_json, link_count, case_count, error_code, duration_ms)
VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
RETURNING id, created_at
`;

export function insertParams(r: HistoryRecord): unknown[] {
  return [
    r.requestType,
    r.status,
    r.armId,
    JSON.stringify(r.request ?? null),
    JSON.stringify(r.response ?? null),
    r.linkCount ?? null,
    r.caseCount ?? null,
    r.errorCode ?? null,
    r.durationMs,
  ];
}

interface RawRow {
  id: number | string;
  created_at: Date | string;
  request_type: string;
  status: string;
  arm_id: string | null;
  request_json: unknown;
  response_json: unknown;
  link_count: number | null;
  case_count: number | null;
  error_code: string | null;
  duration_ms: number;
}

function parseJson(v: unknown): unknown {
  return typeof v === 'string' ? JSON.parse(v) : v;
}

export function mapRow(row: RawRow): StoredHistoryRecord {
  return {
    id: String(row.id),
    createdAt: row.created_at instanceof Date ? row.created_at.toISOString() : String(row.created_at),
    requestType: row.request_type as StoredHistoryRecord['requestType'],
    status: row.status as StoredHistoryRecord['status'],
    armId: row.arm_id,
    request: parseJson(row.request_json),
    response: parseJson(row.response_json),
    linkCount: row.link_count,
    caseCount: row.case_count,
    errorCode: row.error_code,
    durationMs: row.duration_ms,
  };
}

/**
 * 拼查询条件。时间戳入参为 epoch 毫秒，转为带时区的 timestamptz 比较。
 */
export function buildWhere(filter: {
  type?: string;
  status?: string;
  armId?: string;
  sinceEpochMs?: number;
  untilEpochMs?: number;
}): { clause: string; params: unknown[] } {
  const conds: string[] = [];
  const params: unknown[] = [];
  if (filter.type) {
    params.push(filter.type);
    conds.push(`request_type = $${params.length}`);
  }
  if (filter.status) {
    params.push(filter.status);
    conds.push(`status = $${params.length}`);
  }
  if (filter.armId) {
    params.push(filter.armId);
    conds.push(`arm_id = $${params.length}`);
  }
  if (filter.sinceEpochMs !== undefined) {
    params.push(new Date(filter.sinceEpochMs).toISOString());
    conds.push(`created_at >= $${params.length}::timestamptz`);
  }
  if (filter.untilEpochMs !== undefined) {
    params.push(new Date(filter.untilEpochMs).toISOString());
    conds.push(`created_at <= $${params.length}::timestamptz`);
  }
  return { clause: conds.length ? `WHERE ${conds.join(' AND ')}` : '', params };
}
