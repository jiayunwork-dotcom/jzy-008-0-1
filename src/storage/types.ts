/**
 * 核算历史记录的存储抽象。
 * 每一次核算请求（含成功与失败、含批量中的每一组汇总）都写一条记录，
 * 供之后按类型/状态/armId/时间区间分页查询。
 */

export type HistoryRequestType = 'forward' | 'skeleton' | 'batch' | 'inverse';
export type HistoryStatus = 'ok' | 'error';

export interface HistoryRecord {
  requestType: HistoryRequestType;
  status: HistoryStatus;
  armId: string | null;
  request: unknown;
  response: unknown;
  linkCount?: number | null;
  caseCount?: number | null;
  errorCode?: string | null;
  durationMs: number;
}

export interface StoredHistoryRecord extends HistoryRecord {
  id: string;
  createdAt: string;
}

export interface HistoryFilter {
  type?: string;
  status?: string;
  armId?: string;
  sinceEpochMs?: number;
  untilEpochMs?: number;
  limit: number;
  offset: number;
}

export interface HistoryStore {
  init(): Promise<void>;
  insert(record: HistoryRecord): Promise<StoredHistoryRecord>;
  query(filter: HistoryFilter): Promise<StoredHistoryRecord[]>;
  count(filter?: Omit<HistoryFilter, 'limit' | 'offset'>): Promise<number>;
  close(): Promise<void>;
}

export interface CountRow {
  count: string;
}
