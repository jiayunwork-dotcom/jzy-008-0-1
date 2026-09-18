/**
 * 内存版历史存储：与 PgHistoryStore / PGliteHistoryStore 实现同一接口、
 * 同一套过滤与排序语义，专供自动化测试使用（PGlite 的 WASM 动态导入
 * 无法在 jest 的 CJS VM 沙箱内执行）。也可通过 KIN_STORAGE=memory 临时使用。
 */
import {
  HistoryFilter,
  HistoryRecord,
  HistoryStore,
  StoredHistoryRecord,
} from './types';

interface Row extends HistoryRecord {
  id: number;
  createdAtMs: number;
}

export class MemoryHistoryStore implements HistoryStore {
  private rows: Row[] = [];
  private seq = 0;

  async init(): Promise<void> {
    /* 无外部资源 */
  }

  async insert(record: HistoryRecord): Promise<StoredHistoryRecord> {
    const row: Row = { ...record, id: ++this.seq, createdAtMs: Date.now() };
    this.rows.push(row);
    return this.map(row);
  }

  async query(filter: HistoryFilter): Promise<StoredHistoryRecord[]> {
    return this.rows
      .filter((r) => this.match(r, filter))
      .sort((a, b) => b.id - a.id)
      .slice(filter.offset, filter.offset + filter.limit)
      .map((r) => this.map(r));
  }

  async count(filter?: Omit<HistoryFilter, 'limit' | 'offset'>): Promise<number> {
    return this.rows.filter((r) => this.match(r, filter ?? {})).length;
  }

  async close(): Promise<void> {
    this.rows = [];
  }

  private match(r: Row, f: Omit<HistoryFilter, 'limit' | 'offset'>): boolean {
    if (f.type && r.requestType !== f.type) return false;
    if (f.status && r.status !== f.status) return false;
    if (f.armId && r.armId !== f.armId) return false;
    if (f.sinceEpochMs !== undefined && r.createdAtMs < f.sinceEpochMs) return false;
    if (f.untilEpochMs !== undefined && r.createdAtMs > f.untilEpochMs) return false;
    return true;
  }

  private map(r: Row): StoredHistoryRecord {
    return {
      id: String(r.id),
      createdAt: new Date(r.createdAtMs).toISOString(),
      requestType: r.requestType,
      status: r.status,
      armId: r.armId,
      request: r.request,
      response: r.response,
      linkCount: r.linkCount,
      caseCount: r.caseCount,
      errorCode: r.errorCode,
      durationMs: r.durationMs,
    };
  }
}
