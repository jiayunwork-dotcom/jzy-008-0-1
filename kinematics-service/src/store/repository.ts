export type RecordKind = 'fk' | 'skeleton' | 'ik' | 'fk_batch';

export interface HistoryRecord {
  id: string;
  kind: RecordKind;
  ok: boolean;
  createdAt: string; // ISO 8601
  request: unknown;
  response: unknown;
}

export interface HistoryFilter {
  kind?: RecordKind;
  ok?: boolean;
  since?: string;
  until?: string;
  limit?: number;
}

export interface HistoryRepository {
  readonly dialect: string;
  init(): Promise<void>;
  save(record: HistoryRecord): Promise<void>;
  query(filter: HistoryFilter): Promise<HistoryRecord[]>;
  count(): Promise<number>;
  close(): Promise<void>;
}
