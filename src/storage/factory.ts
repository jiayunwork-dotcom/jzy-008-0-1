/**
 * 存储工厂：按环境变量选择实现。
 * - KIN_STORAGE=pg        → 连接外部 Postgres（Docker Compose 依赖）
 * - 缺省 / 其它            → PGlite 嵌入式落盘（KIN_DATA_DIR 指定目录）
 */
import { HistoryStore } from './types';
import { PGliteHistoryStore } from './pgliteStore';
import { PgHistoryStore } from './pgStore';

export async function createStore(): Promise<HistoryStore> {
  const mode = (process.env.KIN_STORAGE || 'pglite').toLowerCase();
  if (mode === 'pg') {
    const store = new PgHistoryStore(process.env.DATABASE_URL);
    await store.init();
    return store;
  }
  const dataDir = process.env.KIN_DATA_DIR || './data/pglite';
  const store = new PGliteHistoryStore(dataDir);
  await store.init();
  return store;
}
