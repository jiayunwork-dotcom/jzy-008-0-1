import { mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import { createApp } from './app.js';
import { loadConfig } from './config.js';
import type { HistoryRepository } from './store/repository.js';
import { PgHistoryRepository } from './store/pg.js';
import { SqliteHistoryRepository } from './store/sqlite.js';

export async function createStore(config = loadConfig()): Promise<HistoryRepository> {
  const store: HistoryRepository = config.databaseUrl
    ? new PgHistoryRepository(config.databaseUrl)
    : new SqliteHistoryRepository(config.sqlitePath);
  if (!config.databaseUrl && config.sqlitePath !== ':memory:') {
    mkdirSync(dirname(config.sqlitePath), { recursive: true });
  }
  await store.init();
  return store;
}

async function main(): Promise<void> {
  const config = loadConfig();
  const store = await createStore(config);
  const app = createApp(store);
  const server = app.listen(config.port, () => {
    console.log(`kinematics-service listening on :${config.port} (db: ${store.dialect})`);
  });
  const shutdown = async () => {
    server.close();
    await store.close();
    process.exit(0);
  };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}

if (import.meta.url === `file://${process.argv[1]}`) {
  main().catch((err) => {
    console.error('fatal:', err);
    process.exit(1);
  });
}
