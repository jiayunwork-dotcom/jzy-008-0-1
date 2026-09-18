/**
 * 服务启动入口。
 * 默认 PGlite 嵌入式落盘（可独立部署）；KIN_STORAGE=pg 时连接外部 Postgres。
 */
import { createStore } from './storage/factory';
import { createApp } from './server/app';

async function main(): Promise<void> {
  const port = Number(process.env.PORT || 3000);
  const host = process.env.HOST || '0.0.0.0';

  const store = await createStore();
  const { app } = createApp(store);

  const server = app.listen(port, host, () => {
    console.log(`[dh-kinematics-service] listening on http://${host}:${port}`);
    console.log(`[dh-kinematics-service] storage = ${process.env.KIN_STORAGE || 'pglite'} (${process.env.KIN_DATA_DIR || './data/pglite'})`);
  });

  const shutdown = async (signal: string): Promise<void> => {
    console.log(`[dh-kinematics-service] received ${signal}, shutting down...`);
    server.close(() => {
      void store.close().then(() => process.exit(0));
    });
    // 防止关闭卡死。
    setTimeout(() => process.exit(1), 5000).unref();
  };
  process.on('SIGTERM', () => void shutdown('SIGTERM'));
  process.on('SIGINT', () => void shutdown('SIGINT'));
}

main().catch((err) => {
  console.error('[dh-kinematics-service] failed to start:', err);
  process.exit(1);
});
