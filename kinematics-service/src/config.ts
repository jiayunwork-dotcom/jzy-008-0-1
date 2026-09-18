export interface ServiceConfig {
  port: number;
  databaseUrl?: string;
  sqlitePath: string;
}

export function loadConfig(env: NodeJS.ProcessEnv = process.env): ServiceConfig {
  return {
    port: Number(env.PORT ?? 8080),
    databaseUrl: env.DATABASE_URL,
    sqlitePath: env.SQLITE_PATH ?? './data/kinematics.db',
  };
}
