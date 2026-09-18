/** Structured, distinguishable error raised before any computation happens. */
export class KinematicsError extends Error {
  constructor(
    public readonly code: string,
    message: string,
    public readonly details: Record<string, unknown> = {},
  ) {
    super(message);
    this.name = 'KinematicsError';
  }

  toJSON() {
    return { code: this.code, message: this.message, details: this.details };
  }
}

export function errorBody(err: unknown) {
  if (err instanceof KinematicsError) {
    return { error: { code: err.code, message: err.message, details: err.details } };
  }
  const message = err instanceof Error ? err.message : String(err);
  return { error: { code: 'INTERNAL', message, details: {} } };
}
