import { tmpdir } from 'os';
import { join } from 'path';
import { mkdtempSync } from 'fs';
import { ArmRegistry } from '../src/validation/armRegistry';
import { HistoryStore } from '../src/storage/types';
import { MemoryHistoryStore } from '../src/storage/memoryStore';
import { KinematicsService } from '../src/services/kinematicsService';
import { RawForwardRequest } from '../src/validation/types';

export function tmpDataDir(): string {
  return mkdtempSync(join(tmpdir(), 'kin-test-'));
}

export interface TestEnv {
  store: HistoryStore;
  service: KinematicsService;
  registry: ArmRegistry;
}

export async function makeService(): Promise<TestEnv> {
  const store = new MemoryHistoryStore();
  await store.init();
  const registry = new ArmRegistry();
  let seq = 0;
  const service = new KinematicsService({ store, registry, nextRequestId: () => ++seq });
  return { store, service, registry };
}

/** 平面 n 转动关节臂的正演请求。 */
export function planarChain(lengths: number[], joints: number[], angleUnit: 'radian' | 'degree' = 'radian'): RawForwardRequest {
  return {
    links: lengths.map((a) => ({ jointType: 'revolute' as const, a, alpha: 0, d: 0 })),
    joints: joints.slice(),
    angleUnit,
  };
}

export const approx = (a: number, b: number, tol = 1e-9): boolean => Math.abs(a - b) <= tol;

export function vecApprox(a: number[], b: number[], tol = 1e-9): boolean {
  return a.length === b.length && a.every((v, i) => approx(v, b[i], tol));
}

export function matApprox(A: number[][], B: number[][], tol = 1e-9): boolean {
  return A.length === B.length && A.every((row, i) => row.every((v, j) => approx(v, B[i][j], tol)));
}
