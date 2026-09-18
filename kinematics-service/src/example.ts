import { forwardKinematics, type ForwardResult } from './kinematics/forward.js';
import type { FkRequest } from './validation/validate.js';
import { validateFkRequest } from './validation/validate.js';

/** Preset demo: planar 3R arm, known link lengths, all-zero joint angles. */
export const DEMO_LINK_LENGTHS = [1.0, 0.7, 0.4] as const;

export function demoRequest(): FkRequest {
  return {
    angleUnit: 'rad',
    links: DEMO_LINK_LENGTHS.map((a) => ({ jointType: 'revolute', a, alpha: 0, d: 0 })),
    joints: [0, 0, 0],
  };
}

export interface DemoResult {
  description: string;
  linkLengths: readonly number[];
  joints: number[];
  fk: ForwardResult;
  /** Hand-computed straight-line expectation: (sum of lengths, 0, 0), identity rotation. */
  expected: { position: [number, number, number]; rotation: number[][] };
  matchesExpectation: boolean;
}

export function runDemo(): DemoResult {
  const chain = validateFkRequest(demoRequest());
  const fk = forwardKinematics(chain.links);
  const sum = DEMO_LINK_LENGTHS.reduce((s, v) => s + v, 0);
  const expected: DemoResult['expected'] = {
    position: [sum, 0, 0],
    rotation: [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ],
  };
  const posOk =
    Math.abs(fk.position[0] - sum) < 1e-12 &&
    Math.abs(fk.position[1]) < 1e-12 &&
    Math.abs(fk.position[2]) < 1e-12;
  const rotOk = fk.rotation.every((row, i) => row.every((v, j) => Math.abs(v - (i === j ? 1 : 0)) < 1e-12));
  return {
    description: 'Planar 3R arm, all-zero joint angles — end effector must sit at the straight-line position (sum of link lengths, 0, 0) with identity rotation.',
    linkLengths: DEMO_LINK_LENGTHS,
    joints: [0, 0, 0],
    fk,
    expected,
    matchesExpectation: posOk && rotOk,
  };
}
