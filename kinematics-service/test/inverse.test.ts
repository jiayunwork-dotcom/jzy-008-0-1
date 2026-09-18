import { describe, expect, it } from 'vitest';
import { inversePlanar2R, planar2rForward, IK_TOLERANCE } from '../src/kinematics/inverse2r.js';
import { KinematicsError } from '../src/errors.js';

describe('planar 2R inverse kinematics', () => {
  it('returns elbow-up and elbow-down branches that FK back onto the target', () => {
    const a1 = 1.0, a2 = 0.7;
    const x = 1.1, y = 0.6;
    const { solutions } = inversePlanar2R(x, y, a1, a2);
    expect(solutions).toHaveLength(2);
    const branches = solutions.map((s) => s.branch).sort();
    expect(branches).toEqual(['elbow-down', 'elbow-up']);
    for (const s of solutions) {
      expect(s.fkError).toBeLessThanOrEqual(IK_TOLERANCE);
      const [fx, fy] = planar2rForward(s.joints[0], s.joints[1], a1, a2);
      expect(fx).toBeCloseTo(x, 9);
      expect(fy).toBeCloseTo(y, 9);
    }
    // the two branches really differ
    expect(Math.abs(solutions[0].joints[1] - solutions[1].joints[1])).toBeGreaterThan(1e-6);
  });

  it('round-trips a known joint configuration', () => {
    const a1 = 1.2, a2 = 0.9;
    const t1 = 0.7, t2 = -0.8;
    const [x, y] = planar2rForward(t1, t2, a1, a2);
    const { solutions } = inversePlanar2R(x, y, a1, a2);
    const match = solutions.some(
      (s) => Math.abs(s.joints[0] - t1) < 1e-9 && Math.abs(s.joints[1] - t2) < 1e-9,
    );
    expect(match).toBe(true);
  });

  it('reports UNREACHABLE beyond the outer radius', () => {
    try {
      inversePlanar2R(3.0, 0, 1.0, 0.7);
      expect.unreachable();
    } catch (err) {
      expect(err).toBeInstanceOf(KinematicsError);
      expect((err as KinematicsError).code).toBe('UNREACHABLE');
    }
  });

  it('reports UNREACHABLE inside the inner radius', () => {
    try {
      inversePlanar2R(0.1, 0, 1.0, 0.7);
      expect.unreachable();
    } catch (err) {
      expect((err as KinematicsError).code).toBe('UNREACHABLE');
    }
  });

  it('reports SINGULAR when fully stretched (collinear, outer boundary)', () => {
    try {
      inversePlanar2R(1.7, 0, 1.0, 0.7);
      expect.unreachable();
    } catch (err) {
      expect((err as KinematicsError).code).toBe('SINGULAR');
    }
  });

  it('reports SINGULAR when fully folded (collinear, inner boundary)', () => {
    try {
      inversePlanar2R(0.3, 0, 1.0, 0.7);
      expect.unreachable();
    } catch (err) {
      expect((err as KinematicsError).code).toBe('SINGULAR');
    }
  });

  it('rejects non-finite input', () => {
    try {
      inversePlanar2R(NaN, 0, 1, 1);
      expect.unreachable();
    } catch (err) {
      expect((err as KinematicsError).code).toBe('NON_FINITE');
    }
  });
});
