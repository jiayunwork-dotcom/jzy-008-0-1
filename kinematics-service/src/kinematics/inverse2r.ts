import { KinematicsError } from '../errors.js';
import { forwardKinematics } from './forward.js';

/** Pinned tolerance for IK reachability / singular / FK round-trip checks. */
export const IK_TOLERANCE = 1e-9;

export interface IkBranch {
  branch: 'elbow-up' | 'elbow-down';
  /** [theta1, theta2] in radians. */
  joints: [number, number];
  /** End-effector planar position obtained by re-running FK on this branch. */
  fkPosition: [number, number];
  /** |fkPosition - target| — must be within IK_TOLERANCE. */
  fkError: number;
}

export interface IkResult {
  solutions: IkBranch[];
}

function planar2rFk(theta1: number, theta2: number, a1: number, a2: number): [number, number] {
  return [
    a1 * Math.cos(theta1) + a2 * Math.cos(theta1 + theta2),
    a1 * Math.sin(theta1) + a2 * Math.sin(theta1 + theta2),
  ];
}

/**
 * Inverse kinematics for a planar 2R arm (standard DH, both twists zero).
 * Returns elbow-up and elbow-down branches, each verified by forward
 * kinematics against the pinned tolerance.
 *
 * Throws KinematicsError with code:
 *  - UNREACHABLE : target outside the annulus [|a1-a2|, a1+a2]
 *  - SINGULAR    : links collinear (cos(theta2) = ±1), branches indistinguishable
 */
export function inversePlanar2R(x: number, y: number, a1: number, a2: number): IkResult {
  for (const [name, v] of [['x', x], ['y', y], ['a1', a1], ['a2', a2]] as const) {
    if (typeof v !== 'number' || !Number.isFinite(v)) {
      throw new KinematicsError('NON_FINITE', `IK parameter "${name}" must be a finite number, got ${v}`, { parameter: name });
    }
  }
  if (a1 <= 0 || a2 <= 0) {
    throw new KinematicsError('NEGATIVE_LINK_LENGTH', 'IK link lengths a1 and a2 must be positive', {
      parameter: a1 <= 0 ? 'a1' : 'a2',
    });
  }

  const r2 = x * x + y * y;
  const cosElbow = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);

  if (cosElbow > 1 + IK_TOLERANCE || cosElbow < -1 - IK_TOLERANCE) {
    const r = Math.sqrt(r2);
    throw new KinematicsError(
      'UNREACHABLE',
      `Target (${x}, ${y}) at radius ${r} is outside the reachable annulus [${Math.abs(a1 - a2)}, ${a1 + a2}]`,
      { radius: r, minReach: Math.abs(a1 - a2), maxReach: a1 + a2 },
    );
  }
  if (Math.abs(Math.abs(cosElbow) - 1) <= IK_TOLERANCE) {
    throw new KinematicsError(
      'SINGULAR',
      'Links are collinear at this target (fully stretched or folded); elbow-up/elbow-down branches are indistinguishable',
      { cosElbow },
    );
  }

  const elbow = Math.acos(Math.max(-1, Math.min(1, cosElbow))); // in (0, pi)
  const baseAngle = Math.atan2(y, x);
  const shoulderOffset = Math.atan2(a2 * Math.sin(elbow), a1 + a2 * Math.cos(elbow));

  const branches: Array<{ branch: 'elbow-up' | 'elbow-down'; joints: [number, number] }> = [
    { branch: 'elbow-up', joints: [baseAngle - shoulderOffset, elbow] },
    { branch: 'elbow-down', joints: [baseAngle + shoulderOffset, -elbow] },
  ];

  const solutions: IkBranch[] = branches.map(({ branch, joints }) => {
    const [fx, fy] = planar2rFk(joints[0], joints[1], a1, a2);
    const fkError = Math.hypot(fx - x, fy - y);
    if (fkError > IK_TOLERANCE) {
      // Defensive: never hand back an angle set that does not reproduce the target.
      throw new KinematicsError('INTERNAL', `IK branch "${branch}" failed FK verification (error ${fkError})`, { fkError });
    }
    return { branch, joints, fkPosition: [fx, fy], fkError };
  });

  return { solutions };
}

/** Convenience: FK position of a planar 2R arm, exposed for tests/verification. */
export function planar2rForward(theta1: number, theta2: number, a1: number, a2: number): [number, number] {
  const res = forwardKinematics([
    { theta: theta1, d: 0, a: a1, alpha: 0 },
    { theta: theta2, d: 0, a: a2, alpha: 0 },
  ]);
  return [res.position[0], res.position[1]];
}
