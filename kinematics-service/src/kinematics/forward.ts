import { chain, identity, multiply, rotationOf, rotationToEulerZYX, translationOf, type Mat4 } from '../math/mat4.js';
import { dhTransform, type DhParams } from './dh.js';

export interface ForwardResult {
  /** Base-to-end-effector homogeneous transform (tool transform included). */
  transform: Mat4;
  /** Translation part of `transform`. */
  position: [number, number, number];
  /** Rotation part of `transform`. */
  rotation: number[][];
  /** Fixed ZYX Euler reading of `rotation`, radians. */
  euler: { convention: 'ZYX'; yaw: number; pitch: number; roll: number };
  /**
   * Origin of each joint frame in the base frame, in chain order.
   * jointOrigins[i] is the origin of frame i+1 (after link i).
   */
  jointOrigins: [number, number, number][];
  /** Base origin followed by jointOrigins — connect in order for the skeleton polyline. */
  skeleton: [number, number, number][];
  /** Per-link transforms A_i (useful for auditing the chain product). */
  linkTransforms: Mat4[];
}

/**
 * Forward kinematics for an open chain. Pure function: every matrix is
 * freshly allocated per call, so consecutive calls can never contaminate
 * each other through leftover intermediate state.
 *
 * @param links resolved DH parameters per link (angles already in radians)
 * @param tool  fixed tool transform right-multiplied onto the chain product
 */
export function forwardKinematics(links: DhParams[], tool?: Mat4): ForwardResult {
  const linkTransforms = links.map(dhTransform);

  const jointOrigins: [number, number, number][] = [];
  let prefix = identity();
  for (const a of linkTransforms) {
    prefix = multiply(prefix, a);
    jointOrigins.push(translationOf(prefix));
  }

  let transform = prefix;
  if (tool) transform = multiply(transform, tool);

  const rotation = rotationOf(transform);
  return {
    transform,
    position: translationOf(transform),
    rotation,
    euler: rotationToEulerZYX(rotation),
    jointOrigins,
    skeleton: [[0, 0, 0], ...jointOrigins],
    linkTransforms,
  };
}

/** Recompute the chain product independently — used by the consistency guard. */
export function chainProduct(links: DhParams[]): Mat4 {
  return chain(links.map(dhTransform));
}
