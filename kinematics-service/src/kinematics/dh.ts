import type { Mat4 } from '../math/mat4.js';

/**
 * The single DH convention allowed in this service: STANDARD DH.
 * Per-link homogeneous transform, always composed in this fixed order:
 *
 *   A_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
 *
 *   - theta : joint angle  (free variable for revolute joints), radians
 *   - d     : link offset  (free variable for prismatic joints), length
 *   - a     : link length, length
 *   - alpha : link twist, radians
 */
export interface DhParams {
  theta: number;
  d: number;
  a: number;
  alpha: number;
}

export function dhTransform(p: DhParams): Mat4 {
  const ct = Math.cos(p.theta);
  const st = Math.sin(p.theta);
  const ca = Math.cos(p.alpha);
  const sa = Math.sin(p.alpha);
  return [
    [ct, -st * ca, st * sa, p.a * ct],
    [st, ct * ca, -ct * sa, p.a * st],
    [0, sa, ca, p.d],
    [0, 0, 0, 1],
  ];
}
