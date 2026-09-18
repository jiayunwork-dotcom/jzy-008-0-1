/**
 * 4x4 homogeneous transform matrix helpers.
 * Matrices are row-major number[4][4]. All functions are pure:
 * they never mutate their inputs, so no state can leak between requests.
 */

export type Mat4 = number[][];

export function identity(): Mat4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

/** out = a * b (standard matrix product, fresh allocation). */
export function multiply(a: Mat4, b: Mat4): Mat4 {
  const out: Mat4 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[i][k] * b[k][j];
      out[i][j] = s;
    }
  }
  return out;
}

/** Left-to-right chain product: result = ms[0] * ms[1] * ... * ms[n-1]. */
export function chain(ms: Mat4[]): Mat4 {
  let acc = identity();
  for (const m of ms) acc = multiply(acc, m);
  return acc;
}

export function translationOf(m: Mat4): [number, number, number] {
  return [m[0][3], m[1][3], m[2][3]];
}

export function rotationOf(m: Mat4): number[][] {
  return [
    [m[0][0], m[0][1], m[0][2]],
    [m[1][0], m[1][1], m[1][2]],
    [m[2][0], m[2][1], m[2][2]],
  ];
}

/** Max absolute element-wise difference between two matrices. */
export function maxAbsDiff(a: Mat4, b: Mat4): number {
  let d = 0;
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      const v = Math.abs(a[i][j] - b[i][j]);
      if (v > d) d = v;
    }
  }
  return d;
}

/**
 * Fixed Euler convention for human-readable orientation: ZYX intrinsic
 * (yaw about Z, then pitch about Y, then roll about X), i.e.
 * R = Rz(yaw) * Ry(pitch) * Rx(roll). Angles in radians.
 */
export function rotationToEulerZYX(r: number[][]): {
  convention: 'ZYX';
  yaw: number;
  pitch: number;
  roll: number;
} {
  const r31 = r[2][0];
  const sy = -r31;
  const cy = Math.sqrt(r[0][0] * r[0][0] + r[1][0] * r[1][0]);
  const GIMBAL_EPS = 1e-12;
  let yaw: number, pitch: number, roll: number;
  if (cy > GIMBAL_EPS) {
    pitch = Math.atan2(sy, cy);
    yaw = Math.atan2(r[1][0], r[0][0]);
    roll = Math.atan2(r[2][1], r[2][2]);
  } else {
    // Gimbal lock: pitch = ±90°, roll is unobservable — pin it to 0.
    pitch = Math.atan2(sy, cy);
    yaw = Math.atan2(-r[0][1], r[1][1]);
    roll = 0;
  }
  return { convention: 'ZYX', yaw, pitch, roll };
}
