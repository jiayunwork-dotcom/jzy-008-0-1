import { createHash } from 'node:crypto';
import { KinematicsError } from '../errors.js';
import type { DhParams } from '../kinematics/dh.js';
import type { Mat4 } from '../math/mat4.js';

export type JointType = 'revolute' | 'prismatic';
export type AngleUnit = 'rad' | 'deg';

export interface LinkInput {
  jointType: JointType;
  /** Fixed joint angle (rad/deg per angleUnit). Free variable for revolute joints. */
  theta?: number;
  /** Fixed link offset. Free variable for prismatic joints. */
  d?: number;
  /** Link length. */
  a: number;
  /** Link twist (rad/deg per angleUnit). */
  alpha: number;
}

export interface FkRequest {
  links: LinkInput[];
  joints: number[];
  angleUnit: AngleUnit;
  toolTransform?: Mat4;
  allowNegativeLinkLength?: boolean;
  /** Optional: sha256 fingerprint the caller expects this arm's DH table to have. */
  armSignature?: string;
}

export interface ResolvedChain {
  links: DhParams[];
  tool?: Mat4;
  signature: string;
}

/** Heuristic guard: radians beyond this magnitude almost certainly were degrees. */
const RAD_PLAUSIBLE_MAX = 8 * Math.PI; // 4 full turns
const DEG_PLAUSIBLE_MAX = 4 * 360;

function fail(code: string, message: string, details: Record<string, unknown>): never {
  throw new KinematicsError(code, message, details);
}

function requireFiniteNumber(v: unknown, linkIndex: number | null, parameter: string): number {
  if (v === undefined || v === null) {
    fail('MISSING_FIELD', `Missing required parameter "${parameter}"${linkIndex !== null ? ` on link ${linkIndex}` : ''}`, { linkIndex, parameter });
  }
  if (typeof v !== 'number' || Number.isNaN(v)) {
    fail('INVALID_TYPE', `Parameter "${parameter}"${linkIndex !== null ? ` on link ${linkIndex}` : ''} must be a number, got ${JSON.stringify(v)}`, { linkIndex, parameter, value: v });
  }
  if (!Number.isFinite(v)) {
    fail('NON_FINITE', `Parameter "${parameter}"${linkIndex !== null ? ` on link ${linkIndex}` : ''} must be finite, got ${v}`, { linkIndex, parameter, value: v });
  }
  return v;
}

function checkAnglePlausible(value: number, unit: AngleUnit, linkIndex: number | null, parameter: string): void {
  if (unit === 'rad' && Math.abs(value) > RAD_PLAUSIBLE_MAX) {
    fail('ANGLE_UNIT_MISMATCH',
      `angleUnit is "rad" but "${parameter}"${linkIndex !== null ? ` on link ${linkIndex}` : ''} is ${value} — beyond ${RAD_PLAUSIBLE_MAX} rad, this looks like degrees. Fix the value or pass angleUnit "deg".`,
      { linkIndex, parameter, value, angleUnit: unit });
  }
  if (unit === 'deg' && Math.abs(value) > DEG_PLAUSIBLE_MAX) {
    fail('ANGLE_UNIT_MISMATCH',
      `angleUnit is "deg" but "${parameter}"${linkIndex !== null ? ` on link ${linkIndex}` : ''} is ${value} — beyond ±${DEG_PLAUSIBLE_MAX}°, this looks like radians mislabeled or a runaway value.`,
      { linkIndex, parameter, value, angleUnit: unit });
  }
}

/** Canonical fingerprint of a DH table so callers can pin "this is the same arm". */
export function armSignature(links: LinkInput[]): string {
  const canonical = links.map((l) => ({
    jointType: l.jointType,
    theta: l.theta ?? 0,
    d: l.d ?? 0,
    a: l.a,
    alpha: l.alpha,
  }));
  return createHash('sha256').update(JSON.stringify(canonical)).digest('hex');
}

function validateToolTransform(t: unknown): Mat4 {
  if (!Array.isArray(t) || t.length !== 4 || t.some((row) => !Array.isArray(row) || row.length !== 4)) {
    fail('INVALID_TOOL_TRANSFORM', 'toolTransform must be a 4x4 matrix', { parameter: 'toolTransform' });
  }
  const m = t as unknown[][];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      requireFiniteNumber(m[i][j], null, `toolTransform[${i}][${j}]`);
    }
  }
  const last = m[3] as number[];
  if (Math.abs(last[0]) > 1e-12 || Math.abs(last[1]) > 1e-12 || Math.abs(last[2]) > 1e-12 || Math.abs(last[3] - 1) > 1e-12) {
    fail('INVALID_TOOL_TRANSFORM', 'toolTransform last row must be [0, 0, 0, 1] (a homogeneous transform)', { parameter: 'toolTransform' });
  }
  return m as Mat4;
}

/**
 * Validate a raw FK/skeleton request body and resolve it into radian-domain
 * DH parameters. Angle conversion happens exactly once, here at the entry
 * point — everything downstream only ever sees radians.
 */
export function validateFkRequest(body: unknown): ResolvedChain {
  if (typeof body !== 'object' || body === null || Array.isArray(body)) {
    fail('INVALID_TYPE', 'Request body must be a JSON object', {});
  }
  const req = body as Record<string, unknown>;

  // --- angle unit: convert once, here, before any trig ---------------------
  const angleUnit = req.angleUnit;
  if (angleUnit === undefined) fail('MISSING_FIELD', 'Missing required field "angleUnit" ("rad" or "deg")', { parameter: 'angleUnit' });
  if (angleUnit !== 'rad' && angleUnit !== 'deg') {
    fail('ANGLE_UNIT_INVALID', `angleUnit must be "rad" or "deg", got ${JSON.stringify(angleUnit)}`, { parameter: 'angleUnit', value: angleUnit });
  }
  const toRad = angleUnit === 'deg' ? (v: number) => (v * Math.PI) / 180 : (v: number) => v;

  // --- links ---------------------------------------------------------------
  if (!Array.isArray(req.links)) fail('MISSING_FIELD', 'Missing required field "links" (array of link definitions)', { parameter: 'links' });
  const rawLinks = req.links as unknown[];
  if (rawLinks.length < 2) {
    fail('TOO_FEW_LINKS', `At least 2 links are required, got ${rawLinks.length}`, { parameter: 'links', count: rawLinks.length });
  }

  const links: LinkInput[] = rawLinks.map((raw, i) => {
    if (typeof raw !== 'object' || raw === null || Array.isArray(raw)) {
      fail('INVALID_TYPE', `Link ${i} must be an object`, { linkIndex: i });
    }
    const l = raw as Record<string, unknown>;
    const jointType = l.jointType;
    if (jointType !== 'revolute' && jointType !== 'prismatic') {
      fail('JOINT_TYPE_MISMATCH', `Link ${i}: jointType must be "revolute" or "prismatic", got ${JSON.stringify(jointType)}`, { linkIndex: i, parameter: 'jointType', value: jointType });
    }
    const a = requireFiniteNumber(l.a, i, 'a');
    const allowNeg = req.allowNegativeLinkLength === true;
    if (a < 0 && !allowNeg) {
      fail('NEGATIVE_LINK_LENGTH', `Link ${i}: link length a=${a} is negative. Pass allowNegativeLinkLength:true if this is intentional.`, { linkIndex: i, parameter: 'a', value: a });
    }
    const alpha = requireFiniteNumber(l.alpha, i, 'alpha');
    checkAnglePlausible(alpha, angleUnit as AngleUnit, i, 'alpha');
    const link: LinkInput = { jointType, a, alpha };
    if (l.theta !== undefined) {
      const theta = requireFiniteNumber(l.theta, i, 'theta');
      checkAnglePlausible(theta, angleUnit as AngleUnit, i, 'theta');
      link.theta = theta;
    }
    if (l.d !== undefined) link.d = requireFiniteNumber(l.d, i, 'd');
    return link;
  });

  // --- joint vector ----------------------------------------------------------
  if (!Array.isArray(req.joints)) fail('MISSING_FIELD', 'Missing required field "joints" (array of free-variable values)', { parameter: 'joints' });
  const joints = req.joints as unknown[];
  if (joints.length !== links.length) {
    fail('JOINT_LENGTH_MISMATCH', `joints has ${joints.length} entries but the arm has ${links.length} links`, { parameter: 'joints', expected: links.length, got: joints.length });
  }
  joints.forEach((j, i) => {
    requireFiniteNumber(j, i, `joints[${i}]`);
    if (links[i].jointType === 'revolute') checkAnglePlausible(j as number, angleUnit as AngleUnit, i, `joints[${i}]`);
  });

  // --- optional arm identity check -------------------------------------------
  const signature = armSignature(links);
  if (req.armSignature !== undefined) {
    if (typeof req.armSignature !== 'string') fail('INVALID_TYPE', 'armSignature must be a string', { parameter: 'armSignature' });
    if (req.armSignature !== signature) {
      fail('ARM_SIGNATURE_MISMATCH',
        'The supplied DH table does not match the armSignature the caller pinned — the link set or its order differs from the registered arm. Check for reordered, dropped or modified links.',
        { parameter: 'armSignature', expected: req.armSignature, actual: signature });
    }
  }

  // --- resolve free variables into a radian-domain DH table ------------------
  const resolved: DhParams[] = links.map((l, i) => {
    if (l.jointType === 'revolute') {
      return { theta: toRad(joints[i] as number), d: l.d ?? 0, a: l.a, alpha: toRad(l.alpha) };
    }
    // prismatic: free variable is d; theta is a fixed constant
    return { theta: toRad(l.theta ?? 0), d: joints[i] as number, a: l.a, alpha: toRad(l.alpha) };
  });

  const tool = req.toolTransform !== undefined ? validateToolTransform(req.toolTransform) : undefined;
  return { links: resolved, tool, signature };
}
