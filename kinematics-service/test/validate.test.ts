import { describe, expect, it } from 'vitest';
import { validateFkRequest, armSignature } from '../src/validation/validate.js';
import { KinematicsError } from '../src/errors.js';

const goodBody = () => ({
  angleUnit: 'rad',
  links: [
    { jointType: 'revolute', a: 1.0, alpha: 0, d: 0 },
    { jointType: 'revolute', a: 0.7, alpha: 0, d: 0 },
  ],
  joints: [0.1, 0.2],
});

function expectCode(body: unknown, code: string) {
  try {
    validateFkRequest(body);
    expect.unreachable(`expected ${code}`);
  } catch (err) {
    expect(err).toBeInstanceOf(KinematicsError);
    expect((err as KinematicsError).code).toBe(code);
    return err as KinematicsError;
  }
}

describe('input validation', () => {
  it('accepts a valid request and converts degrees once at entry', () => {
    const chain = validateFkRequest({
      angleUnit: 'deg',
      links: [
        { jointType: 'revolute', a: 1, alpha: 90, d: 0 },
        { jointType: 'revolute', a: 1, alpha: 0, d: 0 },
      ],
      joints: [180, -90],
    });
    expect(chain.links[0].theta).toBeCloseTo(Math.PI, 12);
    expect(chain.links[0].alpha).toBeCloseTo(Math.PI / 2, 12);
    expect(chain.links[1].theta).toBeCloseTo(-Math.PI / 2, 12);
  });

  it('rejects fewer than 2 links', () => {
    const err = expectCode({ angleUnit: 'rad', links: [{ jointType: 'revolute', a: 1, alpha: 0 }], joints: [0] }, 'TOO_FEW_LINKS');
    expect(err.details.parameter).toBe('links');
  });

  it('rejects negative link length unless explicitly allowed, naming the link', () => {
    const body = goodBody();
    body.links[1].a = -0.5;
    const err = expectCode(body, 'NEGATIVE_LINK_LENGTH');
    expect(err.details.linkIndex).toBe(1);
    expect(err.details.parameter).toBe('a');
    // allowed when declared
    const ok = validateFkRequest({ ...body, allowNegativeLinkLength: true });
    expect(ok.links[1].a).toBe(-0.5);
  });

  it('rejects joint vector length mismatch', () => {
    const body = goodBody();
    body.joints = [0.1];
    expectCode(body, 'JOINT_LENGTH_MISMATCH');
  });

  it('rejects unknown joint type, naming the link', () => {
    const body = goodBody();
    (body.links[0] as Record<string, unknown>).jointType = 'screw';
    const err = expectCode(body, 'JOINT_TYPE_MISMATCH');
    expect(err.details.linkIndex).toBe(0);
  });

  it('rejects non-finite values in coordinates and angles', () => {
    const body = goodBody();
    body.joints = [NaN, 0.2];
    expectCode(body, 'INVALID_TYPE');
    const body2 = goodBody();
    body2.joints = [Infinity, 0.2];
    const err = expectCode(body2, 'NON_FINITE');
    expect(err.details.linkIndex).toBe(0);
  });

  it('rejects missing fields with a readable message', () => {
    expectCode({ links: goodBody().links, joints: [0, 0] }, 'MISSING_FIELD');
    const body = goodBody();
    delete (body.links[0] as Record<string, unknown>).a;
    const err = expectCode(body, 'MISSING_FIELD');
    expect(err.details.parameter).toBe('a');
  });

  it('rejects non-numeric values', () => {
    const body = goodBody();
    (body.links[0] as Record<string, unknown>).a = 'long';
    expectCode(body, 'INVALID_TYPE');
  });

  it('rejects angleUnit/values that contradict each other', () => {
    const body = goodBody();
    body.joints = [180, 0.2]; // 180 "rad" — almost certainly degrees
    const err = expectCode(body, 'ANGLE_UNIT_MISMATCH');
    expect(err.message).toMatch(/degrees/i);
  });

  it('rejects invalid angleUnit values', () => {
    const body = { ...goodBody(), angleUnit: 'grad' };
    expectCode(body, 'ANGLE_UNIT_INVALID');
  });

  it('rejects a reordered/changed DH table pinned by armSignature', () => {
    const body = goodBody();
    const sig = armSignature(body.links as never);
    // same arm, same order → accepted
    expect(validateFkRequest({ ...body, armSignature: sig }).signature).toBe(sig);
    // reversed link order claiming to be the same arm → rejected
    const reversed = { ...body, links: [...body.links].reverse(), armSignature: sig };
    const err = expectCode(reversed, 'ARM_SIGNATURE_MISMATCH');
    expect(err.message).toMatch(/order/i);
  });

  it('resolves prismatic free variable into d, keeping theta fixed', () => {
    const chain = validateFkRequest({
      angleUnit: 'rad',
      links: [
        { jointType: 'revolute', a: 0.5, alpha: 0, d: 0 },
        { jointType: 'prismatic', a: 0.3, alpha: 0, theta: 0.25 },
      ],
      joints: [0.1, 0.9],
    });
    expect(chain.links[1].d).toBe(0.9);
    expect(chain.links[1].theta).toBe(0.25);
  });

  it('rejects malformed tool transforms', () => {
    const body = { ...goodBody(), toolTransform: [[1, 0], [0, 1]] };
    expectCode(body, 'INVALID_TOOL_TRANSFORM');
  });
});
