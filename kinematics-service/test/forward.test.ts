import { describe, expect, it } from 'vitest';
import { forwardKinematics, chainProduct } from '../src/kinematics/forward.js';
import { maxAbsDiff, translationOf, rotationOf, type Mat4 } from '../src/math/mat4.js';
import type { DhParams } from '../src/kinematics/dh.js';

const TOL = 1e-12;

function planar3r(a: [number, number, number], joints: [number, number, number]): DhParams[] {
  return a.map((len, i) => ({ theta: joints[i], d: 0, a: len, alpha: 0 }));
}

function expectMatClose(a: Mat4, b: Mat4, tol = TOL) {
  expect(maxAbsDiff(a, b)).toBeLessThan(tol);
}

describe('forward kinematics invariants', () => {
  it('planar 3R with all-zero joints sits at the straight-line pose', () => {
    const a: [number, number, number] = [1.0, 0.7, 0.4];
    const fk = forwardKinematics(planar3r(a, [0, 0, 0]));
    expect(fk.position[0]).toBeCloseTo(2.1, 12);
    expect(fk.position[1]).toBeCloseTo(0, 12);
    expect(fk.position[2]).toBeCloseTo(0, 12);
    expect(fk.rotation).toEqual([
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ]);
    expect(fk.euler.yaw).toBeCloseTo(0, 12);
    expect(fk.euler.pitch).toBeCloseTo(0, 12);
    expect(fk.euler.roll).toBeCloseTo(0, 12);
  });

  it('total transform equals the ordered product of per-link transforms', () => {
    const links: DhParams[] = [
      { theta: 0.3, d: 0.1, a: 0.5, alpha: 0.2 },
      { theta: -1.1, d: 0.0, a: 0.4, alpha: Math.PI / 2 },
      { theta: 2.2, d: 0.05, a: 0.3, alpha: -0.4 },
    ];
    const fk = forwardKinematics(links);
    expectMatClose(fk.transform, chainProduct(links));
  });

  it('adding 2π to a revolute joint leaves the total transform unchanged', () => {
    const joints: [number, number, number] = [0.4, -0.9, 1.7];
    const a: [number, number, number] = [1.0, 0.7, 0.4];
    const base = forwardKinematics(planar3r(a, joints));
    const wrapped = forwardKinematics(planar3r(a, [joints[0], joints[1] + 2 * Math.PI, joints[2]]));
    expectMatClose(base.transform, wrapped.transform);
  });

  it('doubling all link lengths doubles position, keeps rotation', () => {
    const joints: [number, number, number] = [0.5, 0.6, -0.3];
    const base = forwardKinematics(planar3r([1.0, 0.7, 0.4], joints));
    const doubled = forwardKinematics(planar3r([2.0, 1.4, 0.8], joints));
    expect(doubled.position[0]).toBeCloseTo(2 * base.position[0], 10);
    expect(doubled.position[1]).toBeCloseTo(2 * base.position[1], 10);
    expect(doubled.position[2]).toBeCloseTo(2 * base.position[2], 10);
    expectMatClose(
      [...doubled.rotation.map((r) => [...r, 0]), [0, 0, 0, 1]] as Mat4,
      [...base.rotation.map((r) => [...r, 0]), [0, 0, 0, 1]] as Mat4,
    );
  });

  it('moving only the last joint of a planar 3R rotates about a fixed wrist point', () => {
    const a: [number, number, number] = [1.0, 0.7, 0.4];
    const before = forwardKinematics(planar3r(a, [0.3, 0.8, 0.1]));
    const after = forwardKinematics(planar3r(a, [0.3, 0.8, 1.9]));
    // wrist = origin of the last joint frame = jointOrigins[1] (after link 2)
    const wristBefore = before.jointOrigins[1];
    const wristAfter = after.jointOrigins[1];
    for (let k = 0; k < 3; k++) expect(wristAfter[k]).toBeCloseTo(wristBefore[k], 12);
    // end effector moved on a circle of radius a3 around the wrist
    const rBefore = Math.hypot(before.position[0] - wristBefore[0], before.position[1] - wristBefore[1]);
    const rAfter = Math.hypot(after.position[0] - wristAfter[0], after.position[1] - wristAfter[1]);
    expect(rBefore).toBeCloseTo(0.4, 12);
    expect(rAfter).toBeCloseTo(0.4, 12);
  });

  it('prismatic joint: changing only d translates the end effector along that joint axis by the delta', () => {
    // joint 2 prismatic with fixed theta; its z axis in the base frame is Rz(theta1) applied to z
    const theta1 = 0.6;
    const mk = (d2: number): DhParams[] => [
      { theta: theta1, d: 0, a: 0.5, alpha: 0 },
      { theta: 0.3, d: d2, a: 0.4, alpha: 0 }, // prismatic free variable = d
    ];
    const p0 = translationOf(forwardKinematics(mk(0.2)).transform);
    const p1 = translationOf(forwardKinematics(mk(0.9)).transform);
    const delta = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    // joint 2's z axis in base frame: Rz(theta1) * [0,0,1] = [0,0,1] (planar) — use a twisted link to make it 3D
    expect(Math.hypot(...delta)).toBeCloseTo(0.7, 12);
    // direction must equal the joint axis (here: base z, since alpha1 = 0)
    expect(delta[0]).toBeCloseTo(0, 12);
    expect(delta[1]).toBeCloseTo(0, 12);
    expect(delta[2]).toBeCloseTo(0.7, 12);
  });

  it('prismatic joint axis after a twist still matches the displacement direction', () => {
    const mk = (d2: number): DhParams[] => [
      { theta: 0.4, d: 0.1, a: 0.5, alpha: Math.PI / 2 }, // twist makes joint-2 z axis non-vertical
      { theta: 0, d: d2, a: 0.3, alpha: 0 },
    ];
    const p0 = translationOf(forwardKinematics(mk(0.1)).transform);
    const p1 = translationOf(forwardKinematics(mk(0.6)).transform);
    const delta = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    expect(Math.hypot(...delta)).toBeCloseTo(0.5, 12);
    // joint-2 z axis in base frame = Rz(0.4)Rx(π/2) ẑ = Rz(0.4)·[0,-1,0] = [sin0.4, -cos0.4, 0]
    expect(delta[0]).toBeCloseTo(0.5 * Math.sin(0.4), 12);
    expect(delta[1]).toBeCloseTo(-0.5 * Math.cos(0.4), 12);
    expect(delta[2]).toBeCloseTo(0, 12);
  });

  it('skeleton polyline starts at the base and ends at the last joint origin', () => {
    const fk = forwardKinematics(planar3r([1, 0.7, 0.4], [0.2, 0.3, 0.4]));
    expect(fk.skeleton[0]).toEqual([0, 0, 0]);
    expect(fk.skeleton.length).toBe(4);
    expect(fk.skeleton[3]).toEqual(fk.jointOrigins[2]);
    expect(fk.jointOrigins[0][0]).toBeCloseTo(Math.cos(0.2) * 1, 12);
  });

  it('consecutive calls with different tables do not contaminate each other', () => {
    const fk1 = forwardKinematics(planar3r([1, 0.7, 0.4], [0.1, 0.2, 0.3]));
    const fk2 = forwardKinematics(planar3r([2, 2, 2], [1, 1, 1]));
    const fk1again = forwardKinematics(planar3r([1, 0.7, 0.4], [0.1, 0.2, 0.3]));
    expectMatClose(fk1.transform, fk1again.transform);
    expect(fk2.transform).not.toEqual(fk1.transform);
  });

  it('tool transform is right-multiplied onto the chain product', () => {
    const links = planar3r([1, 0.7, 0.4], [0.1, 0.2, 0.3]);
    const tool: Mat4 = [
      [1, 0, 0, 0.1],
      [0, 1, 0, 0],
      [0, 0, 1, 0.05],
      [0, 0, 0, 1],
    ];
    const withTool = forwardKinematics(links, tool);
    const withoutTool = forwardKinematics(links);
    // position shifts by R_chain * tool translation
    const r = rotationOf(withoutTool.transform);
    const shift = [
      r[0][0] * 0.1 + r[0][2] * 0.05,
      r[1][0] * 0.1 + r[1][2] * 0.05,
      r[2][0] * 0.1 + r[2][2] * 0.05,
    ];
    for (let k = 0; k < 3; k++) {
      expect(withTool.position[k]).toBeCloseTo(withoutTool.position[k] + shift[k], 12);
    }
  });
});
