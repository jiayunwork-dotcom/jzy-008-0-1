/**
 * 输入校验测试：所有非法入参都必须被拒绝并指出哪根杆、哪个参数。
 */
import { validateForward, validateInverse } from '../src/validation/validator';
import { ArmRegistry } from '../src/validation/armRegistry';
import { ValidationError } from '../src/validation/errors';
import { RawForwardRequest } from '../src/validation/types';

function expectReject(raw: unknown, code: string, linkIndex?: number): ValidationError {
  let thrown: unknown;
  try {
    validateForward(raw as RawForwardRequest, new ArmRegistry());
  } catch (e) {
    thrown = e;
  }
  expect(thrown).toBeInstanceOf(ValidationError);
  const ve = thrown as ValidationError;
  expect(ve.code).toBe(code);
  if (linkIndex !== undefined) expect(ve.linkIndex).toBe(linkIndex);
  return ve;
}

const okTwo = (over: Partial<RawForwardRequest> = {}): RawForwardRequest => ({
  links: [
    { jointType: 'revolute', a: 1, alpha: 0 },
    { jointType: 'revolute', a: 1, alpha: 0 },
  ],
  joints: [0, 0],
  ...over,
});

describe('正演输入校验', () => {
  test('杆件数少于 2 被拒（0 根 / 1 根）', () => {
    expectReject({ links: [], joints: [] }, 'LINK_COUNT_INVALID');
    expectReject({ links: [{ jointType: 'revolute', a: 1 }], joints: [0] }, 'LINK_COUNT_INVALID');
  });

  test('关节矢量长度与杆件数不一致被拒并给出两个长度', () => {
    const ve = expectReject(
      {
        links: [
          { jointType: 'revolute', a: 1, alpha: 0 },
          { jointType: 'revolute', a: 1, alpha: 0 },
          { jointType: 'revolute', a: 1, alpha: 0 },
        ],
        joints: [0, 0],
      },
      'JOINT_VECTOR_LENGTH_MISMATCH',
    );
    expect(ve.details).toMatchObject({ jointsLength: 2, linkCount: 3 });
  });

  test('缺字段：links / joints / a / jointType', () => {
    expectReject({ joints: [0, 0] }, 'MISSING_FIELD');
    expectReject({ links: okTwo().links }, 'MISSING_FIELD');
    expectReject(
      { links: [{ jointType: 'revolute' }, { jointType: 'revolute', a: 1 }], joints: [0, 0] },
      'MISSING_FIELD',
      1,
    );
    expectReject(
      { links: [{ a: 1 }, { jointType: 'revolute', a: 1 }], joints: [0, 0] },
      'MISSING_FIELD',
      1,
    );
  });

  test('非数值 / 非有限值（NaN、Infinity、字符串）被拒并指明参数', () => {
    expectReject(okTwo({ links: [{ jointType: 'revolute', a: NaN }, { jointType: 'revolute', a: 1 }] }), 'NON_FINITE_VALUE', 1);
    expectReject(okTwo({ links: [{ jointType: 'revolute', a: Infinity }, { jointType: 'revolute', a: 1 }] }), 'NON_FINITE_VALUE', 1);
    expectReject(okTwo({ links: [{ jointType: 'revolute', a: '1' as unknown as number }, { jointType: 'revolute', a: 1 }] }), 'NON_FINITE_VALUE', 1);
    expectReject(okTwo({ joints: [0, NaN] }), 'NON_FINITE_VALUE', 2);
    expectReject(
      okTwo({ links: [{ jointType: 'revolute', a: 1, alpha: 'not-num' as unknown as number }, { jointType: 'revolute', a: 1 }] }),
      'NON_FINITE_VALUE',
      1,
    );
  });

  test('负杆长默认拒绝，声明允许后放行', () => {
    expectReject(
      okTwo({ links: [{ jointType: 'revolute', a: -1 }, { jointType: 'revolute', a: 1 }] }),
      'NEGATIVE_LINK_LENGTH',
      1,
    );
    // 请求级放行
    const req = validateForward(
      okTwo({ allowNegativeLengths: true, links: [{ jointType: 'revolute', a: -1 }, { jointType: 'revolute', a: 1 }] }) as RawForwardRequest,
      new ArmRegistry(),
    );
    expect(req.links[0].a).toBe(-1);
    // 单杆级放行
    const req2 = validateForward(
      okTwo({ links: [{ jointType: 'revolute', a: -1, allowNegativeLength: true }, { jointType: 'revolute', a: 1 }] }) as RawForwardRequest,
      new ArmRegistry(),
    );
    expect(req2.links[0].a).toBe(-1);
  });

  test('关节类型非法被拒', () => {
    expectReject(
      okTwo({ links: [{ jointType: 'continuous', a: 1 }, { jointType: 'revolute', a: 1 }] }),
      'INVALID_JOINT_TYPE',
      1,
    );
  });

  test('关节类型与自由变量对不上被拒（转动声明 d / 移动声明 theta）', () => {
    expectReject(
      okTwo({ joints: [{ value: 0.1, freeVariable: 'd' }, 0] }),
      'JOINT_TYPE_FREE_VARIABLE_MISMATCH',
      1,
    );
    expectReject(
      okTwo({
        links: [{ jointType: 'prismatic', a: 0.5 }, { jointType: 'revolute', a: 1 }],
        joints: [{ value: 0.1, freeVariable: 'theta' }, 0],
      }),
      'JOINT_TYPE_FREE_VARIABLE_MISMATCH',
      1,
    );
  });

  test('移动关节关节值驱动 d（偏距），转动关节驱动 theta', () => {
    const req = validateForward(
      okTwo({
        links: [{ jointType: 'prismatic', a: 0.5, d: 0.2 }, { jointType: 'revolute', a: 1, d: 0.1 }],
        joints: [{ value: 0.3, freeVariable: 'd' }, { value: 0.4, freeVariable: 'theta' }],
      }) as RawForwardRequest,
      new ArmRegistry(),
    );
    expect(req.links[0].d).toBeCloseTo(0.5, 12);
    expect(req.links[0].theta).toBe(0);
    expect(req.links[1].theta).toBeCloseTo(0.4, 12);
    expect(req.links[1].d).toBe(0.1);
  });

  test('角度单位字段非法 / 与数值矛盾（度数值配 radian 单位）被拒或不等价', () => {
    expectReject(okTwo({ angleUnit: 'grad' as unknown as 'radian' }), 'INVALID_ANGLE_UNIT');
    // 不矛盾但语义错配：90 配 radian 不会报“矛盾”，这里验证它确实被当 90 弧度处理（不做猜测）
    const r1 = validateForward(okTwo({ joints: [90, 0], angleUnit: 'radian' }) as RawForwardRequest, new ArmRegistry());
    const r2 = validateForward(okTwo({ joints: [90, 0], angleUnit: 'degree' }) as RawForwardRequest, new ArmRegistry());
    expect(r1.links[0].theta).toBe(90);
    expect(r2.links[0].theta).toBeCloseTo(Math.PI / 2, 12);
  });

  test('单根杆角度单位与请求级矛盾被拒（ANGLE_UNIT_CONFLICT）', () => {
    expectReject(
      okTwo({
        angleUnit: 'radian',
        links: [
          { jointType: 'revolute', a: 1, angleUnit: 'degree' },
          { jointType: 'revolute', a: 1 },
        ],
      }),
      'ANGLE_UNIT_CONFLICT',
      1,
    );
  });

  test('度在入口一次性换算：alpha 也按度换算', () => {
    const req = validateForward(
      okTwo({
        angleUnit: 'degree',
        links: [
          { jointType: 'revolute', a: 1, alpha: 90 },
          { jointType: 'revolute', a: 1, alpha: 0 },
        ],
        joints: [0, 0],
      }) as RawForwardRequest,
      new ArmRegistry(),
    );
    expect(req.links[0].alpha).toBeCloseTo(Math.PI / 2, 12);
  });

  test('工具矩阵形状 / 非有限值被拒', () => {
    expectReject(okTwo({ tool: [[1, 0, 0, 0]] }), 'INVALID_MATRIX');
    expectReject(okTwo({ tool: 'bad' }), 'INVALID_MATRIX');
    expectReject(
      okTwo({
        tool: [
          [1, 0, 0, NaN],
          [0, 1, 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1],
        ],
      }),
      'NON_FINITE_VALUE',
    );
  });

  test('同一 armId 顺序颠倒声称同一把臂 -> ARM_ORDER_REVERSED', () => {
    const reg = new ArmRegistry();
    const linksFwd = [
      { jointType: 'revolute' as const, a: 1, alpha: 0 },
      { jointType: 'revolute' as const, a: 2, alpha: 0 },
      { jointType: 'prismatic' as const, a: 0, alpha: 0 },
    ];
    validateForward({ armId: 'arm-x', links: linksFwd, joints: [0, 0, 0] }, reg);
    let thrown: unknown;
    try {
      validateForward(
        {
          armId: 'arm-x',
          links: [...linksFwd].reverse(),
          joints: [0, 0, 0],
        },
        reg,
      );
    } catch (e) {
      thrown = e;
    }
    expect(thrown).toBeInstanceOf(ValidationError);
    expect((thrown as ValidationError).code).toBe('ARM_ORDER_REVERSED');
  });

  test('同一 armId 结构变更（非同序非逆序）-> ARM_DEFINITION_CONFLICT', () => {
    const reg = new ArmRegistry();
    validateForward({ armId: 'arm-y', links: okTwo().links, joints: [0, 0] }, reg);
    let thrown: unknown;
    try {
      validateForward(
        {
          armId: 'arm-y',
          links: [
            { jointType: 'revolute', a: 1, alpha: 0 },
            { jointType: 'revolute', a: 9, alpha: 0 },
          ],
          joints: [0, 0],
        },
        reg,
      );
    } catch (e) {
      thrown = e;
    }
    expect((thrown as ValidationError).code).toBe('ARM_DEFINITION_CONFLICT');
  });

  test('同一 armId 仅关节位形不同（结构不变）放行', () => {
    const reg = new ArmRegistry();
    validateForward({ armId: 'arm-z', links: okTwo().links, joints: [0, 0] }, reg);
    expect(() => validateForward({ armId: 'arm-z', links: okTwo().links, joints: [0.5, -0.2] }, reg)).not.toThrow();
  });
});

describe('逆解输入校验', () => {
  test('缺字段 / 非正杆长 / 非有限目标 / target 形状错误', () => {
    const inv = (raw: unknown) => validateInverse(raw as never);
    expect(() => inv({ L2: 1, target: [1, 1] })).toThrow(ValidationError);
    expect(() => inv({ L1: 0, L2: 1, target: [1, 1] })).toThrow(ValidationError);
    expect(() => inv({ L1: -1, L2: 1, target: [1, 1] })).toThrow(ValidationError);
    expect(() => inv({ L1: 1, L2: 1, target: [1, NaN] })).toThrow(ValidationError);
    expect(() => inv({ L1: 1, L2: 1, target: [1] })).toThrow(ValidationError);
  });
});
