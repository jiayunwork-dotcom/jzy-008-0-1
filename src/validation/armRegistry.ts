/**
 * 机械臂结构登记表（进程内）。
 *
 * 用途：同一 armId 第一次出现时登记其 DH 结构签名；之后再次出现必须是同一把臂。
 * - 新签名恰好等于已登记签名的逐杆逆序 → ARM_ORDER_REVERSED（杆件顺序颠倒却声称同一把臂）；
 * - 其它不一致 → ARM_DEFINITION_CONFLICT；
 * - 一致或不带 armId 的请求 → 放行。
 *
 * 签名只包含结构量（与当前关节位形无关）：
 * 关节类型、杆长 a、扭角 alpha、转动关节的固定偏距 d、移动关节的固定角 theta。
 * 角度均为弧度（换算在进入本模块之前完成）。
 */
import { MaterializedLink } from '../kinematics/types';
import { ValidationError } from './errors';

const ROUND_DECIMALS = 9;

function roundKey(x: number): string {
  // 归一化 -0 与 0，并用固定小数位避免浮点尾数导致同一把臂对不上。
  return (Math.round(x * 1e9) / 1e9 || 0).toFixed(ROUND_DECIMALS);
}

export function linkStructuralToken(link: MaterializedLink): string {
  const fixed = link.jointType === 'revolute' ? `d=${roundKey(link.d)}` : `theta=${roundKey(link.theta)}`;
  return `${link.jointType}:a=${roundKey(link.a)}:alpha=${roundKey(link.alpha)}:${fixed}`;
}

export class ArmRegistry {
  private readonly arms = new Map<string, string[]>();

  /** 登记/核对 armId；armId 为空表示匿名请求，直接跳过。 */
  check(armId: string | null, links: MaterializedLink[]): void {
    if (!armId) return;
    const tokens = links.map(linkStructuralToken);

    const known = this.arms.get(armId);
    if (!known) {
      this.arms.set(armId, tokens);
      return;
    }

    if (sameSequence(known, tokens)) return;

    const reversed = [...tokens].reverse();
    if (sameSequence(known, reversed)) {
      throw new ValidationError({
        code: 'ARM_ORDER_REVERSED',
        message:
          `armId='${armId}' 的连杆顺序与首次登记时完全颠倒：本次 ${tokens.length} 根杆按逆序排列却声称是同一把臂。` +
          `开链 DH 必须按从基座到末端的固定顺序提交。`,
        details: { armId, registered: known, received: tokens },
      });
    }

    throw new ValidationError({
      code: 'ARM_DEFINITION_CONFLICT',
      message:
        `armId='${armId}' 的 DH 结构与首次登记不一致，却声称是同一把臂。` +
        `如确为另一把臂请使用新的 armId。`,
      details: { armId, registered: known, received: tokens },
    });
  }

  /** 仅供测试/重置使用。 */
  clear(): void {
    this.arms.clear();
  }
}

function sameSequence(a: string[], b: string[]): boolean {
  return a.length === b.length && a.every((v, i) => v === b[i]);
}
