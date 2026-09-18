/**
 * 结构化、可区分的校验错误。
 * 所有非法入参都在进入运动学计算之前抛出本错误，错误中指明
 * 错误码、第几根杆（linkIndex，从 1 起）、哪个参数（parameter）、
 * 批量场景下第几组（caseIndex，从 0 起）以及可读说明。
 */

export type ValidationErrorCode =
  | 'INVALID_REQUEST'
  | 'MISSING_FIELD'
  | 'INVALID_TYPE'
  | 'NON_FINITE_VALUE'
  | 'LINK_COUNT_INVALID'
  | 'NEGATIVE_LINK_LENGTH'
  | 'JOINT_VECTOR_LENGTH_MISMATCH'
  | 'JOINT_TYPE_FREE_VARIABLE_MISMATCH'
  | 'INVALID_JOINT_TYPE'
  | 'INVALID_ANGLE_UNIT'
  | 'ANGLE_UNIT_CONFLICT'
  | 'INVALID_MATRIX'
  | 'ARM_ORDER_REVERSED'
  | 'ARM_DEFINITION_CONFLICT'
  | 'INVERSE_INVALID_PARAMS';

export interface ValidationErrorDetail {
  code: ValidationErrorCode;
  message: string;
  /** 从 1 开始的连杆序号。 */
  linkIndex?: number;
  /** 出问题的参数名，如 'a' | 'alpha' | 'd' | 'theta' | 'joints[1]'。 */
  parameter?: string;
  /** 批量请求中从 0 开始的组序号。 */
  caseIndex?: number;
  details?: Record<string, unknown>;
}

export class ValidationError extends Error {
  readonly code: ValidationErrorCode;
  readonly linkIndex?: number;
  readonly parameter?: string;
  readonly caseIndex?: number;
  readonly details?: Record<string, unknown>;

  constructor(detail: ValidationErrorDetail) {
    super(detail.message);
    this.name = 'ValidationError';
    this.code = detail.code;
    this.linkIndex = detail.linkIndex;
    this.parameter = detail.parameter;
    this.caseIndex = detail.caseIndex;
    this.details = detail.details;
  }

  toJSON(): ValidationErrorDetail & { error: true } {
    return {
      error: true,
      code: this.code,
      message: this.message,
      ...(this.caseIndex !== undefined ? { caseIndex: this.caseIndex } : {}),
      ...(this.linkIndex !== undefined ? { linkIndex: this.linkIndex } : {}),
      ...(this.parameter !== undefined ? { parameter: this.parameter } : {}),
      ...(this.details ? { details: this.details } : {}),
    };
  }
}

/** 便捷构造器：第几根杆的哪个参数出了什么问题。 */
export function linkError(
  code: ValidationErrorCode,
  linkIndex: number,
  parameter: string,
  message: string,
  extra?: Record<string, unknown>,
): ValidationError {
  return new ValidationError({ code, linkIndex, parameter, message, details: extra });
}
