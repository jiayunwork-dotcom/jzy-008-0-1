# kinematics-service

可独立部署的开链机械臂运动学核算服务。上游把各连杆的标准 DH 参数与当前关节矢量交给它，服务返回基座到末端的齐次变换、各轴原点坐标、连杆骨架点列，并对平面两转动关节臂给出逆解。纯服务端组件，无前端、无渲染。

## 钉死的约定（全服务唯一）

- **标准 DH**，每连杆齐次变换固定按 `A_i = RotZ(θ) · TransZ(d) · TransX(a) · RotX(α)` 连乘。
- 转动关节自由变量 = `theta`（关节角）；移动关节自由变量 = `d`（连杆偏距）。
- 服务内部角度统一为**弧度**。请求可声明 `angleUnit: "deg"`，在入口一次性换算为弧度后再参与任何三角函数。
- 姿态除旋转矩阵外，同时给出钉死的 **ZYX 欧拉角**（yaw/pitch/roll，弧度）便于阅读。
- 可选 `toolTransform`（固定 4×4 齐次变换）右乘到链乘结果上，缺省单位阵。
- 所有计算为纯函数：每次请求独立分配全部中间矩阵，相邻请求互不污染。

## 快速开始

```bash
# Docker 一键启动（服务 + Postgres）
docker compose up --build

# 或本地运行（默认 SQLite 持久化到 ./data/kinematics.db）
npm install
npm run dev          # 开发模式
npm run build && npm start

# 测试
npm test
```

环境变量：`PORT`（默认 8080）、`DATABASE_URL`（设置则用 Postgres，否则用 SQLite）、`SQLITE_PATH`（默认 `./data/kinematics.db`）。

## 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/health` | 运行状态（uptime、DB 连通性），供监控采集 |
| GET | `/api/convention` | 回显当前 DH 约定与角度单位 |
| GET | `/api/example/planar3r` | 预置算例：平面三杆、全零关节角，末端落在手算伸直位 |
| POST | `/api/fk` | 单臂正演：总变换 + 位置 + 旋转矩阵 + ZYX 欧拉角 + 各轴原点 + 骨架 |
| POST | `/api/skeleton` | 骨架点列（基座原点 + 各轴原点，顺序相连即折线） |
| POST | `/api/ik/planar2r` | 平面两杆逆解：肘上/肘下两支，均经正演回代验证 |
| POST | `/api/fk/batch` | 批量正演：单组非法不影响其余各组 |
| GET | `/api/history?kind=&ok=&since=&until=&limit=` | 历史核算记录查询 |

### 正演请求体

```json
{
  "angleUnit": "rad",
  "links": [
    { "jointType": "revolute", "a": 1.0, "alpha": 0, "d": 0 },
    { "jointType": "revolute", "a": 0.7, "alpha": 0, "d": 0 },
    { "jointType": "revolute", "a": 0.4, "alpha": 0, "d": 0 }
  ],
  "joints": [0.2, 0.3, 0.4],
  "toolTransform": [[1,0,0,0.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "allowNegativeLinkLength": false,
  "armSignature": "可选：调用方钉死的 DH 表 sha256 指纹"
}
```

`joints[i]` 是第 i 根杆的自由变量：转动关节为 θ，移动关节为 d；另一变量取连杆定义中的常量。响应中的 `armSignature` 可供调用方保存后在后续请求中钉死，若 DH 表顺序颠倒或被改动却声称是同一把臂，服务返回 `ARM_SIGNATURE_MISMATCH`。

### 逆解请求体

```json
{ "x": 1.1, "y": 0.6, "a1": 1.0, "a2": 0.7 }
```

返回 `solutions`（`elbow-up` / `elbow-down`，含正演回代误差 `fkError ≤ 1e-9`）。目标超出可达圆环返回 422 `UNREACHABLE`；两杆共线（完全伸直/折叠）返回 422 `SINGULAR`，绝不给出对不上正演的假角度。

## 结构化错误

计算前校验，错误体形如：

```json
{ "error": { "code": "NEGATIVE_LINK_LENGTH", "message": "Link 1: link length a=-0.5 is negative...", "details": { "linkIndex": 1, "parameter": "a", "value": -0.5 } } }
```

错误码包括：`MISSING_FIELD`、`INVALID_TYPE`、`NON_FINITE`、`TOO_FEW_LINKS`、`NEGATIVE_LINK_LENGTH`、`JOINT_LENGTH_MISMATCH`、`JOINT_TYPE_MISMATCH`、`ANGLE_UNIT_INVALID`、`ANGLE_UNIT_MISMATCH`（单位字段与数值互相矛盾）、`ARM_SIGNATURE_MISMATCH`、`INVALID_TOOL_TRANSFORM`、`UNREACHABLE`、`SINGULAR`。批量正演中非法组在 `results[i].error` 中就地标出第几组、哪根杆、哪个参数，其余组照常返回。

## 代码结构

```
src/
  math/mat4.ts            4×4 齐次变换、ZYX 欧拉角（纯函数）
  kinematics/dh.ts        标准 DH 单连杆变换
  kinematics/forward.ts   链乘正演、各轴原点、骨架点列
  kinematics/inverse2r.ts 平面两杆逆解（含正演回代验证）
  validation/validate.ts  入口校验 + 角度一次性换算 + 臂指纹
  store/                  历史持久化（SQLite / Postgres 双实现）
  routes/kinematics.ts    HTTP 路由
  app.ts / index.ts       Express 装配 / 进程入口
  example.ts              预置平面三杆算例
test/                     42 个自动化测试（vitest + supertest）
```

## 测试覆盖的运动学规则

全零伸直位、转动关节加 2π 总变换不变、杆长加倍位置加倍而旋转不变、末关节绕腕点转而腕点不动、移动关节沿轴线平移偏距增量、链乘与总变换一致、工具变换右乘、逆解两支正演贴合目标、超工作空间/共线奇异、各类非法参数拒绝、批量部分失败其余成功、历史持久化与条件查询、30 路并发请求互不串扰且历史记录不重复不丢失。
