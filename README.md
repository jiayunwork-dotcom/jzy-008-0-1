# 开链机械臂标准 DH 正向运动学核算服务

可独立部署的纯服务端运动学计算组件（TypeScript + Express + 数据库）。
只做一件事：接收各连杆的标准 DH 参数与当前关节矢量，算出基座到末端的齐次变换、
姿态（旋转矩阵 + 钉死欧拉角）、各轴原点坐标与连杆骨架点列；并对平面两转动关节臂
给出肘上/肘下逆解。**不含**前端页面、示教盒、设备台账或三维渲染。

## 1. 钉死的约定

- **标准 DH（唯一约定，不可切换）**，单杆变换固定连乘顺序：

  ```
  A_i = Rz(theta_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)
  ```

  即：绕 z 轴旋转关节角 → 沿 z 轴平移连杆偏距 → 沿 x 轴平移连杆长度 → 绕 x 轴旋转扭角。
  代码在 `src/kinematics/transforms.ts` 中以四个原语矩阵显式连乘实现，并与闭式标准 DH
  矩阵交叉复核。
- 基座到末端总变换：`T = A_1 · A_2 · … · A_n`；工具坐标系再**右乘**固定工具变换
  `T_end = T · T_tool`，工具缺省为单位阵。
- 转动关节（revolute）自由变量 = 关节角 **theta**；移动关节（prismatic）自由变量 =
  连杆偏距 **d**。
- 角度服务内统一为**弧度**。请求可声明 `angleUnit: "degree"`，度在**入口一次性换算**
  后再参与三角函数；一次请求所有连杆只能用同一种单位（单杆单位与请求级矛盾会被拒绝）。
- 姿态阅读约定钉死为**内禀 ZYX 欧拉角**（yaw-pitch-roll）：
  `R = Rz(yaw)·Ry(pitch)·Rx(roll)`，万向锁分支取 yaw=0。
- 钉死容差：矩阵一致性 `1e-10`，位置正演回贴 `1e-8`。

## 2. 模块结构

```
src/
  kinematics/            # 纯函数运动学核心，无 IO、无模块级可变状态
    constants.ts         # 钉死容差与约定描述
    types.ts             # 内部数据类型
    math.ts              # 齐次矩阵运算（乘法、连乘、平移/旋转提取、误差）
    transforms.ts        # Rz/Tz/Tx/Rx 原语与标准 DH 单杆变换（+闭式复核）
    euler.ts             # 旋转矩阵 -> 内禀 ZYX 欧拉角
    chain.ts             # 连杆链乘 / 正演 / 骨架点列 / 前缀轴原点 / 一致性复核
    inverse.ts           # 平面两杆逆解（每支解用同一正演引擎回贴校验）
  validation/
    errors.ts            # 结构化可区分错误（错误码 + 杆号 + 参数名 + 组号）
    types.ts             # HTTP 原始入参类型
    armRegistry.ts       # 同 armId 的结构一致性/逆序识别
    validator.ts         # 全部入参校验与度->弧度入口换算
  storage/
    types.ts             # HistoryStore 接口
    schema.ts            # 建表 SQL 与行映射（pg / PGlite 共用）
    pgStore.ts           # node-postgres 实现（docker-compose 默认）
    pgliteStore.ts       # PGlite 嵌入式 Postgres 实现（本地/零依赖独立部署）
    memoryStore.ts       # 内存实现（测试 / KIN_STORAGE=memory）
    factory.ts
  services/
    kinematicsService.ts # 校验 -> 计算 -> 序列化 -> 持久化的编排
  server/
    app.ts               # Express 路由与统一错误处理
    metrics.ts           # 运行指标
    index?               # （入口在 src/index.ts）
tests/                   # Jest 自动化测试（65 个用例）
```

## 3. 快速开始

```bash
npm install
npm test          # 运行自动化测试
npm run build
npm start         # 默认 PGlite 嵌入式落盘到 ./data/pglite，无需任何外部依赖
# 开发模式： npm run dev
```

### Docker Compose（服务 + Postgres 一键启动）

```bash
docker compose up --build
# 服务在 http://localhost:3000，历史写入 postgres:5432/kinematics
```

健康检查：

```bash
curl http://localhost:3000/health/live
curl http://localhost:3000/health/ready     # 同时探测数据库
curl http://localhost:3000/metrics          # Prometheus 文本
curl http://localhost:3000/metrics/json
```

## 4. HTTP 接口

所有接口前缀 `/api/v1`。错误响应 HTTP 400，形如：

```json
{ "error": true, "code": "NEGATIVE_LINK_LENGTH", "message": "…",
  "linkIndex": 2, "parameter": "a" }
```

批量场景额外带 `caseIndex`（从 0 起）；`linkIndex` 从 1 起。

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET  | `/api/v1/convention` | 回显当前 DH 约定与角度单位 |
| POST | `/api/v1/fk` | 单臂正向运动学 |
| POST | `/api/v1/skeleton` | 骨架点列 / 各轴原点 |
| POST | `/api/v1/batch` | 多组关节角批量正演（逐组独立成败） |
| POST | `/api/v1/inverse` | 平面两杆逆解 |
| GET  | `/api/v1/history` | 历史查询（类型/状态/armId/时间/分页） |
| GET  | `/api/v1/presets/planar3r` | 预置平面三杆算例（载荷 + 手算期望） |
| GET  | `/health/live` `/health/ready` `/metrics` | 运行状态 |

### 4.1 正演 `POST /api/v1/fk`

请求字段：

- `links[]`（≥2）：`jointType`(`revolute|prismatic`)、`a`(杆长)、`alpha`(扭角)，
  可选 `d`、`theta`（结构量，缺省 0）、`allowNegativeLength`、`angleUnit`。
- `joints[]`：长度必须等于杆件数。元素可以是裸数值；
  转动关节驱动 `theta`，移动关节驱动 `d`（在结构基准值上叠加）。
  也可显式写 `{ "value": 0.1, "freeVariable": "d" }`，自由变量与关节类型对不上会被拒。
- `angleUnit`：`radian`（默认）或 `degree`，整次请求统一。
- `tool`：可选 4×4 固定工具变换（右乘）。
- `armId`：可选。同一 id 的 DH 结构必须一致；**杆件逆序**却声称同一把臂、
  或结构变更，分别返回 `ARM_ORDER_REVERSED` / `ARM_DEFINITION_CONFLICT`。
- `allowNegativeLengths`：请求级统一放行负杆长（否则 `a<0` 返回
  `NEGATIVE_LINK_LENGTH`）。

```bash
curl -s -X POST http://localhost:3000/api/v1/fk \
  -H 'content-type: application/json' \
  -d '{
    "armId": "planar3r",
    "angleUnit": "radian",
    "links": [
      {"jointType":"revolute","a":1,"alpha":0},
      {"jointType":"revolute","a":1,"alpha":0},
      {"jointType":"revolute","a":1,"alpha":0}
    ],
    "joints": [0,0,0]
  }'
```

响应（节选）：

```json
{
  "ok": true, "requestId": 1, "armId": "planar3r", "angleUnit": "radian",
  "convention": "standard-dh",
  "result": {
    "linkCount": 3,
    "totalTransform": [[1,0,0,3],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    "endEffectorTransform": [ ... ],
    "position": [3, 0, 0],
    "rotation": [[1,0,0],[0,1,0],[0,0,1]],
    "euler": { "roll": 0, "pitch": 0, "yaw": 0 },
    "links": [ { "index":1, "jointType":"revolute", "transform":[...], "origin":[1,0,0] }, ... ],
    "axisOrigins": [[1,0,0],[2,0,0],[3,0,0]],
    "skeleton": [[0,0,0],[1,0,0],[2,0,0],[3,0,0]],
    "tool": [ ... 单位阵 ... ],
    "chainCheckMaxError": 0,
    "chainCheckTolerance": 1e-10
  }
}
```

度单位示例（入口换算后内部只有弧度）：

```json
{ "angleUnit": "degree",
  "links": [{"jointType":"revolute","a":1,"alpha":0},{"jointType":"revolute","a":1,"alpha":0}],
  "joints": [90, 0] }
```

移动关节（第二杆沿自身关节轴平移 0.25）：

```json
{ "links": [
    {"jointType":"revolute","a":0,"alpha":-1.5707963267948966},
    {"jointType":"prismatic","a":0.4,"alpha":0,"d":0}
  ],
  "joints": [0.5235987755982988, {"value":0.25,"freeVariable":"d"}] }
```

### 4.2 骨架 `POST /api/v1/skeleton`

请求体同正演，返回 `skeleton`（基座原点起的折线点列）、`axisOrigins`、`endPosition`。

### 4.3 批量正演 `POST /api/v1/batch`

```json
{ "angleUnit": "radian",
  "cases": [
    { "name": "home", "links": [ ... ], "joints": [0,0] },
    { "name": "bad",  "links": [ ... ], "joints": [0] }
  ] }
```

HTTP 恒为 200；逐组结果在 `results[]` 内：成功项 `ok:true, result`，
失败项 `ok:false, error{ code, message, caseIndex, linkIndex, parameter }`。
某组非法**不影响其余组**，并汇总 `successCount/failureCount`。

### 4.4 平面两杆逆解 `POST /api/v1/inverse`

```json
{ "L1": 1, "L2": 1, "target": [1.2, 0.5] }
```

成功返回两支 `solutions[]`，每支含 `branch`(`elbow-up|elbow-down`)、`joints`[theta1,theta2]
（弧度）、用本服务正演引擎回算的 `fkPosition` 与回贴残差 `residual`（≤ 1e-8 才会作为解）。

- 超出可达圆环 `[|L1-L2|, L1+L2]`：`status:"no-solution"`，`solutions:[]`，带原因；
- 两杆共线（伸直/等长折叠）：`status:"singular"`，说明分支不可区分；
- 以上业务状态 HTTP 均为 200（不是错误）；只有入参非法才是 400。

### 4.5 历史查询 `GET /api/v1/history`

查询参数：`type`(forward|skeleton|batch|inverse)、`status`(ok|error)、`armId`、
`since`/`until`(epoch 毫秒)、`limit`(默认 50)、`offset`。每次核算请求（成功与失败）
都落库，包含请求/响应快照、杆数、组数、错误码、耗时与时间戳，按 id 倒序返回。

### 4.6 约定回显 `GET /api/v1/convention`

返回标准 DH 公式与原语顺序、角度单位规则、欧拉约定、容差、各关节类型的自由变量。

### 4.7 预置算例 `GET /api/v1/presets/planar3r`

平面三杆、`L1=L2=L3=1`、全零关节角；手算伸直位为位置 `[3,0,0]`、姿态无扭转、
骨架 `[0,0,0]→[1,0,0]→[2,0,0]→[3,0,0]`。返回体含可直接 POST 给 `/fk` 的 `input`
与 `expected`。

## 5. 结构化错误码

| code | 触发条件 |
| --- | --- |
| `MISSING_FIELD` | 缺 `links`/`joints`/`a`/`jointType` 等必填字段 |
| `INVALID_TYPE` | 类型错误（含 target 形状、分页参数等） |
| `NON_FINITE_VALUE` | NaN / Infinity / 非数值，指出杆号与参数 |
| `LINK_COUNT_INVALID` | 杆件数 < 2 |
| `NEGATIVE_LINK_LENGTH` | 杆长为负且未显式允许 |
| `JOINT_VECTOR_LENGTH_MISMATCH` | 关节矢量长度 ≠ 杆件数（附两个长度） |
| `JOINT_TYPE_FREE_VARIABLE_MISMATCH` | 转动声明驱动 d / 移动声明驱动 theta |
| `INVALID_JOINT_TYPE` | jointType 非 revolute/prismatic |
| `INVALID_ANGLE_UNIT` / `ANGLE_UNIT_CONFLICT` | 角度单位非法 / 单杆与请求级矛盾 |
| `INVALID_MATRIX` | tool 不是 4×4 数值矩阵（非有限值另报 NON_FINITE_VALUE） |
| `ARM_ORDER_REVERSED` / `ARM_DEFINITION_CONFLICT` | 同 armId 逆序 / 结构不一致 |
| `INVERSE_INVALID_PARAMS` | 逆解杆长非正等 |
| `INVALID_REQUEST` | 请求体非法 / 非法 JSON 等 |

## 6. 已验证的运动学性质（自动化测试）

`npm test` 覆盖：

- 平面三杆全零伸直位（任意杆长）；
- 任一转动关节加 2π，总变换不变；
- 所有杆长同时加倍 → 位置坐标加倍、旋转不变；
- 只改最后一个关节角 → 末端绕腕点转、腕点基座坐标不变；
- 服务给出的总变换 = 逐杆变换顺序连乘（双路径独立复核，误差 ≤ 1e-10）；
- 工具变换右乘生效、缺省为单位阵；
- 移动关节只改偏距时末端沿该关节轴线平移、距离等于偏距增量、姿态不变；
- 度在入口换算与弧度结果一致；
- 逆解两支各自正演回贴目标（多点扫描）；超工作空间 `no-solution`；
  伸直/折叠共线 `singular`，绝不返回对不上正演的假角度；
- 全部非法入参被结构化拒绝；批量部分失败其余成功；
- 每次请求历史持久化并可按条件查询；
- 20 个并发交错请求结果互不串扰、历史记录一一对应。

## 7. 并发与隔离

- 运动学层全部为纯函数，每次调用在栈上新建全部中间矩阵，无模块级缓存，
  相邻请求不可能互相污染。
- 请求序号在单线程事件循环内自增；历史写入带唯一 `BIGSERIAL` 主键。
- 数据库连接由池管理；批量中逐组独立物化，失败组不登记 arm 结构。

## 8. 配置

见 `.env.example`：`PORT`、`HOST`、`KIN_STORAGE`（`pglite` 默认 / `pg` / `memory`）、
`KIN_DATA_DIR`、`DATABASE_URL` 或 `PGHOST/PGPORT/PGDATABASE/PGUSER/PGPASSWORD`。
