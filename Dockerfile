# --- 多阶段构建 ---
FROM node:20-bookworm-slim AS builder
WORKDIR /app
COPY package*.json ./
RUN npm install --no-audit --no-fund
COPY tsconfig.json ./
COPY src ./src
RUN npm run build

FROM node:20-bookworm-slim AS runtime
WORKDIR /app
ENV NODE_ENV=production
ENV PORT=3000
ENV HOST=0.0.0.0
# 使用 docker-compose 提供的外部 Postgres
ENV KIN_STORAGE=pg
ENV PGHOST=postgres
ENV PGPORT=5432
ENV PGDATABASE=kinematics
ENV PGUSER=kinematics
ENV PGPASSWORD=kinematics

COPY package*.json ./
RUN npm install --omit=dev --no-audit --no-fund && npm cache clean --force
COPY --from=builder /app/dist ./dist

EXPOSE 3000
USER node
CMD ["node", "dist/index.js"]
