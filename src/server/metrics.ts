/**
 * 轻量运行指标（进程内）。供 /ready 与 /metrics 采集。
 * 并发安全依赖 JS 事件循环单线程；计数器自增不会交错。
 */
export class Metrics {
  readonly startedAt = Date.now();
  private counters = new Map<string, number>();
  private inFlight = 0;

  inc(name: string, by = 1): void {
    this.counters.set(name, (this.counters.get(name) ?? 0) + by);
  }

  requestStarted(): void {
    this.inFlight++;
    this.inc('http_requests_total');
  }

  requestFinished(status: number): void {
    this.inFlight--;
    this.inc(`http_status_${status}`);
  }

  uptimeSeconds(): number {
    return (Date.now() - this.startedAt) / 1000;
  }

  snapshot(): {
    uptimeSeconds: number;
    inFlight: number;
    startedAt: string;
    counters: Record<string, number>;
  } {
    return {
      uptimeSeconds: this.uptimeSeconds(),
      inFlight: this.inFlight,
      startedAt: new Date(this.startedAt).toISOString(),
      counters: Object.fromEntries(this.counters.entries()),
    };
  }

  /** Prometheus 文本格式。 */
  prometheus(): string {
    const s = this.snapshot();
    const lines: string[] = [];
    lines.push('# HELP kinematics_uptime_seconds 服务运行秒数');
    lines.push('# TYPE kinematics_uptime_seconds gauge');
    lines.push(`kinematics_uptime_seconds ${s.uptimeSeconds}`);
    lines.push('# HELP kinematics_inflight_requests 处理中的请求数');
    lines.push('# TYPE kinematics_inflight_requests gauge');
    lines.push(`kinematics_inflight_requests ${s.inFlight}`);
    lines.push('# HELP kinematics_http_requests_total HTTP 请求计数（含各状态码维度）');
    lines.push('# TYPE kinematics_http_requests_total counter');
    for (const [k, v] of Object.entries(s.counters)) {
      if (k === 'http_requests_total') {
        lines.push(`kinematics_http_requests_total ${v}`);
      } else if (k.startsWith('http_status_')) {
        lines.push(`kinematics_http_requests_total{status="${k.slice('http_status_'.length)}"} ${v}`);
      }
    }
    return lines.join('\n') + '\n';
  }
}
