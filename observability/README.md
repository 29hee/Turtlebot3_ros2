# ObservabilityStack

A local, agent-agnostic observability stack that gives any coding agent
(Claude Code · Codex · OpenCode) a full **observe → reason → change → re-run**
feedback loop over logs, metrics, and traces.

> [!NOTE]
> In this repository the stack also ingests ROS2 node logs (`/rosout`) via the
> [`rosout_otel_bridge`](../pkgs/rosout_otel_bridge/README.md) package.
> See [docs/ROS2.md](./docs/ROS2.md) for the run/verify guide.

```
app (OTLP) ──> OpenTelemetry Collector ──fanout──> VictoriaLogs    (LogQL)
                                                ├─> VictoriaMetrics  (PromQL)
                                                └─> VictoriaTraces   (Jaeger query)
                                                       ▲
                          ./obs/*.sh  query tools  +  AGENTS.md  ┘   ← any agent
```

> **2026 design notes**
> - **OpenTelemetry Collector**, not Vector, is the fan-out: Vector's OTLP source
>   can't ingest metrics or export logs/metrics over OTLP.
> - **VictoriaTraces has no native TraceQL** yet — it's queried via the **Jaeger
>   query API** (`obs/traces.sh`).

## Quick start

**Plugging in your own app?** → see **[docs/CONNECT.md](./docs/CONNECT.md)**. Short version:
`make up` (infra only), then point your app at `http://localhost:4318` with
`OTEL_SERVICE_NAME=my-app`.

**Just want the self-contained demo?**

```bash
# 1. bring up infra + the bundled sample app
make demo           # = docker compose --profile demo up -d --build

# 2. generate traffic
./workload/run.sh 300

# 3. observe (these are the same tools the agent uses)
./obs/metrics.sh 'sum by (outcome) (orders_processed_total)'
./obs/logs.sh '_time:5m severity_text:error' 20
./obs/traces.sh search-errors sample-app

# 4. (optional) run the browser UI journey
cd e2e && npm install && npm run install-browsers && npm test
```

Open the sample app UI at <http://localhost:3000>.

> `make up` starts **only the shared infra** (collector + 3 stores) — the default
> for the bring-your-own-app workflow. `make demo` adds the sample app.

## What's in here

| Path | What it is |
|---|---|
| `docker-compose.yml` | Orchestrates Victoria 3종 + collector + app on the `dev-observability` network |
| `otel-collector/config.yaml` | OTLP receiver → fan-out to the three stores |
| `app/` | **Swappable** sample service (Node + zero-code OTel). Replace to observe your own app. |
| `obs/` | Agent query tools: `logs.sh` (LogQL), `metrics.sh` (PromQL), `traces.sh` (Jaeger), `correlate.sh` |
| `workload/run.sh` | Synthetic load generator |
| `e2e/` | Playwright browser UI journey |
| `AGENTS.md` | **Operating guide every agent reads** (`CLAUDE.md` symlinks to it) |

## Two ways to use it

- **Bring your own app** — point your service's OTLP exporter at
  `http://localhost:4318` (or `otel-collector:4318` inside the network) and
  replace `./app`. The query tools and docs are unchanged.
- **Self-contained demo** — use the included sample app; the `/api/checkout`
  endpoint has a built-in ~15% flaky failure rate for the agent to find and fix.

See **[AGENTS.md](./AGENTS.md)** for the full agent workflow.

## Teardown

```bash
docker compose down        # stop
docker compose down -v     # stop + wipe stored telemetry
```
