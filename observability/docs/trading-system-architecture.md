# 통합 트레이딩 시스템 아키텍처 (Market Wizards → 전략 → 라이브 → 관측)

> 상태: **설계 (Draft v1)** · 작성 기준일 2026-06-08
> 범위: dolsoe(라이브 엔진) · freqtrade(연구 랩) · ccxt(거래소 레이어) · ObservabilityStack(관측) ·
> Market Wizards 3부작(전략 지식 원천)을 하나의 전략 개발·운용·관측 루프로 엮는다.
> 모든 에이전트 작업은 **paper/가상** 기준. 라이브 실거래는 dolsoe 정책상 **operator-only/manual**
> (키 주입·`--live` 실행·reconcile 우회는 운영자만 수행, 에이전트 금지).

---

## 1. 목표

코인 차트별로 **추세 전략**과 **횡보 전략**을 각각 구축·저장하고, 여러 조합으로 합성하고,
파라미터와 타임프레임을 튜닝하여 — **강한 추세는 끝까지 먹고, 횡보장은 단기로 회전시켜 먹는** —
regime-적응형 전략 포트폴리오를 만든다. 지식의 출발점은 잭 슈웨거 *Market Wizards* 3부작이며,
검증의 출발점은 freqtrade 백테스트, 권위 있는 검증은 dolsoe 백테스트 매트릭스, 운용은 dolsoe
라이브, 전 과정의 가시성은 ObservabilityStack이 담당한다.

---

## 2. 핵심 통찰 (설계를 지배하는 3가지)

### 2.1 책은 "기계적 전략"이 아니라 "원리"를 준다
Market Wizards 시리즈는 트레이더 인터뷰집이다. 매수/매도 규칙이 아니라 **원리**(추세 추종,
손실 조기 절단, 비대칭 손익비, 포지션 사이징, 시장 심리, 돌파/되돌림)를 담는다. 따라서
파이프라인의 첫 단계는 "책 → 검증 가능한 가설(rule)"로의 **번역**이다. 정성적 지혜를
`docs/research/`의 가설 카드로 구조화한 뒤에야 백테스트 가능한 전략이 된다.

### 2.2 freqtrade와 dolsoe는 패러다임이 다르다 — 이식은 "코드"가 아니라 "인사이트"
- **freqtrade**: pair당 단일 entry/exit 신호 모델. 다거래소·hyperopt·플로팅이 강점. → **넓고 싼 탐색(lab)**.
- **dolsoe**: 5중 Gate + 다단계 DCA tier + regime 엔진 + hedge mode. 라이브 실행 의미와 1:1. → **좁고 충실한 검증·운용(factory)**.

이 둘은 전략 표현이 근본적으로 다르므로 freqtrade strategy를 dolsoe로 코드 변환하지 않는다.
freqtrade에서는 **지표·임계값·regime 분류·타임프레임 같은 인사이트**를 빠르게 탐색하고,
그 결과를 *방향성 가설*로만 받아들여(=dolsoe CLAUDE.md의 "direction-only는 가설" 규칙) dolsoe의
gate/tier/flag로 재구현한 뒤, dolsoe 자체 backtest matrix로 권위 있게 재검증한다.

### 2.3 목표 달성에 필요한 골격은 dolsoe에 이미 있다
새 엔진을 만들지 않는다. 사용자 목표는 dolsoe 기존 구조에 직접 매핑된다:

| 사용자 목표 | dolsoe 구성요소 | 현재 성숙도 |
|---|---|---|
| 횡보장 단타로 다 먹기 | Envelope 평균회귀 8-tier (`core/envelope.py`) | 성숙 (라이브 핵심) |
| 강한 추세 다 먹기 | `TrendStrategy` Donchian/ATR (`core/strategy/trend_strategy.py`) | opt-in, 미성숙 → **집중 개발 대상** |
| 전략 조합/합성 | `StrategyPlugin` ABC + `StrategyRouter`(자본 배분) | 골격 존재, 정적 weight |
| regime별 전략 선택 | regime detection (PANIC/RALLY/TRENDING_UP·DOWN/RANGE) | 존재 (kill switch 보유) |
| 값 조정 | `run_ablation_matrix.py` + 신규 freqtrade hyperopt | 존재 |
| 시간 조절(타임프레임) | `walkforward_sweep.py` | 존재 |
| 코인 차트별 | 심볼별 파라미터(44종) | 존재 |

→ **작업의 본질**: ① Trend 엔진을 책+연구로 강화, ② Router를 regime-조건부로 진화,
③ 두 전략을 합성·튜닝, ④ 전 과정을 관측.

---

## 3. 구성요소 & 역할

| 구성요소 | 위치 | 역할 | 라이브 권한 |
|---|---|---|---|
| **Market Wizards 3부작** | `~/Documents/Book/*.epub` | 전략 원리의 1차 지식 원천 | — |
| **freqtrade** | `freqtrade-lab/` (신규, docker image + user_data) | 전략 **연구/백테스트/hyperopt 전용**. 항상 dry-run | **절대 라이브 금지** (trade 권한 키 미주입) |
| **ccxt** | freqtrade 내장 + dolsoe `core/exchange/` | 거래소 통합. **Bitget을 정본 거래소로 통일** | dolsoe만 라이브 키 사용 |
| **dolsoe** | `~/Desktop/dolsoe` | 전략 운용 엔진. paper(에이전트) / live(operator-only) | live = operator-only |
| **ObservabilityStack** | `~/Desktop/ObservabilityStack` | logs/metrics/traces 통합 관측. observe→reason→change 루프 | read-only (관측은 절대 매매 안 함) |

---

## 4. 토폴로지

### 4.1 신호 흐름 (런타임)

```
                 ┌──────────────────── 지식/연구 평면 (offline) ───────────────────┐
 Market Wizards  │  epub → 원리 추출 → 가설 카드(docs/research) → freqtrade 전략     │
   *.epub  ─────▶│                         │ backtest/hyperopt (Bitget OHLCV)       │
                 │                         ▼                                        │
                 │                  방향성 가설(검증 대상 파라미터/지표/regime)        │
                 └─────────────────────────┬───────────────────────────────────────┘
                                           │ 이식 (코드 아님, 인사이트)
                                           ▼
                 ┌──────────────────── 운용 평면 (dolsoe) ──────────────────────────┐
 ccxt(Bitget) ──▶│  MarketData → [Envelope(횡보) ‖ Trend(추세)] → StrategyRouter     │
                 │                         → 5중 Gate → 주문 실행 → Position/CB/Reconcile │
                 │   paper(가상주문) ‖ live(operator-only, hedge mode)               │
                 └─────────────────────────┬───────────────────────────────────────┘
                                           │ OTLP (logs/metrics/traces)
 freqtrade(dry-run) ───OTLP──────────────▶ │
                                           ▼
                 ┌──────────────────── 관측 평면 (ObservabilityStack) ───────────────┐
                 │ otel-collector :4318 ──fanout──▶ VictoriaLogs   :9428 (LogQL)      │
                 │                              ├─▶ VictoriaMetrics :8428 (PromQL)     │
                 │                              └─▶ VictoriaTraces  :10428 (Jaeger)    │
                 │     에이전트/운영자 ◀── ./obs/*.sh (logs/metrics/traces/correlate)  │
                 └───────────────────────────────────────────────────────────────────┘
```

### 4.2 리포지토리/프로세스 배치

```
~/Desktop/
├── ObservabilityStack/      # 관측 인프라 (always-on: make up). OTLP 4317/4318 → localhost 공개
│   └── docs/trading-system-architecture.md   ← 본 문서
├── dolsoe/                  # 라이브 엔진 (host 실행). OTLP → localhost:4318
├── freqtrade-lab/           # 신규: freqtrade 연구 랩 (docker image + user_data, dry-run only)
│   ├── docker-compose.yml   #   dev-observability 네트워크 join, OTLP → otel-collector:4318
│   ├── user_data/{config.bitget.json, strategies/, hyperopts/, backtest_results/}
│   └── (data → 공유 마켓데이터 캐시 참조)
└── market-data/            # 신규: ccxt로 받은 Bitget OHLCV 정본 캐시 (freqtrade·dolsoe 공용)
```

- 관측 스택만 docker 상시 가동(`make up`). dolsoe·freqtrade는 host/컨테이너에서 OTLP를 `localhost:4318`로 송신(`docs/CONNECT.md` 계약).
- `OTEL_SERVICE_NAME`으로 신호 분리: `dolsoe-engine` / `dolsoe-gateway` / `freqtrade`.

---

## 5. 전략 도메인 설계 — 추세 / 횡보 / 합성

목표를 dolsoe의 `StrategyPlugin` 인터페이스 위에 얹는다.

### 5.1 횡보 전략 (Envelope 평균회귀) — "횡보장 단기로 다 먹기"
- 기존 8-tier Envelope (A-1 ±1.1% … Z-1 ±13%) + DCA + Trailing TP. **이미 라이브 핵심**.
- 책 인사이트 적용 지점: 단타 회전율(빠른 trailing/짧은 max_hold), 손실 조기 절단(derisk),
  횡보 판별 정확도(regime=RANGE에서만 활성). → 주로 **파라미터·타임프레임 튜닝** 작업.

### 5.2 추세 전략 (Donchian/ATR) — "강한 추세 다 먹기"
- 기존 `TrendStrategy(donchian_n, atr_mult, hard_stop_mult, atr_period, …)`. **집중 개발 대상**.
- 책 인사이트 핵심(추세 추종가들의 공통 원리): 돌파 진입 + 변동성 기반 stop + **이익은 길게(trailing),
  손실은 짧게** + 피라미딩(가산). → Trend 엔진에 trailing/피라미딩/하드스톱 강화.

### 5.3 합성 (StrategyRouter) — "여러 조합으로 합성"
- 현재 정적 weight Router를 **regime-조건부 라우팅**으로 진화:
  - `RANGE` → Envelope 비중↑, Trend 비중↓
  - `TRENDING_UP/DOWN` → Trend 비중↑, Envelope의 역추세 진입 차단(이미 regime block 존재)
  - `PANIC/RALLY` → 노출 축소(기존 방어 레이어 + kill switch AND-composition)
- 모든 합성 변경은 **opt-in env flag, 기본 off, kill switch AND** (dolsoe Feature Flag 정책 준수).

### 5.4 값 조정 / 시간 조절
- **값**: `run_ablation_matrix.py`(≥10셀, block-bootstrap 95% CI) — dolsoe 승격 게이트의 권위 검증.
- **시간**: `walkforward_sweep.py`로 타임프레임/룩백 walk-forward — 과최적화 방지.
- **탐색 가속**: freqtrade hyperopt로 후보 파라미터 영역을 넓게 좁힌 뒤 dolsoe로 확정 검증.

---

## 6. 책 지식 파이프라인 (Market Wizards → 가설)

```
epub  ──추출──▶  원문 텍스트  ──정제──▶  원리 카드           ──조작화──▶  검증 가능 가설
(3권)          (XHTML→md)      docs/research/external/      docs/research/internal/
                               market-wizards/<원리>.md      strategy-hypotheses/<id>.md
```

- **추출**: epub=zip(XHTML). 텍스트만 추출해 챕터별 md로 저장(`docs/research/external/market-wizards/`).
- **원리 카드**: 트레이더별/주제별로 추세·횡보·리스크 원리를 1카드=1원리로 구조화
  (출처 인용 + 우리 시장(코인 선물)에의 적용 가설).
- **가설 조작화**: 각 원리를 *검증 가능한 rule*로 변환 — 지표/임계값/타임프레임/regime 조건 명시.
  이것이 freqtrade 백테스트와 dolsoe ablation의 입력이 된다.
- 책 저작권: 원문 전체 복제·재배포 금지. 추출 텍스트는 **로컬 연구용**으로만, 카드에는 요지·인용 최소화.

---

## 7. 전략 라이프사이클 Funnel (탐색 → 운용)

dolsoe의 기존 거버넌스(CLAUDE.md)를 그대로 존중하고 freqtrade를 **1단계로만** 끼워넣는다.

| 단계 | 도구 | 산출물 | 통과 기준 |
|---|---|---|---|
| 0. 원리 | 책 | 원리 카드 | 출처·적용 가설 명시 |
| 1. 탐색 | **freqtrade** (dry-run, Bitget OHLCV) | 후보 지표/파라미터/타임프레임 | 백테스트 edge + hyperopt 수렴 (넓고 싸게) |
| 2. 가설 | docs | 방향성 가설(direction-only) | "dominates" 등 단정 금지 |
| 3. 이식 | dolsoe | gate/tier/flag (opt-in, default off, kill switch AND) | 단위 테스트 flag on/off 대칭 |
| 4. 확정 검증 | `run_backtest_matrix` + `run_ablation_matrix` | KPI delta + 95% CI | Sharpe Δ≥+0.3, maxDD ≥1σ 개선, trade수 감소 ≤50% |
| 5. 시간 견고성 | `walkforward_sweep` | walk-forward OOS | OOS에서 edge 유지 |
| 6. Paper soak | dolsoe `--paper` (관측) | ≥7일 무중단 | 관측상 이상 없음 |
| 7. 승격 | **operator-only** | architect + risk sign-off → ramp-up → live | 4-게이트 + hedge mode + reconcile pass |

- 1~6은 에이전트 수행 가능(전부 가상/검증). **7은 운영자 전용** — 에이전트는 키 주입·`--live`·reconcile 우회를 하지 않는다.

---

## 8. 데이터 레이어 (ccxt 정본화)

- **정본 거래소 = Bitget**. 연구를 Binance에서 하고 운용을 Bitget에서 하면 결과가 어긋난다 →
  freqtrade도 ccxt Bitget로 OHLCV를 받아 **dolsoe와 동일 시장**에서 연구한다.
- **공유 캐시 `market-data/`**: 한 번 받은 Bitget OHLCV를 freqtrade·dolsoe 백테스트가 공용으로 읽어
  apples-to-apples 비교 보장. dolsoe `scripts/fetch_ohlcv_fixtures.py`를 정본 fetcher로 삼고
  freqtrade는 동일 데이터 어댑터를 사용(또는 freqtrade download 후 동일 기간·심볼·타임프레임 정합).
- `ccxt enableRateLimit=True` 필수, 44심볼 일괄 fetch는 `asyncio.Semaphore`로 동시성 제한 (dolsoe 규칙).

---

## 9. 관측(Observability) 통합

### 9.1 서비스 식별
`OTEL_SERVICE_NAME`: `dolsoe-engine`, `dolsoe-gateway`, `freqtrade`. 쿼리 시 `service_name`으로 필터.

### 9.2 메트릭 택사노미 (OTLP dotted → VictoriaMetrics는 `_` 변환; PromQL은 underscore로 질의)
| 메트릭 | 타입 | 라벨 | 용도 |
|---|---|---|---|
| `dolsoe.engine.tick.duration_seconds` | histogram | mode | 틱 루프 지연 |
| `dolsoe.equity.usdt` | gauge | mode(paper/live) | 자본금 곡선 |
| `dolsoe.pnl.unrealized` / `dolsoe.pnl.realized` | gauge/counter | symbol,direction | 손익 |
| `dolsoe.positions.open` | gauge | symbol,direction,tier | 보유 포지션 |
| `dolsoe.gate.evaluations` | counter | gate,result,symbol,tier | 5중 Gate 통과/탈락 |
| `dolsoe.entries.skipped` | counter | reason(SkipReason) | 진입 생략 분포 |
| `dolsoe.orders` | counter | state(submitted/filled/failed) | 주문 실행 |
| `dolsoe.order.latency_seconds` | histogram | — | 주문 왕복 지연 |
| `dolsoe.cb.mode` | gauge(state) | — | Circuit Breaker 상태 |
| `dolsoe.reconcile.drift` | gauge | type(SIZE_DRIFT/…) | DB↔Bitget 불일치 |
| `dolsoe.regime` | gauge(state) | scope(BTC/symbol) | 현재 regime |
| `dolsoe.strategy.weight` | gauge | strategy(envelope/trend) | Router 배분 |
| `freqtrade.backtest.*`, `freqtrade.dryrun.*` | — | strategy | 연구 런 결과 |

### 9.3 로그
- dolsoe는 structlog JSON 출력 → OTLP 로그로 브리지(`event`→본문, 필드→attributes:
  `symbol/tier_id/direction/reason/gate_results`). `severity_text`(info/warn/error) 사용.
- **trace_id/span_id 주입** 필수 — 메트릭 이상 → 로그 → 트레이스로 pivot.

### 9.4 트레이스
- `_tick()`을 루트 스팬으로(틱=트랜잭션 경계). 자식 스팬: 심볼별 gate 평가 / 주문 실행.
- VictoriaTraces는 Jaeger query API(`./obs/traces.sh`)로 조회.

### 9.5 관측 루프 (에이전트가 실제로 도는 사이클)
PnL 급락(metric) → `./obs/logs.sh '_time:15m service.name:dolsoe-engine severity_text:error'` →
실패 trace_id → `./obs/correlate.sh <id>` → 어느 gate/주문이 문제였는지 → 코드/파라미터 수정 →
paper 재실행 → before/after 비교. (`AGENTS.md`의 observe→reason→change→re-run을 라이브 엔진에 적용)

### 9.6 시크릿 보호
otel-collector에 **redaction/attributes processor** 추가로 `*api_key*/secret/passphrase` 속성 제거.
관측 평면에 거래소 키가 절대 흘러들지 않게 한다.

---

## 10. 안전 경계 (실거래 = 실제 돈)

- **에이전트 ≠ 운영자**. 에이전트는: 가상/백테스트/paper만. 라이브 실행·키 주입·reconcile 우회·
  orphan adoption·CB 수동 해제 **금지** (dolsoe가 `RuntimeError`로 강제).
- **freqtrade는 영구 dry-run**: trade 권한 키를 절대 주입하지 않음(읽기 전용 데이터만). 라이브는 dolsoe 단일 경로.
- **라이브 4-게이트**(dolsoe): 키 주입 → hedge mode → reconcile pass → paper soak ≥7일 → architect+risk 승인.
- **kill switch 상시 유지**: `DOLSOE_REGIME_AWARE_KILL_SWITCH=0` minutes-grade rollback 경로 보존.
- **관측은 read-only**: ObservabilityStack은 어떤 주문도 내지 않는다.

---

## 11. 단계별 구현 로드맵

> 본 문서(설계)가 Phase 0. 이후는 사용자 승인 후 진행.

- **Phase 1 — 관측 가동 + dolsoe 계측**: `make up`; dolsoe에 OTel(logs/metrics/traces) 배선,
  §9 택사노미 구현; paper 엔진 신호가 Victoria*에 들어오는지 `./obs/*.sh`로 확인.
- **Phase 2 — freqtrade 랩**: `freqtrade-lab/` (docker image, dry-run, ccxt Bitget config),
  공유 `market-data/` 캐시, hyperopt 환경, OTLP 송신.
- **Phase 3 — 책 파이프라인**: epub 추출 → 원리 카드 → 가설 카드(§6).
- **Phase 4 — 전략 강화**: Trend 엔진(추세) 개발 + Envelope(횡보) 튜닝 + Router regime-조건부화(§5),
  각 변경은 opt-in flag + kill switch.
- **Phase 5 — 검증 funnel**: freqtrade 탐색 → dolsoe ablation/walkforward 확정 → paper soak(관측)(§7 1~6).
- **Phase 6 — (operator-only)**: 운영자 승격 절차. 에이전트 범위 밖.

---

## 12. 오픈 이슈 / 결정 필요

1. **freqtrade 배치**: 공식 docker image + user_data 권장(소스 포크 불필요). 동의 여부?
2. **dolsoe 계측 방식**: structlog→OTLP 브리지를 dolsoe 코드에 직접 넣을지(=dolsoe 커밋 발생),
   아니면 사이드카/래퍼로 비침투 구현할지. dolsoe는 커밋 컨벤션이 엄격하므로 결정 필요.
3. **마켓데이터 정본 fetcher**: dolsoe `fetch_ohlcv_fixtures.py`를 단일 정본으로 쓸지, freqtrade
   download와 어떻게 정합할지(심볼/기간/타임프레임 매핑).
4. **책 저작권 처리**: 추출 텍스트의 보관 범위(로컬 연구용 한정) 확정.
5. **타임프레임 셋**: 횡보 단타용(예 1m/5m)과 추세용(예 1h/4h) 후보 범위 합의.
6. **본 문서 위치**: 현재 ObservabilityStack/docs. dolsoe/docs/adr로 미러링(ADR화)할지.
```
