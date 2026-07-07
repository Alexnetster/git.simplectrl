# KANBAN — 로봇 축구 1:1 데모

> 진행 방식: TDD(테스트 먼저) + 칸반 순서. 자세한 규칙은 [docs/05-개발프로세스.md](docs/05-개발프로세스.md).
> WIP 한도: **In Progress = 1** (한 번에 한 카드).

**카드 형식**
```
- [ ] KB-NN 제목 — 한 줄 설명 (테스트: 검증 내용) [의존: KB-xx]
```

**Definition of Done**: 관련 테스트 통과 + 커밋 완료 + (해당 시) 문서 갱신.

---

### 계획(에픽) 로드맵
수직 슬라이스 우선. 각 Plan은 독립 동작 소프트웨어를 낸다.

| Plan | 내용 | 문서 |
|---|---|---|
| **Plan 1 — 걷는 뼈대** ✅완료 | 결정적 sim + Controller + WS 30Hz + canvas + 골/스코어 | [계획](docs/superpowers/plans/2026-07-03-walking-skeleton.md) |
| **Plan 2 — 물리/충돌(rapier2d)** ⭐다음 | 복합 콜라이더·밀기 드리블·벽 반사 | (예정) |
| Plan 3 — 전투/데미지/파츠 | 상호 데미지·부위HP·파손다운·넉백/스턴·로드아웃 | (예정) |
| Plan 4 — 제어 모드/입력 | 직접(키보드)·전략(마우스)·런타임 전환 | (예정) |
| Plan 5 — 게임 흐름 | ATTRACT/SELECT/PLAYING/RESULT·슬롯 참가/인계 | (예정) |
| Plan 6 — 랭킹 | 로봇별 승률 인메모리 | (예정) |
| Plan 7 — NET SIM·재연결 | 지연/지터/드랍·heartbeat·슬롯 유예 | (예정) |
| Plan 8 — 폴리싱·README·GIF·CI | 관측성·ADR·데모 | (예정) |

---

## Backlog

**Plan 2+** — 각 Plan 착수 시 writing-plans로 카드 추가.

**Plan 2 인입 메모(걷는 뼈대에서 넘긴 항목)**
- 로봇↔공 충돌(밀기 드리블) → 라이브 골 성립 (현재 공 정지)
- 고정스텝 누산기(라이브 루프) + 골든 리플레이(결정성 회귀)
- sim step에서 `rot` 각도 래핑([-π,π]) — 현재 무한 누적
- 클라 보간(interpolation) — 현재 최신 스냅샷만
- 포트/URL 설정 상수화(현 8090 하드코딩 2곳)
- (선택) 클라 net.ts 재연결/try-catch, 서버·클라 상수 단일 소스

## Todo
_(비어 있음 — Plan 2 착수 시 채움)_

## In Progress
_(비어 있음)_

## Done
**Plan 1 — 걷는 뼈대 ✅** (branch `feat/walking-skeleton`)
- [x] KB-01 프로젝트 스캐폴딩 — server(cargo)·client(vite-ts)
- [x] KB-02 월드 타입·상수
- [x] KB-03 결정적 공 적분(마찰)
- [x] KB-04 로봇 이동(thrust/turn)
- [x] KB-05 골 판정·스코어·리셋
- [x] KB-06 Controller 트레잇 + ChaseBall AI
- [x] KB-07 고정 timestep tick 함수
- [x] KB-08 WebSocket 30Hz 브로드캐스트 + sim 루프 (+ 경고 정리)
- [x] KB-09 클라 수신·canvas 렌더
- [x] KB-10 검증: cargo test 8/8, WS E2E(curl로 101+state 프레임), 포트 8080→8090
