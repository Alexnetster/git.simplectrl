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
| **Plan 2 — 물리/충돌(rapier2d)** ✅완료 | 밀기 드리블·벽 반사·골 센서·누산기·리플레이 | [계획](docs/superpowers/plans/2026-07-08-physics-collision.md) |
| **Plan 3 — 전투/데미지/파츠** ⭐다음 | 상호 데미지·부위HP·파손다운·넉백/스턴·로드아웃 | (예정) |
| Plan 4 — 제어 모드/입력 | 직접(키보드)·전략(마우스)·런타임 전환 | (예정) |
| Plan 5 — 게임 흐름 | ATTRACT/SELECT/PLAYING/RESULT·슬롯 참가/인계 | (예정) |
| Plan 6 — 랭킹 | 로봇별 승률 인메모리 | (예정) |
| Plan 7 — NET SIM·재연결 | 지연/지터/드랍·heartbeat·슬롯 유예 | (예정) |
| Plan 8 — 폴리싱·README·GIF·CI | 관측성·ADR·데모 | (예정) |

---

## Backlog

**Plan 3+** — 각 Plan 착수 시 writing-plans로 카드 추가.

**Plan 3 인입 메모(Plan 2에서 넘긴 관찰)**
- **대칭 AI-vs-AI는 평형** → 공이 중앙 근처 드리프트, 라이브 골이 잘 안 남. 신뢰성 있는 득점엔 **비대칭**(사람 조작=Plan 4, 난이도/스탯 차=Plan 3) 필요. 골 로직 자체는 유닛 검증됨.
- 클라 보간(interpolation) — 아직 최신 스냅샷 렌더
- 포트/URL 설정 상수화(8090 하드코딩 2곳), 클라 재연결/try-catch
- (관찰) main publish가 프레임당 1회라 스톨 시 순간 <30Hz — 다운스트림 필요 시 보완

## Todo
_(비어 있음 — Plan 3 착수 시 채움)_

## In Progress
_(비어 있음)_

## Done
**Plan 2 — 물리/충돌(rapier2d) ✅** (branch `feat/walking-skeleton`)
- [x] KB-11 rapier2d 0.26 + 물리 월드(벽/공/로봇2)
- [x] KB-12 물리 스텝 + 골 판정·리셋
- [x] KB-13 골 입구 벽 분리 + 라이브 득점 로직
- [x] KB-14 tick→PhysicsWorld, kinematic sim 은퇴 (+ KICKOFF 단일 소스)
- [x] KB-15 고정스텝 누산기(+spiral cap) + main 배선
- [x] KB-16 골든 리플레이 + 상태 해시 (#[cfg(test)])
- [x] KB-17 검증: cargo test 11/11, WS E2E(공 물리 이동 확인). 대칭 AI라 라이브 골은 비대칭 필요

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
