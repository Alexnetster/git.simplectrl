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
| **Plan 1 — 걷는 뼈대** ⭐진행 | 결정적 sim + Controller + WS 30Hz + canvas + 골/스코어 | [계획](docs/superpowers/plans/2026-07-03-walking-skeleton.md) |
| Plan 2 — 물리/충돌(rapier2d) | 복합 콜라이더·밀기 드리블·벽 반사 | (예정) |
| Plan 3 — 전투/데미지/파츠 | 상호 데미지·부위HP·파손다운·넉백/스턴·로드아웃 | (예정) |
| Plan 4 — 제어 모드/입력 | 직접(키보드)·전략(마우스)·런타임 전환 | (예정) |
| Plan 5 — 게임 흐름 | ATTRACT/SELECT/PLAYING/RESULT·슬롯 참가/인계 | (예정) |
| Plan 6 — 랭킹 | 로봇별 승률 인메모리 | (예정) |
| Plan 7 — NET SIM·재연결 | 지연/지터/드랍·heartbeat·슬롯 유예 | (예정) |
| Plan 8 — 폴리싱·README·GIF·CI | 관측성·ADR·데모 | (예정) |

---

## Backlog

**Plan 1 — 걷는 뼈대 (TDD 순서)**
- [ ] KB-01 프로젝트 스캐폴딩 — server(cargo)·client(vite-ts) (테스트: 빌드 성공)
- [ ] KB-02 월드 타입·상수 — Vec2/RobotState/BallState/GameState (테스트: 킥오프 상태) [의존: KB-01]
- [ ] KB-03 결정적 공 적분 — 등속+마찰 (테스트: 위치·속도) [의존: KB-02]
- [ ] KB-04 로봇 이동 — thrust/turn 적용 (테스트: 전진·회전) [의존: KB-03]
- [ ] KB-05 골 판정·스코어·리셋 (테스트: 골→스코어+킥오프) [의존: KB-04]
- [ ] KB-06 Controller 트레잇 + ChaseBall AI (테스트: 공쪽 thrust) [의존: KB-02]
- [ ] KB-07 고정 timestep tick 함수 (테스트: 시간·이동) [의존: KB-05, KB-06]
- [ ] KB-08 WebSocket 30Hz 브로드캐스트 + sim 루프 (테스트: state JSON 직렬화 / 수동: WS 수신) [의존: KB-07]
- [ ] KB-09 클라 수신·canvas 렌더 (수동: 브라우저 경기 표시) [의존: KB-08]
- [ ] KB-10 뼈대 검증·KANBAN 갱신 (전체 cargo test PASS + 수용 확인) [의존: KB-09]

**Plan 2+** — 각 Plan 착수 시 writing-plans로 카드 추가.

## Todo
_(비어 있음 — KB-01부터 착수)_

## In Progress
_(비어 있음)_

## Done
_(비어 있음)_
