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
| **Plan 3a — 파츠/로드아웃/스탯** ✅완료 | 파츠 조립·스탯→물리·카탈로그·비대칭 프리셋 | [계획](docs/superpowers/plans/2026-07-08-parts-loadout.md) |
| **Plan 3b — 전투/데미지** ✅완료 | 부위 콜라이더·충돌 이벤트·상호 데미지·부위HP·파손다운 | [계획](docs/superpowers/plans/2026-07-08-combat-damage.md) |
| **Plan 3c — 효과 선택** ✅완료 | 넉백/스턴/데미지 effect 프로필·impact 비례 중첩 | [계획](docs/superpowers/plans/2026-07-08-combat-effects.md) |
| **Plan 4a — 사람 조작 최소 슬라이스** ⭐다음 | 슬롯 참가+키보드 uplink+사람 조종+클라 상태렌더 | [계획](docs/superpowers/plans/2026-07-08-interactivity.md) |
| Plan 4b — 전략 모드·AI 토글 | 마우스 전략·런타임 제어 전환 | (예정) |
| Plan 5 — 게임 흐름 | ATTRACT/SELECT/PLAYING/RESULT·슬롯 UI | (예정) |
| Plan 6 — 랭킹 | 로봇별 승률 인메모리 | (예정) |
| Plan 7 — NET SIM·재연결 | 지연/지터/드랍·heartbeat·슬롯 유예 | (예정) |
| Plan 8 — 폴리싱·README·GIF·CI | 관측성·ADR·데모 | (예정) |

---

## Backlog

**Plan 4a — 사람 조작 최소 슬라이스 (TDD 순서, [계획](docs/superpowers/plans/2026-07-08-interactivity.md))**
- [ ] KB-36 HumanController(최근 입력 보유, 순수) (테스트: decide=보유입력)
- [ ] KB-37 업링크 파싱 join/input/leave(순수) (테스트: 파싱·기형 무시) [의존: KB-36]
- [ ] KB-38 WS recv → mpsc, 세션 생명주기 (수동: uplink 수신) [의존: KB-37]
- [ ] KB-39 슬롯 Controller 스왑(사람↔AI)+입력 적용 (테스트: join→human, leave→AI) [의존: KB-38]
- [ ] KB-40 클라 키보드 입력 + 참가 버튼 (빌드 통과) [의존: KB-37]
- [ ] KB-41 클라 HP/스턴/다운 렌더 캐치업 (빌드 통과) [의존: KB-40]
- [ ] KB-42 E2E(사람이 로봇 조종·전투 라이브) + 문서/KANBAN [의존: KB-39,41]

> ⚠️ 착수 전 **드라이런 권장**: WS send+recv 동시(split/select)·핸들러↔sim태스크 mpsc 배선·세션→슬롯 매핑.

**Plan 4b+** — 각 Plan 착수 시 writing-plans로 카드 추가.

**남은 관찰/부채 (후속)**
- 클라 보간 — 아직 최신 스냅샷 렌더 / 포트·URL 상수화(8090×2), 클라 재연결·try-catch / main publish 프레임당 1회(스톨 시 순간 <30Hz) / 클라 vitest 미설정
- (Plan 3a Minor) `apply_controls` 중복 가드·테스트명 정확성·aggregate slot 유니크 assert — 코스메틱
- (Plan 3b Minor) 리플레이 전투 해시 테스트가 대칭AI라 데미지 없이 통과 가능(메커니즘은 강제충돌 테스트로 검증됨) / 다운 로봇도 접촉 데미지 가함(물리 장애물) / PART_NAMES↔part_count 결합 debug_assert — 전부 선택
- (Plan 3c 튜닝/후속) impact=상대속도(post-step)·**부위별 취약도(KB-34 스킵)**·다중 부위쌍 동시 데미지·**효과 가중치 본격 밸런싱**(현재 초기값만: fore-std kb 0.6 / body-std stun 0.5 / body-light dmg 0.4)

## Todo
_(비어 있음 — Plan 3b 착수 시 채움)_

## In Progress
_(비어 있음)_

## Done
**Plan 3c — 효과 선택 ✅** (branch `feat/combat-damage`)
- [x] KB-30 effect 프로필 + 임팩트 비례 중첩 선택(순수)
- [x] KB-31 스턴 타이머(순수)
- [x] KB-32 충돌 시 넉백(임펄스)/스턴(입력차단)/데미지 적용
- [x] KB-33 스냅샷 st에 "stun"
- [x] KB-34 dmg_w 가산 배선 + 카탈로그 효과값(실전 넉백/스턴 발동, 비대칭) *(부위 취약도는 스킵)*
- [x] KB-35 검증: cargo test 33/33, debug+release warning 0, 실전 프리셋 충돌 효과 확인

**Plan 3b — 전투/데미지 ✅** (branch `feat/combat-damage`)
- [x] KB-24 데미지 공식(순수)
- [x] KB-25 부위 HP + 파손다운/리페어(순수)
- [x] KB-26 부위별 복합 콜라이더 + user_data 태깅
- [x] KB-27 충돌 이벤트→상호 데미지(로봇↔로봇만, part_map 멤버십 필터)
- [x] KB-28 다운 입력 무시 + 스냅샷 디버프 필드(parts/down/st)
- [x] KB-29 검증: cargo test 27/27, debug+release warning 0, 스냅샷 디버프 필드 확인. 라이브 충돌은 비대칭 필요(대칭 AI 미접촉)

**Plan 3a — 파츠/로드아웃/스탯 ✅** (branch `feat/walking-skeleton`)
- [x] KB-18 파츠/스탯 카탈로그 + 로드아웃 집계
- [x] KB-19 물리에 로봇별 스탯 반영(accel/turn/maxSpeed/mass)
- [x] KB-20 maxSpeed 클램프
- [x] KB-21 catalog 다운링크 + 스냅샷 robot preset id
- [x] KB-22 main 비대칭 프리셋(striker/guard) + 헤드리스 검증
- [x] KB-23 검증: cargo test 17/17, 릴리스 warning 0, WS 비대칭 이동+catalog 확인

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
