use crate::combat::CombatState;
use crate::parts::StatSet;
use crate::world::*;
use rapier2d::prelude::*;
use std::collections::HashMap;

const WALL_T: f32 = 0.2; // 벽 두께
const BALL_R: f32 = 0.2;
const RESTITUTION: f32 = 0.85;

/// 로봇 부위 수. 부위별 자식 콜라이더(복합 바디)로 구성.
const NUM_PARTS: usize = 3;
/// 부위명(스냅샷 `parts`용). PART_SHAPES/PART_HP_WEIGHT와 index 정합.
const PART_NAMES: [&str; NUM_PARTS] = ["body", "foreleg", "hindleg"];
/// 부위 콜라이더 (반폭 hx, 반높이 hy, 로컬 오프셋 ox, oy). 앞(+x)=전진 방향.
/// 합집합 x∈[-0.25,0.25], y∈[-0.2,0.2] — 기존 단일 큐보이드(ROBOT_HX/HY) 풋프린트 근사.
const PART_SHAPES: [(f32, f32, f32, f32); NUM_PARTS] = [
    (0.12, 0.20, 0.0, 0.0),   // body(중심)
    (0.09, 0.15, 0.16, 0.0),  // foreleg(앞)
    (0.09, 0.15, -0.16, 0.0), // hindleg(뒤)
];
/// 부위별 최대 HP = 기저치 + 로봇 총 hp × 가중치. 기저치로 항상 양수(0-HP 오다운 방지).
const PART_HP_WEIGHT: [f32; NUM_PARTS] = [0.5, 0.25, 0.25];
const PART_HP_BASE: f32 = 10.0;

fn part_hps(total_hp: f32) -> Vec<f32> {
    PART_HP_WEIGHT
        .iter()
        .map(|w| PART_HP_BASE + total_hp.max(0.0) * w)
        .collect()
}

/// user_data(u128) 태깅: 상위 64비트=robot_idx, 하위=part_idx. (physics.rs 경계 전용)
fn tag(robot: usize, part: usize) -> u128 {
    ((robot as u128) << 64) | (part as u128)
}

/// 충돌 쌍이 데미지 대상인지 판정(순수). 둘 다 로봇 부위(Some)이고 **서로 다른 로봇**일 때만.
/// 벽/공(None)·같은 로봇 부위 쌍은 무데미지. 결정성 위해 (robot,part) 오름차순 정규화.
fn damage_pair(
    a: Option<(usize, usize)>,
    b: Option<(usize, usize)>,
) -> Option<((usize, usize), (usize, usize))> {
    let (a, b) = (a?, b?);
    if a.0 == b.0 {
        return None; // 같은 로봇(자기 부위끼리) → 무데미지
    }
    Some(if a <= b { (a, b) } else { (b, a) })
}

pub struct PhysicsWorld {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    gravity: Vector<Real>,
    params: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad: DefaultBroadPhase,
    narrow: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    query: QueryPipeline,
    ball: RigidBodyHandle,
    robots: Vec<RigidBodyHandle>,
    stats: Vec<StatSet>,
    preset_ids: Vec<String>,
    /// 로봇 부위 콜라이더 멤버십+디코드: handle → (robot_idx, part_idx).
    /// 벽/공은 부재 → 오데미지 방지(둘 다 멤버인 쌍만 데미지).
    part_map: HashMap<ColliderHandle, (usize, usize)>,
    /// 로봇별 부위 HP·파손 다운 상태(결정적 순수 로직 combat.rs).
    combat: Vec<CombatState>,
    pub score: (u32, u32),
    pub time: f32,
}

impl PhysicsWorld {
    /// 기본 스탯(기존 하드코딩 등가)으로 위임 — 기존 물리/골/tick 테스트 보존.
    /// 실행 바이너리는 `new_kickoff_with`(프리셋 배정)를 쓰므로 테스트 전용.
    #[cfg(test)]
    pub fn new_kickoff() -> Self {
        use crate::parts::default_stats;
        Self::new_kickoff_with(
            [default_stats(), default_stats()],
            [String::new(), String::new()],
        )
    }

    /// 로봇별 스탯/프리셋 id를 받아 킥오프 월드를 만든다.
    /// `stats[i].mass`는 콜라이더 밀도 유래 질량에 **가산**(mass=0=no-op).
    pub fn new_kickoff_with(stats: [StatSet; 2], preset_ids: [String; 2]) -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        let hw = FIELD_W / 2.0;
        let hh = FIELD_H / 2.0;

        // 상/하 벽 (고정)
        for (hx, hy, x, y) in [(hw, WALL_T, 0.0, hh), (hw, WALL_T, 0.0, -hh)] {
            colliders.insert(
                ColliderBuilder::cuboid(hx, hy)
                    .translation(vector![x, y])
                    .restitution(RESTITUTION)
                    .build(),
            );
        }

        // 좌우 벽: 골 입구(y ∈ [−GOAL_W/2, GOAL_W/2])를 비운 위/아래 두 조각
        for side in [hw, -hw] {
            let seg = (hh - GOAL_W / 2.0) / 2.0; // 각 조각 반높이
            let cy = GOAL_W / 2.0 + seg; // 조각 중심 y
            for sy in [cy, -cy] {
                colliders.insert(
                    ColliderBuilder::cuboid(WALL_T, seg)
                        .translation(vector![side, sy])
                        .restitution(RESTITUTION)
                        .build(),
                );
            }
        }

        // 공 (동적)
        let ball = bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(vector![0.0, 0.0])
                .linear_damping(0.4)
                .build(),
        );
        colliders.insert_with_parent(
            ColliderBuilder::ball(BALL_R).restitution(RESTITUTION).build(),
            ball,
            &mut bodies,
        );

        // 로봇 2대 (배치는 world::KICKOFF 단일 소스)
        // 각 로봇 = 부위별 자식 콜라이더 복합 바디. user_data 태깅 + part_map 멤버십.
        let mut robots = Vec::new();
        let mut part_map: HashMap<ColliderHandle, (usize, usize)> = HashMap::new();
        let mut combat = Vec::new();
        for (i, &(x, rot)) in KICKOFF.iter().enumerate() {
            let rb = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(vector![x, 0.0])
                    .rotation(rot)
                    .linear_damping(2.0)
                    // 회전은 apply_controls에서 set_angvel(rate 제어)로 매 스텝 덮어써
                    // angular_damping 효과는 사실상 미미 (조작감 튜닝 여지로만 유지).
                    .angular_damping(4.0)
                    // 콜라이더 밀도 유래 질량에 가산(스탯 mass; 0=no-op).
                    .additional_mass(stats[i].mass)
                    .build(),
            );
            for (p, &(hx, hy, ox, oy)) in PART_SHAPES.iter().enumerate() {
                let ch = colliders.insert_with_parent(
                    ColliderBuilder::cuboid(hx, hy)
                        .translation(vector![ox, oy])
                        .active_events(ActiveEvents::COLLISION_EVENTS)
                        .user_data(tag(i, p))
                        .build(),
                    rb,
                    &mut bodies,
                );
                part_map.insert(ch, (i, p));
            }
            combat.push(CombatState::new(&part_hps(stats[i].hp)));
            robots.push(rb);
        }

        PhysicsWorld {
            bodies,
            colliders,
            gravity: vector![0.0, 0.0],
            params: IntegrationParameters {
                dt: DT,
                ..Default::default()
            },
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad: DefaultBroadPhase::new(),
            narrow: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            query: QueryPipeline::new(),
            ball,
            robots,
            stats: stats.to_vec(),
            preset_ids: preset_ids.to_vec(),
            part_map,
            combat,
            score: (0, 0),
            time: 0.0,
        }
    }

    /// 로봇당 부위 콜라이더 수(테스트/디버그).
    #[cfg(test)]
    pub fn robot_part_count(&self) -> usize {
        NUM_PARTS
    }

    fn apply_controls(&mut self, controls: &[ControlOutput]) {
        for (i, (h, c)) in self.robots.iter().zip(controls.iter()).enumerate() {
            let st = &self.stats[i];
            let rb = &mut self.bodies[*h];
            rb.set_angvel(c.turn * st.turn_rate, true);
            let angle = rb.rotation().angle();
            let dir = vector![angle.cos(), angle.sin()];
            rb.apply_impulse(dir * (c.thrust * st.accel * DT), true);
            // maxSpeed 클램프 (impulse 적용 후)
            let v = *rb.linvel();
            let sp = (v.x * v.x + v.y * v.y).sqrt();
            if sp > st.max_speed && sp > 0.0 {
                let k = st.max_speed / sp;
                rb.set_linvel(vector![v.x * k, v.y * k], true);
            }
        }
    }

    pub fn step(&mut self, controls: &[ControlOutput]) {
        self.apply_controls(controls);
        // 충돌 이벤트 채널: collision + (미사용) contact-force 둘 다 필요(rapier 재수출).
        let (cs, cr) = rapier2d::crossbeam::channel::unbounded();
        let (fs, _fr) = rapier2d::crossbeam::channel::unbounded();
        let ev = ChannelEventCollector::new(cs, fs);
        self.pipeline.step(
            &self.gravity,
            &self.params,
            &mut self.islands,
            &mut self.broad,
            &mut self.narrow,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            Some(&mut self.query),
            &(),
            &ev,
        );
        self.apply_collision_damage(&cr);
        for c in &mut self.combat {
            c.tick_down();
        }
        self.check_goal();
        self.time += DT;
    }

    /// 로봇↔로봇 충돌 이벤트를 상호 데미지로 적용. 공/벽 접촉은 무데미지.
    /// 결정성: 한 스텝 다중 히트를 (rA,rB,pA,pB)로 정렬 후 적용(f32 비결합성 방지).
    fn apply_collision_damage(
        &mut self,
        cr: &rapier2d::crossbeam::channel::Receiver<CollisionEvent>,
    ) {
        // 1) 수집 + 필터(둘 다 로봇 부위 & 서로 다른 로봇) + 오름차순 정규화
        let mut hits: Vec<((usize, usize), (usize, usize))> = Vec::new();
        while let Ok(CollisionEvent::Started(h1, h2, _)) = cr.try_recv() {
            let a = self.part_map.get(&h1).copied();
            let b = self.part_map.get(&h2).copied();
            if let Some(pair) = damage_pair(a, b) {
                hits.push(pair);
            }
        }
        // 2) 결정성 정렬
        hits.sort_by_key(|&((ra, pa), (rb, pb))| (ra, rb, pa, pb));
        // 3) 상호 데미지 적용
        for ((ra, pa), (rb, pb)) in hits {
            // impact = 두 로봇 바디 상대 linvel 크기(ADR-009 접촉 임펄스의 의도적 간소화).
            let impact = self.relative_speed(ra, rb);
            let dmg_a = crate::combat::damage_on_contact(&self.stats[rb], &self.stats[ra], impact);
            let dmg_b = crate::combat::damage_on_contact(&self.stats[ra], &self.stats[rb], impact);
            self.combat[ra].apply_damage(pa, dmg_a);
            self.combat[rb].apply_damage(pb, dmg_b);
        }
    }

    /// 두 로봇 바디의 상대 속도 크기.
    fn relative_speed(&self, ra: usize, rb: usize) -> f32 {
        let va = *self.bodies[self.robots[ra]].linvel();
        let vb = *self.bodies[self.robots[rb]].linvel();
        let d = va - vb;
        (d.x * d.x + d.y * d.y).sqrt()
    }

    fn check_goal(&mut self) {
        let bp = *self.bodies[self.ball].translation();
        let half_w = FIELD_W / 2.0;
        let in_mouth = bp.y.abs() <= GOAL_W / 2.0;
        if bp.x > half_w && in_mouth {
            self.score.0 += 1;
            self.reset_kickoff();
        } else if bp.x < -half_w && in_mouth {
            self.score.1 += 1;
            self.reset_kickoff();
        }
    }

    fn reset_kickoff(&mut self) {
        // 공
        let b = &mut self.bodies[self.ball];
        b.set_translation(vector![0.0, 0.0], true);
        b.set_linvel(vector![0.0, 0.0], true);
        b.set_angvel(0.0, true);
        // 로봇 (배치는 world::KICKOFF 단일 소스)
        for (h, (x, rot)) in self.robots.iter().zip(KICKOFF) {
            let rb = &mut self.bodies[*h];
            rb.set_translation(vector![x, 0.0], true);
            rb.set_rotation(Rotation::new(rot), true);
            rb.set_linvel(vector![0.0, 0.0], true);
            rb.set_angvel(0.0, true);
        }
    }

    #[cfg(test)]
    pub fn kick_ball_for_test(&mut self, v: Vector<Real>) {
        self.bodies[self.ball].set_linvel(v, true);
    }

    #[cfg(test)]
    pub fn set_ball_for_test(&mut self, pos: Vector<Real>, vel: Vector<Real>) {
        let b = &mut self.bodies[self.ball];
        b.set_translation(pos, true);
        b.set_linvel(vel, true);
    }

    #[cfg(test)]
    pub fn set_robot_for_test(&mut self, i: usize, pos: Vector<Real>, rot: f32) {
        let rb = &mut self.bodies[self.robots[i]];
        rb.set_translation(pos, true);
        rb.set_rotation(Rotation::new(rot), true);
        rb.set_linvel(vector![0.0, 0.0], true);
        rb.set_angvel(0.0, true);
    }

    /// 로봇 i의 최소 부위 HP비율(테스트/디버그).
    #[cfg(test)]
    pub fn hp_ratio_min(&self, i: usize) -> f32 {
        self.combat[i].hp_ratio_min()
    }

    pub fn snapshot(&self) -> GameState {
        let b = &self.bodies[self.ball];
        let ball = BallState {
            pos: to_vec2(b.translation()),
            vel: to_vec2(b.linvel()),
        };
        let robots = self
            .robots
            .iter()
            .enumerate()
            .map(|(i, h)| {
                let rb = &self.bodies[*h];
                RobotState {
                    id: if i == 0 { Team::Blue } else { Team::Red },
                    pos: to_vec2(rb.translation()),
                    rot: rb.rotation().angle(), // rapier가 정규화된 각도 반환
                    vel: to_vec2(rb.linvel()),
                    robot: self.preset_ids[i].clone(),
                }
            })
            .collect();
        GameState {
            robots,
            ball,
            score: self.score,
            time: self.time,
        }
    }
}

fn to_vec2(v: &Vector<Real>) -> Vec2 {
    Vec2 { x: v.x, y: v.y }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_has_multiple_tagged_part_colliders() {
        let w = PhysicsWorld::new_kickoff();
        // 로봇당 부위 콜라이더 ≥2 (복합 바디)
        assert!(w.robot_part_count() >= 2, "로봇당 부위 콜라이더 ≥2");
        // part_map 멤버십 = 로봇 수 × 부위 수, 모두 유효한 (robot,part) 디코드
        assert_eq!(w.part_map.len(), 2 * w.robot_part_count());
        assert!(w.part_map.values().all(|&(r, p)| r < 2 && p < NUM_PARTS));
    }

    #[test]
    fn damage_pair_only_for_different_robots() {
        // 벽/공(None) 포함 쌍 → 무데미지 (wall-no-damage)
        assert_eq!(damage_pair(None, Some((0, 0))), None, "벽↔로봇 무데미지");
        assert_eq!(damage_pair(Some((1, 0)), None), None);
        assert_eq!(damage_pair(None, None), None);
        // 같은 로봇의 다른 부위 → 무데미지 (self-part-no-damage)
        assert_eq!(
            damage_pair(Some((0, 0)), Some((0, 1))),
            None,
            "자기 부위끼리 무데미지"
        );
        // 다른 로봇 → 데미지(오름차순 정규화 쌍)
        assert_eq!(damage_pair(Some((1, 2)), Some((0, 0))), Some(((0, 0), (1, 2))));
        assert_eq!(damage_pair(Some((0, 1)), Some((1, 0))), Some(((0, 1), (1, 0))));
    }

    #[test]
    fn robots_colliding_take_mutual_damage() {
        use crate::parts::{aggregate, catalog};
        let cat = catalog();
        let mut w = PhysicsWorld::new_kickoff_with(
            [aggregate(&cat, "striker"), aggregate(&cat, "guard")],
            ["striker".to_string(), "guard".to_string()],
        );
        // 공을 치워 로봇끼리만 충돌
        w.set_ball_for_test(vector![0.0, 3.0], vector![0.0, 0.0]);
        // 두 로봇을 마주보게 근접 배치
        w.set_robot_for_test(0, vector![-0.4, 0.0], 0.0);
        w.set_robot_for_test(1, vector![0.4, 0.0], std::f32::consts::PI);
        let before = (w.hp_ratio_min(0), w.hp_ratio_min(1));
        // 서로를 향해 돌진(robot0 +x, robot1 -x)
        let toward = [ControlOutput {
            thrust: 1.0,
            turn: 0.0,
        }; 2];
        for _ in 0..120 {
            w.step(&toward);
        }
        let after = (w.hp_ratio_min(0), w.hp_ratio_min(1));
        assert!(
            after.0 < before.0 && after.1 < before.1,
            "충돌 시 양쪽 부위 HP 감소 (before {before:?} after {after:?})"
        );
    }

    #[test]
    fn ball_contact_does_no_damage() {
        use crate::parts::{aggregate, catalog};
        let cat = catalog();
        let mut w = PhysicsWorld::new_kickoff_with(
            [aggregate(&cat, "striker"), aggregate(&cat, "striker")],
            [String::new(), String::new()],
        );
        // robot0을 공(원점) 왼쪽에 두고 돌진, robot1은 멀리(로봇충돌 배제)
        w.set_robot_for_test(0, vector![-0.6, 0.0], 0.0);
        w.set_robot_for_test(1, vector![5.0, 3.0], 0.0);
        for _ in 0..300 {
            w.step(&[
                ControlOutput {
                    thrust: 1.0,
                    turn: 0.0,
                },
                ControlOutput::default(),
            ]);
        }
        assert!(w.hp_ratio_min(0) > 0.99, "공 접촉은 무데미지");
        assert!(w.hp_ratio_min(1) > 0.99);
    }

    #[test]
    fn wall_contact_does_no_damage() {
        use crate::parts::{aggregate, catalog};
        let cat = catalog();
        let mut w = PhysicsWorld::new_kickoff_with(
            [aggregate(&cat, "striker"), aggregate(&cat, "striker")],
            [String::new(), String::new()],
        );
        w.set_ball_for_test(vector![5.0, -3.0], vector![0.0, 0.0]);
        // robot0을 상단 벽으로 돌진(rot=+PI/2), robot1은 멀리
        w.set_robot_for_test(0, vector![-3.0, 3.0], std::f32::consts::FRAC_PI_2);
        w.set_robot_for_test(1, vector![3.0, -3.0], 0.0);
        for _ in 0..200 {
            w.step(&[
                ControlOutput {
                    thrust: 1.0,
                    turn: 0.0,
                },
                ControlOutput::default(),
            ]);
        }
        assert!(w.hp_ratio_min(0) > 0.99, "벽 접촉은 무데미지");
    }

    #[test]
    fn kickoff_world_has_ball_and_two_robots_in_bounds() {
        let w = PhysicsWorld::new_kickoff();
        let s = w.snapshot();
        assert_eq!(s.robots.len(), 2);
        assert_eq!(s.ball.pos, Vec2 { x: 0.0, y: 0.0 });
        // 경계 안
        assert!(s.ball.pos.x.abs() <= FIELD_W / 2.0);
    }

    #[test]
    fn stepping_keeps_ball_in_bounds_and_advances_time() {
        let mut w = PhysicsWorld::new_kickoff();
        // 공에 강한 초기 속도
        w.kick_ball_for_test(vector![50.0, 30.0]);
        for _ in 0..600 {
            w.step(&[ControlOutput::default(); 2]);
        } // 10초
        let s = w.snapshot();
        assert!(s.time > 9.0);
        assert!(s.ball.pos.x.abs() <= FIELD_W / 2.0 + 0.5); // 벽 안(여유)
        assert!(s.ball.pos.y.abs() <= FIELD_H / 2.0 + 0.5);
    }

    #[test]
    fn ball_driven_into_right_goal_scores_blue() {
        let mut w = PhysicsWorld::new_kickoff();
        w.kick_ball_for_test(vector![40.0, 0.0]); // 오른쪽으로 강하게
        let mut scored = false;
        for _ in 0..300 {
            w.step(&[ControlOutput::default(); 2]);
            if w.score.0 == 1 {
                scored = true;
                break;
            }
        }
        assert!(scored, "공이 오른쪽 골로 들어가 Blue 득점해야 함");
        // 득점 후 공은 킥오프로 리셋
        assert!(w.snapshot().ball.pos.x.abs() < 0.1);
    }

    #[test]
    fn snapshot_carries_preset_id() {
        use crate::parts::{aggregate, catalog};
        let cat = catalog();
        let w = PhysicsWorld::new_kickoff_with(
            [aggregate(&cat, "striker"), aggregate(&cat, "guard")],
            ["striker".to_string(), "guard".to_string()],
        );
        let s = w.snapshot();
        assert_eq!(s.robots[0].robot, "striker");
        assert_eq!(s.robots[1].robot, "guard");
    }

    #[test]
    fn robot_speed_capped_by_max_speed() {
        use crate::parts::StatSet;
        let slow = StatSet {
            max_speed: 1.0,
            accel: 10.0,
            turn_rate: 1.0,
            mass: 1.0,
            ..Default::default()
        };
        let mut w =
            PhysicsWorld::new_kickoff_with([slow, slow], [String::new(), String::new()]);
        let fwd = [ControlOutput {
            thrust: 1.0,
            turn: 0.0,
        }; 2];
        for _ in 0..120 {
            w.step(&fwd);
        }
        let v = w.snapshot().robots[0].vel;
        let sp = (v.x * v.x + v.y * v.y).sqrt();
        assert!(sp <= 1.05, "속도는 max_speed 근처로 제한되어야 함 (got {sp})");
    }

    #[test]
    fn higher_accel_robot_travels_farther() {
        use crate::parts::{aggregate, catalog};
        let cat = catalog();
        // robot0=guard(accel↑), robot1=striker → 이동 거리가 달라야 함
        let mut w = PhysicsWorld::new_kickoff_with(
            [aggregate(&cat, "guard"), aggregate(&cat, "striker")],
            ["guard".to_string(), "striker".to_string()],
        );
        let fwd = [
            ControlOutput {
                thrust: 1.0,
                turn: 0.0,
            },
            ControlOutput {
                thrust: 1.0,
                turn: 0.0,
            },
        ];
        let x0 = w
            .snapshot()
            .robots
            .iter()
            .map(|r| r.pos.x)
            .collect::<Vec<_>>();
        for _ in 0..60 {
            w.step(&fwd);
        }
        let s = w.snapshot();
        let d0 = (s.robots[0].pos.x - x0[0]).abs();
        let d1 = (s.robots[1].pos.x - x0[1]).abs();
        assert!(d0 != d1, "스탯이 다르면 이동 거리가 달라야 함");
    }

    #[test]
    fn ball_exiting_outside_goal_mouth_does_not_score() {
        let mut w = PhysicsWorld::new_kickoff();
        // 골 입구 밖(|y| > GOAL_W/2)에서 오른쪽 벽 쪽으로 밀어냄
        let y = GOAL_W / 2.0 + 1.0;
        w.set_ball_for_test(vector![FIELD_W / 2.0 - 1.0, y], vector![40.0, 0.0]);
        for _ in 0..300 {
            w.step(&[ControlOutput::default(); 2]);
        }
        // 골 입구 밖이라 벽에 막혀 무득점
        assert_eq!(w.score, (0, 0), "골 입구 밖으로 나가면 무득점이어야 함");
    }
}
