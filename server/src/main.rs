mod accumulator;
mod world;
mod combat;
mod control;
mod human;
mod loop_runner;
mod net;
mod parts;
mod physics;
#[cfg(test)]
mod replay;
mod session;

use accumulator::Accumulator;
use control::{ChaseBallAi, Controller};
use human::HumanController;
use physics::PhysicsWorld;
use session::{SessionId, Uplink};
use std::sync::Arc;
use tokio::sync::{mpsc, watch};
use tokio::time::{interval, Duration, Instant};
use world::{GameState, Team};

/// 슬롯(로봇)마다 Controller를 AI↔사람으로 스왑한다. sim 태스크가 배타 소유
/// (Mutex 불필요). `owner`는 그 슬롯을 현재 잡고 있는 세션(사람일 때만 Some).
struct SlotControllers {
    ctrls: Vec<Box<dyn Controller>>,
    owner: Vec<Option<SessionId>>,
}

impl SlotControllers {
    fn new_ai() -> Self {
        Self {
            ctrls: vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)],
            owner: vec![None, None],
        }
    }

    fn slot_index(team: Team) -> usize {
        match team {
            Team::Blue => 0,
            Team::Red => 1,
        }
    }

    fn owner_slot(&self, sid: SessionId) -> Option<usize> {
        self.owner.iter().position(|o| *o == Some(sid))
    }

    /// 테스트/향후 다운링크(누가 사람인지 표시)용 조회 헬퍼.
    #[allow(dead_code)]
    fn is_human(&self, i: usize) -> bool {
        self.owner[i].is_some()
    }

    /// join/input/leave를 슬롯 상태에 반영. 이미 다른 세션이 점유한 슬롯의
    /// join은 거부(무시) — 슬롯 경합 시 기존 점유자가 유지된다.
    fn apply(&mut self, uplink: Uplink, sid: SessionId) {
        match uplink {
            Uplink::Join(team) => {
                let i = Self::slot_index(team);
                if let Some(existing) = self.owner[i] {
                    if existing != sid {
                        return; // 슬롯 경합: 거부
                    }
                }
                // 같은 세션이 다른 슬롯을 이미 잡고 있었다면 그쪽은 AI로 되돌린다
                // (한 세션 = 최대 한 슬롯).
                if let Some(prev) = self.owner_slot(sid) {
                    if prev != i {
                        self.ctrls[prev] = Box::new(ChaseBallAi);
                        self.owner[prev] = None;
                    }
                }
                self.ctrls[i] = Box::new(HumanController::default());
                self.owner[i] = Some(sid);
            }
            Uplink::Leave => {
                if let Some(i) = self.owner_slot(sid) {
                    self.ctrls[i] = Box::new(ChaseBallAi);
                    self.owner[i] = None;
                }
            }
            Uplink::Input(input) => {
                if let Some(i) = self.owner_slot(sid) {
                    if let Some(hc) = self.ctrls[i].as_any_mut().downcast_mut::<HumanController>() {
                        hc.set(input);
                    }
                }
            }
        }
    }

    fn as_mut_slice(&mut self) -> &mut [Box<dyn Controller>] {
        &mut self.ctrls
    }
}

#[tokio::main]
async fn main() {
    let (tx, rx) = watch::channel(GameState::new_kickoff());
    let (uplink_tx, mut uplink_rx) = mpsc::unbounded_channel::<(SessionId, Uplink)>();

    // 물리 루프: ~120Hz 프레임을 실제 경과 시간으로 계측해 고정스텝 누산기에
    // 먹이고, 누산된 만큼 물리를 전진(고정 dt). 2스텝마다(=30Hz) 상태 발행.
    tokio::spawn(async move {
        // 비대칭 프리셋: Blue=striker(빠름), Red=guard(가속/질량↑).
        let cat = parts::catalog();
        let mut world = PhysicsWorld::new_kickoff_with(
            [
                parts::aggregate(&cat, "striker"),
                parts::aggregate(&cat, "guard"),
            ],
            ["striker".to_string(), "guard".to_string()],
        );
        let mut slots = SlotControllers::new_ai();
        let mut acc = Accumulator::new(world::DT);
        let mut ticker = interval(Duration::from_millis(8)); // ~120Hz 프레임
        let mut last = Instant::now();
        let mut since_pub: u32 = 0;
        loop {
            ticker.tick().await;
            // 업링크 논블로킹 드레인 → 슬롯 컨트롤러(AI↔사람) 반영.
            while let Ok((sid, u)) = uplink_rx.try_recv() {
                slots.apply(u, sid);
            }
            let now = Instant::now();
            let elapsed = now.duration_since(last).as_secs_f32();
            last = now;
            let steps = acc.feed(elapsed);
            for _ in 0..steps {
                loop_runner::tick(&mut world, slots.as_mut_slice());
                since_pub += 1;
            }
            if since_pub >= 2 {
                since_pub = 0;
                let _ = tx.send(world.snapshot()); // ~30Hz
            }
        }
    });

    net::serve(Arc::new(rx), uplink_tx).await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use world::ControlOutput;

    #[test]
    fn join_swaps_slot_to_human_leave_reverts_to_ai() {
        let mut slots = SlotControllers::new_ai();
        assert!(!slots.is_human(0));
        slots.apply(Uplink::Join(Team::Blue), 1);
        assert!(slots.is_human(0));
        slots.apply(Uplink::Leave, 1);
        assert!(!slots.is_human(0));
    }

    #[test]
    fn join_rejected_when_slot_already_taken() {
        let mut slots = SlotControllers::new_ai();
        slots.apply(Uplink::Join(Team::Blue), 1);
        assert!(slots.is_human(0));
        slots.apply(Uplink::Join(Team::Blue), 2); // 다른 세션의 경합 join
        assert_eq!(
            slots.owner[0],
            Some(1),
            "기존 점유 세션이 유지되어야 함(거부)"
        );
        // 경합 세션(2)의 입력은 이 슬롯에 적용되면 안 됨: leave(2)해도 슬롯은 그대로 사람(1) 소유.
        slots.apply(Uplink::Leave, 2);
        assert!(slots.is_human(0));
    }

    #[test]
    fn input_only_applies_to_owning_session() {
        let mut slots = SlotControllers::new_ai();
        slots.apply(Uplink::Join(Team::Blue), 1);
        slots.apply(
            Uplink::Input(ControlOutput {
                thrust: 1.0,
                turn: 0.5,
            }),
            1,
        );
        let out = slots.ctrls[0]
            .as_any_mut()
            .downcast_mut::<HumanController>()
            .unwrap();
        // decide()는 view를 쓰지 않으므로 임의 뷰로도 최근 입력을 그대로 반환.
        let robot = world::RobotState {
            id: Team::Blue,
            pos: world::Vec2 { x: 0.0, y: 0.0 },
            rot: 0.0,
            vel: world::Vec2 { x: 0.0, y: 0.0 },
            robot: String::new(),
            parts: Vec::new(),
            down: world::Down::default(),
            st: Vec::new(),
        };
        let ball = world::BallState {
            pos: world::Vec2 { x: 0.0, y: 0.0 },
            vel: world::Vec2 { x: 0.0, y: 0.0 },
        };
        let decided = out.decide(&world::GameView {
            me: &robot,
            ball: &ball,
        });
        assert_eq!(decided.thrust, 1.0);
        assert_eq!(decided.turn, 0.5);

        // 슬롯을 점유하지 않은 세션(2)의 input은 무시된다.
        slots.apply(
            Uplink::Input(ControlOutput {
                thrust: -1.0,
                turn: -1.0,
            }),
            2,
        );
        let hc = slots.ctrls[0]
            .as_any_mut()
            .downcast_mut::<HumanController>()
            .unwrap();
        let decided2 = hc.decide(&world::GameView {
            me: &robot,
            ball: &ball,
        });
        assert_eq!(decided2.thrust, 1.0, "타 세션 입력은 반영되면 안 됨");
    }
}
