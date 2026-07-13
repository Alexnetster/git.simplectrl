use crate::world::*;
use std::any::Any;

/// 인간/AI/스크립트 공용 인터페이스 (아키텍처 주춧돌)
pub trait Controller: Send {
    fn decide(&mut self, view: &GameView) -> ControlOutput;
    /// 슬롯 컨트롤러 스왑(AI↔사람) 시 구체 타입으로 downcast하기 위함
    /// (예: `SlotControllers`가 사람 슬롯에 최신 입력을 주입).
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// 공을 향해 전진하는 기본 AI. 벽/펜스/코너에 박혀 정지하면(공을 계속 밀어도
/// 속도≈0) 스턱으로 판정하고 잠깐 후진+회전으로 빠져나온다(KB-49).
#[derive(Default)]
pub struct ChaseBallAi {
    /// 정지(속도<STUCK_SPEED) 지속 프레임 수.
    stuck: u32,
    /// 남은 탈출 기동 프레임(>0이면 후진+회전).
    escape: u32,
}

/// 스턱 판정 속도 임계(m/s). 이 미만을 정지로 본다.
const STUCK_SPEED: f32 = 0.25;
/// 정지가 이만큼(프레임=~1초 @60Hz) 지속되면 스턱으로 보고 탈출 시작.
const STUCK_LIMIT: u32 = 60;
/// 탈출 기동 지속(프레임=~0.66초).
const ESCAPE_FRAMES: u32 = 40;

impl ChaseBallAi {
    /// 필드 중앙(y=0) 쪽으로 도는 turn 부호. 위쪽(+y) 벽이면 CW로 내려오게.
    fn escape_turn(pos_y: f32) -> f32 {
        if pos_y >= 0.0 { -1.0 } else { 1.0 }
    }

    /// 후진하며 중앙 쪽으로 회전(벽에서 멀어짐).
    fn escape_output(pos_y: f32) -> ControlOutput {
        ControlOutput {
            thrust: -1.0,
            turn: Self::escape_turn(pos_y),
            run: false,
            kick: false,
        }
    }
}

impl Controller for ChaseBallAi {
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn decide(&mut self, view: &GameView) -> ControlOutput {
        // 탈출 기동 진행 중: 끝날 때까지 후진+회전.
        if self.escape > 0 {
            self.escape -= 1;
            self.stuck = 0;
            return Self::escape_output(view.me.pos.y);
        }
        // 정지 지속 추적(공을 밀어도 벽에 막혀 속도가 안 나는 상태).
        let speed = (view.me.vel.x * view.me.vel.x + view.me.vel.y * view.me.vel.y).sqrt();
        if speed < STUCK_SPEED {
            self.stuck += 1;
        } else {
            self.stuck = 0;
        }
        if self.stuck >= STUCK_LIMIT {
            self.stuck = 0;
            self.escape = ESCAPE_FRAMES;
            return Self::escape_output(view.me.pos.y);
        }

        // 평소: 공을 향해 전진.
        let dx = view.ball.pos.x - view.me.pos.x;
        let dy = view.ball.pos.y - view.me.pos.y;
        let target = dy.atan2(dx);
        let mut diff = target - view.me.rot;
        while diff > std::f32::consts::PI {
            diff -= std::f32::consts::TAU;
        }
        while diff < -std::f32::consts::PI {
            diff += std::f32::consts::TAU;
        }
        ControlOutput {
            thrust: 1.0,
            turn: diff.clamp(-1.0, 1.0),
            // AI는 달리기를 쓰지 않는다(KB-45 YAGNI: AI sprint 없음).
            run: false,
            // AI는 차기를 쓰지 않는다(KB-48 YAGNI: AI 킥 없음).
            kick: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chaseball_thrusts_forward() {
        let robot = RobotState {
            id: Team::Blue,
            pos: Vec2 { x: 0.0, y: 0.0 },
            rot: 0.0,
            vel: Vec2 { x: 0.0, y: 0.0 },
            robot: String::new(),
            parts: Vec::new(),
            down: Down::default(),
            st: Vec::new(),
            stamina: 1.0,
        };
        let ball = BallState {
            pos: Vec2 { x: 5.0, y: 0.0 },
            vel: Vec2 { x: 0.0, y: 0.0 },
        };
        let mut ai = ChaseBallAi::default();
        let out = ai.decide(&GameView {
            me: &robot,
            ball: &ball,
        });
        assert!(out.thrust > 0.0); // 공쪽으로 전진
    }

    /// 벽에 박혀 정지(속도≈0)가 오래 지속되면 후진 탈출 기동으로 전환해야 한다(KB-49).
    #[test]
    fn stuck_against_wall_triggers_reverse_escape() {
        // 위쪽(+y) 벽 근처에서 정지, 공은 벽 너머(도달 불가) 방향.
        let robot = RobotState {
            id: Team::Blue,
            pos: Vec2 { x: 5.5, y: 3.5 },
            rot: 0.0,
            vel: Vec2 { x: 0.0, y: 0.0 }, // 정지
            robot: String::new(),
            parts: Vec::new(),
            down: Down::default(),
            st: Vec::new(),
            stamina: 1.0,
        };
        let ball = BallState {
            pos: Vec2 { x: 6.5, y: 4.5 },
            vel: Vec2 { x: 0.0, y: 0.0 },
        };
        let view = GameView { me: &robot, ball: &ball };
        let mut ai = ChaseBallAi::default();
        // 처음에는 전진(공쪽).
        assert!(ai.decide(&view).thrust > 0.0);
        // 정지가 STUCK_LIMIT 넘게 지속되면 후진 탈출로 전환.
        let mut escaped = false;
        for _ in 0..(STUCK_LIMIT + ESCAPE_FRAMES + 5) {
            if ai.decide(&view).thrust < 0.0 {
                escaped = true;
                break;
            }
        }
        assert!(escaped, "정지 지속 시 후진 탈출 기동이 나와야 함");
    }

    /// 정상 주행(속도 충분)에서는 스턱 판정이 되지 않아야 한다(오탐 방지).
    #[test]
    fn moving_normally_never_escapes() {
        let robot = RobotState {
            id: Team::Blue,
            pos: Vec2 { x: 0.0, y: 0.0 },
            rot: 0.0,
            vel: Vec2 { x: 3.0, y: 0.0 }, // 충분한 속도
            robot: String::new(),
            parts: Vec::new(),
            down: Down::default(),
            st: Vec::new(),
            stamina: 1.0,
        };
        let ball = BallState {
            pos: Vec2 { x: 5.0, y: 0.0 },
            vel: Vec2 { x: 0.0, y: 0.0 },
        };
        let view = GameView { me: &robot, ball: &ball };
        let mut ai = ChaseBallAi::default();
        for _ in 0..(STUCK_LIMIT * 3) {
            assert!(ai.decide(&view).thrust > 0.0, "정상 주행 중엔 항상 전진");
        }
    }
}
