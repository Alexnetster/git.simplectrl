use crate::world::*;
use std::any::Any;

/// 인간/AI/스크립트 공용 인터페이스 (아키텍처 주춧돌)
pub trait Controller: Send {
    fn decide(&mut self, view: &GameView) -> ControlOutput;
    /// 슬롯 컨트롤러 스왑(AI↔사람) 시 구체 타입으로 downcast하기 위함
    /// (예: `SlotControllers`가 사람 슬롯에 최신 입력을 주입).
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

pub struct ChaseBallAi;

impl Controller for ChaseBallAi {
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn decide(&mut self, view: &GameView) -> ControlOutput {
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
        let mut ai = ChaseBallAi;
        let out = ai.decide(&GameView {
            me: &robot,
            ball: &ball,
        });
        assert!(out.thrust > 0.0); // 공쪽으로 전진
    }
}
