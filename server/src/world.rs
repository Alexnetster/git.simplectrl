use serde::Serialize;

pub const FIELD_W: f32 = 12.0; // meters
pub const FIELD_H: f32 = 8.0;
pub const GOAL_W: f32 = 2.4;
pub const DT: f32 = 1.0 / 60.0; // fixed timestep

/// 킥오프 로봇 배치 (x, rot) — index 0 = Blue, 1 = Red. 단일 소스.
/// physics(new_kickoff/reset_kickoff)와 GameState::new_kickoff가 공유한다.
pub const KICKOFF: [(f32, f32); 2] = [(-3.0, 0.0), (3.0, std::f32::consts::PI)];

#[derive(Clone, Copy, PartialEq, Debug, Serialize)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Copy, Serialize)]
pub struct RobotState {
    pub id: Team,
    pub pos: Vec2,
    pub rot: f32,
    pub vel: Vec2,
}

#[derive(Clone, Copy, Serialize)]
pub struct BallState {
    pub pos: Vec2,
    pub vel: Vec2,
}

#[derive(Clone, Copy, PartialEq, Debug, Serialize)]
pub enum Team {
    Blue,
    Red,
}

#[derive(Clone, Serialize)]
pub struct GameState {
    pub robots: Vec<RobotState>,
    pub ball: BallState,
    pub score: (u32, u32),
    pub time: f32,
}

/// 컨트롤러가 보는 읽기 전용 뷰
pub struct GameView<'a> {
    pub me: &'a RobotState,
    pub ball: &'a BallState,
}

/// 컨트롤러가 내는 명령(액추에이터 층)
#[derive(Clone, Copy, Default)]
pub struct ControlOutput {
    pub thrust: f32,
    pub turn: f32,
} // -1..1

impl GameState {
    pub fn new_kickoff() -> Self {
        let robots = KICKOFF
            .iter()
            .enumerate()
            .map(|(i, &(x, rot))| RobotState {
                id: if i == 0 { Team::Blue } else { Team::Red },
                pos: Vec2 { x, y: 0.0 },
                rot,
                vel: Vec2 { x: 0.0, y: 0.0 },
            })
            .collect();
        GameState {
            robots,
            ball: BallState {
                pos: Vec2 { x: 0.0, y: 0.0 },
                vel: Vec2 { x: 0.0, y: 0.0 },
            },
            score: (0, 0),
            time: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_state_has_two_robots_and_centered_ball() {
        let s = GameState::new_kickoff();
        assert_eq!(s.robots.len(), 2);
        assert_eq!(s.ball.pos, Vec2 { x: 0.0, y: 0.0 });
        assert_eq!(s.score, (0, 0));
    }
}
