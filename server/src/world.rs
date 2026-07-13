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

/// 파손 다운 상태(스냅샷 디버프). `repair_in`=리페어까지 남은 초.
#[derive(Clone, Serialize, Default)]
pub struct Down {
    pub broken: bool,
    pub repair_in: f32,
}

// Copy 불가: `robot: String`/Vec 필드 때문에 Clone만 파생(스냅샷 클론에 충분).
#[derive(Clone, Serialize)]
pub struct RobotState {
    pub id: Team,
    pub pos: Vec2,
    pub rot: f32,
    pub vel: Vec2,
    /// 로드아웃/프리셋 id (스냅샷에 additive; 기존 필드 불변).
    pub robot: String,
    /// 부위별 (부위명, HP비율 0..1).
    pub parts: Vec<(String, f32)>,
    /// 파손 다운 상태.
    pub down: Down,
    /// 상태이상 태그(3b: 파손 다운 시 `["downed"]`, 그 외 빈 벡터).
    pub st: Vec<String>,
    /// 스태미나 비율 0..1(KB-45). 용량 없는 로봇은 항상 1.0.
    pub stamina: f32,
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
    pub turn: f32, // -1..1
    /// 달리기(Shift 홀드) 요청(KB-45). AI는 항상 false(달리기 미사용, YAGNI).
    pub run: bool,
    /// 차기(킥) 요청(KB-48, 모드리스 탭). 서버가 로봇별 이전 값과 비교해
    /// **false→true 상승엣지에서만** 1회 발사(홀드해도 반복 없음). AI는 항상 false.
    pub kick: bool,
}

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
                robot: String::new(),
                parts: Vec::new(),
                down: Down::default(),
                st: Vec::new(),
                stamina: 1.0,
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
