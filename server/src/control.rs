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

/// AI 슛 판정(KB-52). 공이 이 거리(m) 안(서버 사거리 0.9보다 안쪽)이고,
/// 정면 콘 안이며, 상대 골 방향으로 정렬됐을 때만 찬다(자책골 방지).
const AI_KICK_RANGE: f32 = 0.85;
const AI_KICK_FRONT_COS: f32 = 0.5; // 공이 정면 ~60° 이내
const AI_KICK_GOAL_COS: f32 = 0.55; // 상대 골 방향 ~57° 이내(정렬됐을 때만 슛)

impl ChaseBallAi {
    /// 필드 중앙(y=0) 쪽으로 도는 turn 부호. 위쪽(+y) 벽이면 CW로 내려오게.
    fn escape_turn(pos_y: f32) -> f32 {
        if pos_y >= 0.0 { -1.0 } else { 1.0 }
    }

    /// 상대 골 x좌표. 블루는 +x, 레드는 −x로 공격(physics::check_goal과 동일 규약).
    fn enemy_goal_x(team: Team) -> f32 {
        match team {
            Team::Blue => FIELD_W / 2.0,
            Team::Red => -FIELD_W / 2.0,
        }
    }

    /// 공이 정면 사거리 안 + 정면이 상대 골 방향으로 정렬 → 슛(자책골 회피).
    fn wants_kick(view: &GameView) -> bool {
        let (fx, fy) = (view.me.rot.cos(), view.me.rot.sin());
        let bdx = view.ball.pos.x - view.me.pos.x;
        let bdy = view.ball.pos.y - view.me.pos.y;
        let bdist = (bdx * bdx + bdy * bdy).sqrt();
        if bdist > AI_KICK_RANGE {
            return false;
        }
        // 공이 정면 콘 안(거리≈0이면 정면으로 간주).
        let ball_front = bdist <= 1e-6 || (fx * bdx + fy * bdy) / bdist >= AI_KICK_FRONT_COS;
        // 정면이 상대 골 방향으로 정렬(자책골 방지).
        let gdx = Self::enemy_goal_x(view.me.id) - view.me.pos.x;
        let gdy = -view.me.pos.y;
        let gdist = (gdx * gdx + gdy * gdy).sqrt();
        let goalward = gdist <= 1e-6 || (fx * gdx + fy * gdy) / gdist >= AI_KICK_GOAL_COS;
        ball_front && goalward
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
            // 슛(KB-52): 정면 사거리 안 + 상대 골 정렬 시에만. 서버가 상승엣지로 1회
            // 발사하므로, 사거리 안에서 kick=true를 유지해도 재발사는 쿨다운/엣지가 관리.
            kick: Self::wants_kick(view),
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

    fn view_at(team: Team, pos: Vec2, rot: f32, ball: Vec2) -> (RobotState, BallState) {
        (
            RobotState {
                id: team, pos, rot, vel: Vec2 { x: 0.0, y: 0.0 },
                robot: String::new(), parts: Vec::new(),
                down: Down::default(), st: Vec::new(), stamina: 1.0,
            },
            BallState { pos: ball, vel: Vec2 { x: 0.0, y: 0.0 } },
        )
    }

    /// 공이 정면 사거리 안이고 상대 골(블루=+x) 방향으로 정렬되면 슛(KB-52).
    #[test]
    fn ai_kicks_when_ball_in_front_toward_enemy_goal() {
        let (me, ball) = view_at(Team::Blue, Vec2 { x: 0.0, y: 0.0 }, 0.0, Vec2 { x: 0.6, y: 0.0 });
        let mut ai = ChaseBallAi::default();
        assert!(ai.decide(&GameView { me: &me, ball: &ball }).kick, "정렬+사거리 내 → 슛");
    }

    /// 정면이 자기 골 쪽이면(상대 골 반대) 사거리·정면이어도 안 참(자책골 방지).
    #[test]
    fn ai_does_not_kick_toward_own_goal() {
        // 블루가 −x(자기 골)를 향한 채 앞의 공을 참 → 상대 골 정렬 실패로 무슛.
        let (me, ball) = view_at(Team::Blue, Vec2 { x: 0.0, y: 0.0 }, std::f32::consts::PI, Vec2 { x: -0.6, y: 0.0 });
        let mut ai = ChaseBallAi::default();
        assert!(!ai.decide(&GameView { me: &me, ball: &ball }).kick, "자기 골 방향이면 무슛");
    }

    /// 공이 사거리 밖이면 안 참.
    #[test]
    fn ai_does_not_kick_when_ball_far() {
        let (me, ball) = view_at(Team::Blue, Vec2 { x: 0.0, y: 0.0 }, 0.0, Vec2 { x: 3.0, y: 0.0 });
        let mut ai = ChaseBallAi::default();
        assert!(!ai.decide(&GameView { me: &me, ball: &ball }).kick, "사거리 밖 무슛");
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
