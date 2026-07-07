use crate::world::*;

/// 결정적 한 스텝. controls[i]는 robots[i]에 대응.
pub fn step(s: &mut GameState, controls: &[ControlOutput]) {
    // 공: 등속 + 마찰
    s.ball.pos.x += s.ball.vel.x * DT;
    s.ball.pos.y += s.ball.vel.y * DT;
    s.ball.vel.x *= BALL_FRICTION;
    s.ball.vel.y *= BALL_FRICTION;

    const ACCEL: f32 = 8.0;
    const TURN_RATE: f32 = 3.0;
    for (r, c) in s.robots.iter_mut().zip(controls.iter()) {
        r.rot += c.turn * TURN_RATE * DT;
        let (dx, dy) = (r.rot.cos(), r.rot.sin());
        r.vel.x += dx * c.thrust * ACCEL * DT;
        r.vel.y += dy * c.thrust * ACCEL * DT;
        r.vel.x *= 0.9;
        r.vel.y *= 0.9; // 감쇠
        r.pos.x += r.vel.x * DT;
        r.pos.y += r.vel.y * DT;
    }
    s.time += DT;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ball_moves_by_velocity_and_slows_by_friction() {
        let mut s = GameState::new_kickoff();
        s.ball.vel = Vec2 { x: 1.0, y: 0.0 };
        step(&mut s, &[ControlOutput::default(), ControlOutput::default()]);
        // 위치는 vel*dt 만큼 이동
        assert!((s.ball.pos.x - (1.0 * DT)).abs() < 1e-6);
        // 속도는 마찰로 감소
        assert!(s.ball.vel.x < 1.0 && s.ball.vel.x > 0.0);
    }

    #[test]
    fn robot_thrust_moves_it_forward_along_rotation() {
        let mut s = GameState::new_kickoff();
        s.robots[0].rot = 0.0; // +x 방향
        let ctrls = [
            ControlOutput {
                thrust: 1.0,
                turn: 0.0,
            },
            ControlOutput::default(),
        ];
        step(&mut s, &ctrls);
        assert!(s.robots[0].pos.x > -3.0); // 앞으로 이동
    }

    #[test]
    fn robot_turn_changes_rotation() {
        let mut s = GameState::new_kickoff();
        let ctrls = [
            ControlOutput {
                thrust: 0.0,
                turn: 1.0,
            },
            ControlOutput::default(),
        ];
        let before = s.robots[0].rot;
        step(&mut s, &ctrls);
        assert!(s.robots[0].rot != before);
    }
}
