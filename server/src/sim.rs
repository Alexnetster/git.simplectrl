use crate::world::*;

/// 결정적 한 스텝. controls[i]는 robots[i]에 대응.
pub fn step(s: &mut GameState, controls: &[ControlOutput]) {
    // 공: 등속 + 마찰
    s.ball.pos.x += s.ball.vel.x * DT;
    s.ball.pos.y += s.ball.vel.y * DT;
    s.ball.vel.x *= BALL_FRICTION;
    s.ball.vel.y *= BALL_FRICTION;

    let _ = controls; // 로봇 이동은 Task 3
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
}
