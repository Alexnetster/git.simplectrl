use crate::control::Controller;
use crate::physics::PhysicsWorld;
use crate::world::*;

/// 한 tick: 각 컨트롤러 decide → physics step. (결정적)
pub fn tick(world: &mut PhysicsWorld, controllers: &mut [Box<dyn Controller>]) {
    let snap = world.snapshot();
    debug_assert_eq!(
        controllers.len(),
        snap.robots.len(),
        "컨트롤러 수와 로봇 수가 일치해야 함 (controls[i] ↔ robots[i])"
    );
    let outs: Vec<ControlOutput> = controllers
        .iter_mut()
        .enumerate()
        .map(|(i, c)| {
            let view = GameView {
                me: &snap.robots[i],
                ball: &snap.ball,
            };
            c.decide(&view)
        })
        .collect();
    world.step(&outs);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::ChaseBallAi;

    #[test]
    fn tick_drives_physics_and_ball_moves_when_robot_pushes() {
        let mut w = PhysicsWorld::new_kickoff();
        let mut ctrls: Vec<Box<dyn Controller>> =
            vec![Box::new(ChaseBallAi::default()), Box::new(ChaseBallAi::default())];
        // 두 AI가 좌우 대칭이라 공을 중앙에서 맞미는 평형으로 수렴한다.
        // "밀면 움직인다"는 의도상, 스냅샷 최종값이 아니라 구동 중 임의 시점에
        // 공이 실제로 움직였는지(변위/속도)를 판정한다. (결정적)
        let mut ball_moved = false;
        for _ in 0..300 {
            tick(&mut w, &mut ctrls);
            let s = w.snapshot();
            if s.ball.pos.x.abs() > 0.05
                || s.ball.pos.y.abs() > 0.05
                || s.ball.vel.x.abs() > 0.05
                || s.ball.vel.y.abs() > 0.05
            {
                ball_moved = true;
                break;
            }
        } // 5초
        assert!(ball_moved, "AI가 공을 밀면 공이 움직여야 함");
    }
}
