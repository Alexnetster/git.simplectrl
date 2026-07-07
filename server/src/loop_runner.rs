use crate::control::{ChaseBallAi, Controller};
use crate::sim::step;
use crate::world::*;

/// 한 tick: 각 로봇 컨트롤러 decide → sim step. (순수, 테스트 대상)
pub fn tick(state: &mut GameState, controllers: &mut [Box<dyn Controller>]) {
    let outs: Vec<ControlOutput> = controllers
        .iter_mut()
        .enumerate()
        .map(|(i, c)| {
            let view = GameView {
                me: &state.robots[i],
                ball: &state.ball,
            };
            c.decide(&view)
        })
        .collect();
    step(state, &outs);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_advances_time_and_moves_ball_when_pushed() {
        let mut s = GameState::new_kickoff();
        let mut ctrls: Vec<Box<dyn Controller>> =
            vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)];
        let t0 = s.time;
        for _ in 0..120 {
            tick(&mut s, &mut ctrls);
        } // 2초
        assert!(s.time > t0);
        // AI가 공으로 접근 → 로봇이 중앙 쪽으로 이동했는지
        assert!(s.robots[0].pos.x > -3.0);
    }
}
