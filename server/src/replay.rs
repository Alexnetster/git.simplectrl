//! 결정성 회귀(골든 리플레이) 하니스. 테스트 전용(`#[cfg(test)] mod replay`)이라
//! 릴리스 바이너리엔 포함되지 않으며, `run_headless`/`hash_state`는 테스트에서 소비된다.

use crate::control::{ChaseBallAi, Controller};
use crate::physics::PhysicsWorld;
use crate::world::GameState;

/// 결정적 상태 해시(부동소수를 비트로). 로봇 pos/rot + 공 pos + 스코어.
pub fn hash_state(s: &GameState) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for r in &s.robots {
        r.pos.x.to_bits().hash(&mut h);
        r.pos.y.to_bits().hash(&mut h);
        r.rot.to_bits().hash(&mut h);
    }
    s.ball.pos.x.to_bits().hash(&mut h);
    s.ball.pos.y.to_bits().hash(&mut h);
    s.score.hash(&mut h);
    h.finish()
}

/// 새 물리 월드 + 2 ChaseBallAi로 N 스텝 돌린 뒤 최종 스냅샷 해시. 결정적.
pub fn run_headless(steps: u32) -> u64 {
    let mut w = PhysicsWorld::new_kickoff();
    let mut c: Vec<Box<dyn Controller>> = vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)];
    for _ in 0..steps {
        crate::loop_runner::tick(&mut w, &mut c);
    }
    hash_state(&w.snapshot())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_inputs_same_hash_same_build() {
        assert_eq!(run_headless(600), run_headless(600));
    }
}
