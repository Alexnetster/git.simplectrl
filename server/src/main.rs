mod accumulator;
mod world;
mod control;
mod loop_runner;
mod net;
mod parts;
mod physics;
#[cfg(test)]
mod replay;

use accumulator::Accumulator;
use control::{ChaseBallAi, Controller};
use physics::PhysicsWorld;
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{interval, Duration, Instant};
use world::GameState;

#[tokio::main]
async fn main() {
    let (tx, rx) = watch::channel(GameState::new_kickoff());

    // 물리 루프: ~120Hz 프레임을 실제 경과 시간으로 계측해 고정스텝 누산기에
    // 먹이고, 누산된 만큼 물리를 전진(고정 dt). 2스텝마다(=30Hz) 상태 발행.
    tokio::spawn(async move {
        // 비대칭 프리셋: Blue=striker(빠름), Red=guard(가속/질량↑).
        let cat = parts::catalog();
        let mut world = PhysicsWorld::new_kickoff_with(
            [
                parts::aggregate(&cat, "striker"),
                parts::aggregate(&cat, "guard"),
            ],
            ["striker".to_string(), "guard".to_string()],
        );
        let mut ctrls: Vec<Box<dyn Controller>> =
            vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)];
        let mut acc = Accumulator::new(world::DT);
        let mut ticker = interval(Duration::from_millis(8)); // ~120Hz 프레임
        let mut last = Instant::now();
        let mut since_pub: u32 = 0;
        loop {
            ticker.tick().await;
            let now = Instant::now();
            let elapsed = now.duration_since(last).as_secs_f32();
            last = now;
            let steps = acc.feed(elapsed);
            for _ in 0..steps {
                loop_runner::tick(&mut world, &mut ctrls);
                since_pub += 1;
            }
            if since_pub >= 2 {
                since_pub = 0;
                let _ = tx.send(world.snapshot()); // ~30Hz
            }
        }
    });

    net::serve(Arc::new(rx)).await;
}
