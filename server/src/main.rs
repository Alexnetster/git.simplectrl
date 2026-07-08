mod world;
mod control;
mod loop_runner;
mod net;
mod physics;

use control::{ChaseBallAi, Controller};
use physics::PhysicsWorld;
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{interval, Duration};
use world::GameState;

#[tokio::main]
async fn main() {
    let (tx, rx) = watch::channel(GameState::new_kickoff());

    // 물리 루프: 60Hz tick, 2 tick마다(=30Hz) 상태 발행.
    // (고정스텝 누산기 + 골든 리플레이는 다음 디스패치 KB-15/16.)
    tokio::spawn(async move {
        let mut world = PhysicsWorld::new_kickoff();
        let mut ctrls: Vec<Box<dyn Controller>> =
            vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)];
        let mut ticker = interval(Duration::from_secs_f32(world::DT));
        let mut n: u64 = 0;
        loop {
            ticker.tick().await;
            loop_runner::tick(&mut world, &mut ctrls);
            n += 1;
            if n % 2 == 0 {
                let _ = tx.send(world.snapshot());
            }
        }
    });

    net::serve(Arc::new(rx)).await;
}
