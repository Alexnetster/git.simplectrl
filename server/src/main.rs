mod world;
mod sim;
mod control;
mod loop_runner;
mod net;
mod physics;

use control::{ChaseBallAi, Controller};
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{interval, Duration};
use world::GameState;

#[tokio::main]
async fn main() {
    let (tx, rx) = watch::channel(GameState::new_kickoff());

    // sim 루프: 60Hz tick, 2 tick마다(=30Hz) 상태 발행
    tokio::spawn(async move {
        let mut state = GameState::new_kickoff();
        let mut ctrls: Vec<Box<dyn Controller>> =
            vec![Box::new(ChaseBallAi), Box::new(ChaseBallAi)];
        let mut ticker = interval(Duration::from_secs_f32(world::DT));
        let mut n: u64 = 0;
        loop {
            ticker.tick().await;
            loop_runner::tick(&mut state, &mut ctrls);
            n += 1;
            if n % 2 == 0 {
                let _ = tx.send(state.clone());
            }
        }
    });

    net::serve(Arc::new(rx)).await;
}
