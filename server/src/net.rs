use crate::world::GameState;
use serde::Serialize;

use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::Response,
    routing::get,
    Router,
};
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{interval, Duration};

#[derive(Serialize)]
pub struct StateMsg<'a> {
    pub t: &'a str,
    pub state: &'a GameState,
}

type Shared = Arc<watch::Receiver<GameState>>;

pub async fn serve(rx: Shared) {
    let app = Router::new()
        .route("/ws", get(ws_handler))
        .with_state(rx);
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8090")
        .await
        .unwrap();
    println!("listening on ws://localhost:8090/ws");
    axum::serve(listener, app).await.unwrap();
}

async fn ws_handler(ws: WebSocketUpgrade, State(rx): State<Shared>) -> Response {
    ws.on_upgrade(move |socket| push_state(socket, rx))
}

async fn push_state(mut socket: WebSocket, rx: Shared) {
    let mut tick = interval(Duration::from_millis(33)); // ~30Hz
    loop {
        tick.tick().await;
        let snapshot = rx.borrow().clone();
        let msg = StateMsg {
            t: "state",
            state: &snapshot,
        };
        let json = serde_json::to_string(&msg).unwrap();
        if socket.send(Message::Text(json)).await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::GameState;

    #[test]
    fn state_serializes_to_json_with_type_tag() {
        let s = GameState::new_kickoff();
        let msg = StateMsg {
            t: "state",
            state: &s,
        };
        let j = serde_json::to_string(&msg).unwrap();
        assert!(j.contains("\"t\":\"state\""));
        assert!(j.contains("\"score\""));
    }
}
