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

/// 파츠 스탯(다운링크용). 문서 02 §4.2대로 중첩 `stats` 객체로 낸다.
#[derive(Serialize)]
pub struct StatsDto {
    pub max_speed: f32,
    pub accel: f32,
    pub turn_rate: f32,
    pub mass: f32,
    pub kick_power: f32,
    pub attack: f32,
    pub defense: f32,
    pub hp: f32,
}

#[derive(Serialize)]
pub struct PartDto {
    pub id: String,
    pub slot: String,
    pub stats: StatsDto,
}

#[derive(Serialize)]
pub struct CatalogMsg {
    pub t: &'static str,
    pub presets: Vec<String>,
    pub parts: Vec<PartDto>,
}

/// 파츠 카탈로그를 다운링크 DTO로 변환. 순수 함수(핸들러에서 직접 호출).
/// HashMap 순회는 JSON 배열 순서에만 영향(결정성 sim 경로 아님).
pub fn catalog_msg() -> CatalogMsg {
    let cat = crate::parts::catalog();
    let parts = cat
        .parts
        .values()
        .map(|p| PartDto {
            id: p.id.to_string(),
            slot: p.slot.as_str().to_string(),
            stats: StatsDto {
                max_speed: p.stats.max_speed,
                accel: p.stats.accel,
                turn_rate: p.stats.turn_rate,
                mass: p.stats.mass,
                kick_power: p.stats.kick_power,
                attack: p.stats.attack,
                defense: p.stats.defense,
                hp: p.stats.hp,
            },
        })
        .collect();
    let presets = cat.presets.keys().map(|k| k.to_string()).collect();
    CatalogMsg {
        t: "catalog",
        presets,
        parts,
    }
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
    // 접속 시 카탈로그 1회 전송(welcome 메시지는 없음) — 틱 루프 진입 전.
    let cat_json = serde_json::to_string(&catalog_msg()).unwrap();
    if socket.send(Message::Text(cat_json)).await.is_err() {
        return;
    }
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

    #[test]
    fn catalog_msg_serializes_parts_and_presets() {
        let j = serde_json::to_string(&catalog_msg()).unwrap();
        assert!(j.contains("\"t\":\"catalog\""));
        assert!(j.contains("striker"));
    }
}
