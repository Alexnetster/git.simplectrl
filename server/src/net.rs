use crate::session::{parse_uplink, SessionId, Uplink};
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
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::{mpsc, watch};
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
type UplinkTx = mpsc::UnboundedSender<(SessionId, Uplink)>;

/// axum state: 다운링크 소스(watch rx) + 업링크 목적지(mpsc tx).
/// UnboundedSender는 Clone+Send+Sync라 Arc 불필요.
#[derive(Clone)]
struct AppState {
    watch_rx: Shared,
    uplink_tx: UplinkTx,
}

/// WS 접속마다 발급되는 세션 id 카운터. `ws_handler` 진입 시 증가.
static SESSION_COUNTER: AtomicU64 = AtomicU64::new(1);

pub async fn serve(watch_rx: Shared, uplink_tx: UplinkTx) {
    let state = AppState {
        watch_rx,
        uplink_tx,
    };
    let app = Router::new()
        .route("/ws", get(ws_handler))
        .with_state(state);
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8090")
        .await
        .unwrap();
    println!("listening on ws://localhost:8090/ws");
    axum::serve(listener, app).await.unwrap();
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    let sid = SESSION_COUNTER.fetch_add(1, Ordering::Relaxed);
    ws.on_upgrade(move |socket| handle_socket(socket, state, sid))
}

/// 다운링크(30Hz state push)와 업링크(recv→parse→mpsc)를 **한 태스크**에서
/// `select!`로 동시 처리한다. (split()/futures-util 비권장 — 드라이런 확정 사항.)
async fn handle_socket(mut socket: WebSocket, state: AppState, sid: SessionId) {
    // 접속 시 카탈로그 1회 전송(welcome 메시지는 없음) — 틱 루프 진입 전.
    let cat_json = serde_json::to_string(&catalog_msg()).unwrap();
    if socket.send(Message::Text(cat_json)).await.is_err() {
        return;
    }
    let mut tick = interval(Duration::from_millis(33)); // ~30Hz
    loop {
        tokio::select! {
            _ = tick.tick() => {
                let snapshot = state.watch_rx.borrow().clone();
                let msg = StateMsg { t: "state", state: &snapshot };
                let json = serde_json::to_string(&msg).unwrap();
                if socket.send(Message::Text(json)).await.is_err() {
                    break;
                }
            }
            msg = socket.recv() => match msg {
                Some(Ok(Message::Text(s))) => {
                    if let Some(u) = parse_uplink(&s) {
                        let _ = state.uplink_tx.send((sid, u));
                    }
                }
                Some(Ok(_)) => {}
                Some(Err(_)) | None => break,
            }
        }
    }
    // 이탈/끊김 → sim 태스크가 해당 세션의 슬롯을 AI로 복귀.
    let _ = state.uplink_tx.send((sid, Uplink::Leave));
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
