export type Vec2 = { x: number; y: number };
/** 파손 다운 상태(스냅샷 디버프). repair_in = 리페어까지 남은 초. */
export type Down = { broken: boolean; repair_in: number };
export type Robot = {
  id: "Blue" | "Red";
  pos: Vec2;
  rot: number;
  /** 부위별 (부위명, HP비율 0..1). 3b부터 서버가 방출. */
  parts?: [string, number][];
  down?: Down;
  /** 상태이상 태그: "downed" | "stun" 등. */
  st?: string[];
  /** 스태미나 비율 0..1(KB-45). */
  stamina?: number;
};
export type Ball = { pos: Vec2 };
export type GameState = { robots: Robot[]; ball: Ball; score: [number, number]; time: number };

let socket: WebSocket | null = null;

export function connect(onState: (s: GameState) => void): void {
  // 127.0.0.1 고정: 이 머신에서 `localhost`는 IPv6(::1)로 다른 서비스에 갈 수 있어
  // IPv4 0.0.0.0 바인드 서버에 안 닿음. (LAN/폰은 추후 PUBLIC_URL 설정으로)
  const ws = new WebSocket("ws://127.0.0.1:8090/ws");
  socket = ws;
  ws.onopen = () => console.log("[ws] connected");
  ws.onerror = (e) => console.error("[ws] error", e);
  ws.onclose = (e) => console.warn("[ws] closed", e.code, e.reason);
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.t === "state") onState(msg.state as GameState);
  };
}

/** 업링크 송신(join/input/leave 등). 연결이 열려있지 않으면 조용히 무시. */
export function send(msg: Record<string, unknown>): void {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(msg));
  }
}
