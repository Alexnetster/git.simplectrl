export type Vec2 = { x: number; y: number };
export type Robot = { id: "Blue" | "Red"; pos: Vec2; rot: number };
export type Ball = { pos: Vec2 };
export type GameState = { robots: Robot[]; ball: Ball; score: [number, number]; time: number };

let socket: WebSocket | null = null;

export function connect(onState: (s: GameState) => void): void {
  const ws = new WebSocket("ws://localhost:8090/ws");
  socket = ws;
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
