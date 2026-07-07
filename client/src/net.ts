export type Vec2 = { x: number; y: number };
export type Robot = { id: "Blue" | "Red"; pos: Vec2; rot: number };
export type Ball = { pos: Vec2 };
export type GameState = { robots: Robot[]; ball: Ball; score: [number, number]; time: number };

export function connect(onState: (s: GameState) => void): void {
  const ws = new WebSocket("ws://localhost:8090/ws");
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.t === "state") onState(msg.state as GameState);
  };
}
