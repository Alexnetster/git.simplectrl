import type { GameState } from "./net";
const FIELD_W = 12, FIELD_H = 8, GOAL_W = 2.4;

export function render(ctx: CanvasRenderingContext2D, s: GameState): void {
  const { width, height } = ctx.canvas;
  const sx = width / FIELD_W, sy = height / FIELD_H;
  const tx = (x: number) => width / 2 + x * sx;
  const ty = (y: number) => height / 2 - y * sy;

  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#888"; ctx.strokeRect(0, 0, width, height);
  // 골대
  ctx.fillStyle = "#333";
  ctx.fillRect(0, ty(GOAL_W/2), 4, GOAL_W*sy);
  ctx.fillRect(width-4, ty(GOAL_W/2), 4, GOAL_W*sy);
  // 로봇
  for (const r of s.robots) {
    ctx.fillStyle = r.id === "Blue" ? "#39f" : "#f55";
    ctx.save(); ctx.translate(tx(r.pos.x), ty(r.pos.y)); ctx.rotate(-r.rot);
    ctx.fillRect(-15, -12, 30, 24);
    ctx.fillStyle = "#fff"; ctx.fillRect(10, -3, 8, 6); // 앞방향 표시
    ctx.restore();
  }
  // 공
  ctx.fillStyle = "#fff"; ctx.beginPath();
  ctx.arc(tx(s.ball.pos.x), ty(s.ball.pos.y), 7, 0, Math.PI*2); ctx.fill();
  // 스코어
  ctx.fillStyle = "#fff"; ctx.font = "20px sans-serif";
  ctx.fillText(`${s.score[0]} : ${s.score[1]}`, width/2 - 20, 24);
}
