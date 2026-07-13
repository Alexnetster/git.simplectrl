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
    const downed = r.st?.includes("downed") ?? false;
    const stunned = r.st?.includes("stun") ?? false;

    ctx.save(); ctx.translate(tx(r.pos.x), ty(r.pos.y)); ctx.rotate(-r.rot);
    ctx.globalAlpha = downed ? 0.4 : 1.0; // 파손 다운 시 흐리게
    ctx.fillStyle = r.id === "Blue" ? "#39f" : "#f55";
    ctx.fillRect(-15, -12, 30, 24);
    ctx.fillStyle = "#fff"; ctx.fillRect(10, -3, 8, 6); // 앞방향 표시
    ctx.restore();
    ctx.globalAlpha = 1.0;

    // HP바: 부위 중 최소 HP비율(가장 위험한 부위 기준)
    const barW = 30, barH = 4;
    if (r.parts && r.parts.length > 0) {
      const minHp = Math.min(...r.parts.map(([, hp]) => hp));
      const bx = tx(r.pos.x) - barW / 2, by = ty(r.pos.y) - 22;
      ctx.fillStyle = "#222"; ctx.fillRect(bx, by, barW, barH);
      ctx.fillStyle = minHp > 0.5 ? "#3f3" : minHp > 0.2 ? "#fa3" : "#f33";
      ctx.fillRect(bx, by, barW * Math.max(0, minHp), barH);
    }
    // 스태미나바(KB-45): HP바 바로 아래에 작게 표시.
    if (r.stamina !== undefined) {
      const sbx = tx(r.pos.x) - barW / 2, sby = ty(r.pos.y) - 17;
      ctx.fillStyle = "#222"; ctx.fillRect(sbx, sby, barW, 3);
      ctx.fillStyle = "#3cf";
      ctx.fillRect(sbx, sby, barW * Math.max(0, Math.min(1, r.stamina)), 3);
    }

    // 스턴/다운 상태 표시
    if (downed || stunned) {
      ctx.fillStyle = "#ff0";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(downed ? "DOWN" : "STUN", tx(r.pos.x), ty(r.pos.y) - 26);
      ctx.textAlign = "left";
    }
  }
  // 공
  ctx.fillStyle = "#fff"; ctx.beginPath();
  ctx.arc(tx(s.ball.pos.x), ty(s.ball.pos.y), 7, 0, Math.PI*2); ctx.fill();
  // 스코어
  ctx.fillStyle = "#fff"; ctx.font = "20px sans-serif";
  ctx.fillText(`${s.score[0]} : ${s.score[1]}`, width/2 - 20, 24);
}
