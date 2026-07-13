import type { GameState, Robot } from "./net";
const FIELD_W = 12, FIELD_H = 8, GOAL_W = 2.4;

// ── 다리/보행(KB-47, 클라 전용 비주얼) ────────────────────────────────
// 이동 물리는 서버에서 몸체속도로 추상화돼 있고, 다리는 순수 렌더 연출이다.
// 4족=트로트(대각선 쌍 교대), 6족=삼각보행(tripod, 좌우 교대 3각).
type Chassis = "quad" | "hex";
type Leg = { x: number; y: number; ph: number }; // 로컬(+x=전방, y=측면), ph=위상오프셋
const QUAD: Leg[] = [
  { x: 6, y: 7, ph: 0 }, { x: 6, y: -7, ph: Math.PI },
  { x: -6, y: 7, ph: Math.PI }, { x: -6, y: -7, ph: 0 },
];
const HEX: Leg[] = [
  { x: 7, y: 7, ph: 0 }, { x: 7, y: -7, ph: Math.PI },
  { x: 0, y: 7, ph: Math.PI }, { x: 0, y: -7, ph: 0 },
  { x: -7, y: 7, ph: 0 }, { x: -7, y: -7, ph: Math.PI },
];

// 섀시 = 6족(거미형)으로 확정(KB-47 후속). 4족 렌더 코드(QUAD)는 향후
// 로드아웃/섀시 파츠로 개별화할 때를 위해 남겨둔다.
function chassisFor(_r: Robot): Chassis {
  return "hex";
}

// 보행 위상 상태(로봇별). rAF 프레임마다 실시간 dt × 평활 속도로 위상 전진 →
// 30Hz 스냅샷 사이에도 다리가 끊기지 않고 부드럽게 움직인다.
type Gait = { phase: number; px: number; py: number; spd: number };
const gait = new Map<string, Gait>();
let lastT = 0;

const GAIT_FREQ = 2.6; // 보행 주파수(rad / 이동 m)

function drawRobotBody(ctx: CanvasRenderingContext2D, r: Robot, phase: number): void {
  const chassis = chassisFor(r);
  const legs = chassis === "hex" ? HEX : QUAD;
  const reach = chassis === "hex" ? 13 : 11;
  const swing = 5;
  const bodyCol = r.id === "Blue" ? "#39f" : "#f55";
  const legCol = r.id === "Blue" ? "#1d5f9e" : "#a33636";

  // 다리(몸통 뒤에 먼저 그림)
  ctx.strokeStyle = legCol;
  ctx.lineWidth = 2.2;
  ctx.lineCap = "round";
  for (const l of legs) {
    const side = l.y > 0 ? 1 : -1;
    const footX = l.x + Math.sin(phase + l.ph) * swing;
    const footY = l.y + side * reach;
    const kneeX = (l.x + footX) / 2;
    const kneeY = (l.y + footY) / 2 + side * 2; // 바깥으로 살짝 꺾인 무릎
    ctx.beginPath();
    ctx.moveTo(l.x, l.y);
    ctx.lineTo(kneeX, kneeY);
    ctx.lineTo(footX, footY);
    ctx.stroke();
    ctx.fillStyle = legCol;
    ctx.beginPath(); ctx.arc(footX, footY, 1.6, 0, Math.PI * 2); ctx.fill();
  }

  // 몸통(둥근 사각) + 전방 표시
  ctx.fillStyle = bodyCol;
  const bl = 11, bw = 8; // 반길이/반폭
  if (ctx.roundRect) {
    ctx.beginPath(); ctx.roundRect(-bl, -bw, bl * 2, bw * 2, 4); ctx.fill();
  } else {
    ctx.fillRect(-bl, -bw, bl * 2, bw * 2);
  }
  ctx.fillStyle = "#fff";
  ctx.fillRect(bl - 4, -2.5, 6, 5); // 앞방향 표시(머리)
}

export function render(ctx: CanvasRenderingContext2D, s: GameState): void {
  const { width, height } = ctx.canvas;
  const sx = width / FIELD_W, sy = height / FIELD_H;
  const tx = (x: number) => width / 2 + x * sx;
  const ty = (y: number) => height / 2 - y * sy;

  // 프레임 dt(초). 탭 복귀 등 큰 점프는 클램프.
  const now = (typeof performance !== "undefined" ? performance.now() : Date.now());
  const dt = lastT === 0 ? 0 : Math.min(0.1, (now - lastT) / 1000);
  lastT = now;

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

    // 보행 위상 전진: 스냅샷 위치 변화로 순간속도 추정 → EMA 평활 → dt로 전진.
    const g = gait.get(r.id) ?? { phase: 0, px: r.pos.x, py: r.pos.y, spd: 0 };
    const d = Math.hypot(r.pos.x - g.px, r.pos.y - g.py);
    const inst = dt > 0 ? d / dt : 0;
    g.spd += (inst - g.spd) * Math.min(1, dt * 8);
    g.px = r.pos.x; g.py = r.pos.y;
    g.phase += g.spd * dt * GAIT_FREQ;
    gait.set(r.id, g);

    ctx.save();
    ctx.translate(tx(r.pos.x), ty(r.pos.y));
    ctx.rotate(-r.rot);
    // 미세 몸통 흔들림(걸을 때만): 측면으로 살짝 sway.
    ctx.translate(0, Math.sin(g.phase * 2) * Math.min(1, g.spd) * 1.2);
    ctx.globalAlpha = downed ? 0.4 : 1.0; // 파손 다운 시 흐리게
    drawRobotBody(ctx, r, g.phase);
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
