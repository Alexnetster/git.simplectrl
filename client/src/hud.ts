// 크롬 HUD 배선(미드나잇 프리시전 콘솔): 서버 스냅샷의 score/time을
// 캔버스 밖 스코어보드·경기시계 DOM에 반영한다. (인캔버스 스코어/시간은 제거)
import type { GameState } from "./net";

const scoreBlueEl = document.getElementById("score-blue");
const scoreRedEl = document.getElementById("score-red");
const clockEl = document.getElementById("clock-time");

function formatClock(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  const mm = Math.floor(s / 60);
  const ss = s % 60;
  return `${String(mm).padStart(2, "0")}:${String(ss).padStart(2, "0")}`;
}

let lastBlue: number | null = null;
let lastRed: number | null = null;
let lastClock: string | null = null;

/** 매 프레임 최신 렌더 state로 호출. score[0]=Blue, score[1]=Red(서버 world.rs 규약). */
export function updateHud(s: GameState): void {
  const [blue, red] = s.score;
  if (scoreBlueEl && blue !== lastBlue) {
    scoreBlueEl.textContent = String(blue);
    lastBlue = blue;
  }
  if (scoreRedEl && red !== lastRed) {
    scoreRedEl.textContent = String(red);
    lastRed = red;
  }
  const clock = formatClock(s.time);
  if (clockEl && clock !== lastClock) {
    clockEl.textContent = clock;
    lastClock = clock;
  }
}
