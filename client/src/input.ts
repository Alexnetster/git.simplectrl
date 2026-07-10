import { send } from "./net";

/** 현재 눌린 키 상태. (01-UX §3: ↑↓ 이동, ←→ 회전 — 이동/회전 우선, 슛 등은 후속) */
const keys = { up: false, down: false, left: false, right: false };

type InputState = { fwd: boolean; back: boolean; turn: -1 | 0 | 1 };
let lastSent: InputState | null = null;

function computeInput(): InputState {
  // turn: ←=+1(좌회전), →=-1(우회전). 둘 다 눌리면 상쇄(0).
  const turn: -1 | 0 | 1 = keys.left === keys.right ? 0 : keys.left ? 1 : -1;
  return { fwd: keys.up, back: keys.down, turn };
}

function sendIfChanged(): void {
  const cur = computeInput();
  if (
    lastSent === null ||
    cur.fwd !== lastSent.fwd ||
    cur.back !== lastSent.back ||
    cur.turn !== lastSent.turn
  ) {
    lastSent = cur;
    send({ t: "input", fwd: cur.fwd, back: cur.back, turn: cur.turn });
  }
}

function handleKey(e: KeyboardEvent, pressed: boolean): void {
  switch (e.key) {
    case "ArrowUp":
      keys.up = pressed;
      break;
    case "ArrowDown":
      keys.down = pressed;
      break;
    case "ArrowLeft":
      keys.left = pressed;
      break;
    case "ArrowRight":
      keys.right = pressed;
      break;
    default:
      return;
  }
  sendIfChanged();
}

/** 키보드 캡처 시작: keydown/keyup에서 변화 시에만 input 업링크 송신. */
export function initInput(): void {
  window.addEventListener("keydown", (e) => handleKey(e, true));
  window.addEventListener("keyup", (e) => handleKey(e, false));
}

/** [BLUE로 참가]/[RED로 참가] 버튼 → join 업링크. */
export function initJoinButtons(blueBtnId: string, redBtnId: string): void {
  document.getElementById(blueBtnId)?.addEventListener("click", () => {
    send({ t: "join", slot: "blue" });
  });
  document.getElementById(redBtnId)?.addEventListener("click", () => {
    send({ t: "join", slot: "red" });
  });
}
