// 개발용 넷코드 패널(Plan 7a): netsim(delay/jitter/drop) 조절, 보간 on/off,
// RTT 표시. index.html의 #dev-panel 마크업과 짝을 이룬다.
import { sendNetsim, sendPing, getRtt } from "./net";
import { setInterpEnabled } from "./interp";

const PING_INTERVAL_MS = 1000;
const RTT_REFRESH_MS = 250;

export function initDevPanel(): void {
  const delayInput = document.getElementById("netsim-delay") as HTMLInputElement | null;
  const jitterInput = document.getElementById("netsim-jitter") as HTMLInputElement | null;
  const dropInput = document.getElementById("netsim-drop") as HTMLInputElement | null;
  const interpToggle = document.getElementById("interp-toggle") as HTMLInputElement | null;
  const rttEl = document.getElementById("rtt-value");

  function sendCurrentNetsim(): void {
    const delay_ms = Number(delayInput?.value) || 0;
    const jitter_ms = Number(jitterInput?.value) || 0;
    const drop_pct = Number(dropInput?.value) || 0;
    sendNetsim(delay_ms, jitter_ms, drop_pct);
  }

  delayInput?.addEventListener("change", sendCurrentNetsim);
  jitterInput?.addEventListener("change", sendCurrentNetsim);
  dropInput?.addEventListener("change", sendCurrentNetsim);

  if (interpToggle) {
    setInterpEnabled(interpToggle.checked);
    interpToggle.addEventListener("change", () => setInterpEnabled(interpToggle.checked));
  }

  setInterval(() => sendPing(), PING_INTERVAL_MS);
  setInterval(() => {
    if (!rttEl) return;
    const rtt = getRtt();
    rttEl.textContent = rtt === null ? "RTT: -- ms" : `RTT: ${Math.round(rtt)} ms`;
  }, RTT_REFRESH_MS);
}
