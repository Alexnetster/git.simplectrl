import { connect, type GameState } from "./net";
import { render } from "./render";
import { initInput, initJoinButtons } from "./input";

const ctx = (document.getElementById("c") as HTMLCanvasElement).getContext("2d")!;
let latest: GameState | null = null;
connect((s) => { latest = s; });
initInput();
initJoinButtons("join-blue", "join-red");
function frame() { if (latest) render(ctx, latest); requestAnimationFrame(frame); }
requestAnimationFrame(frame);
