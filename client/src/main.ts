import { connect, type GameState } from "./net";
import { render } from "./render";

const ctx = (document.getElementById("c") as HTMLCanvasElement).getContext("2d")!;
let latest: GameState | null = null;
connect((s) => { latest = s; });
function frame() { if (latest) render(ctx, latest); requestAnimationFrame(frame); }
requestAnimationFrame(frame);
