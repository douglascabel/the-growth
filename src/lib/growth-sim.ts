// Pure simulation for "The Growth": robot cycle, tower stacking, collapse.
// Units: meters, Y up. Pedestal top at y = PED_H, centered at origin.

export const BAR = { L: 0.15, W: 0.05, H: 0.01 };
export const PED = { S: 1.2, H: 1.0 };
export const ROBOT_BASE = { x: -1.9, z: 0 };
export const FEED = { x: -1.9, y: 0.82, z: 1.35 }; // top of feeder tray stack
export const ARM = { a1: 0.33, h0: 0.78, L1: 1.25, L2: 1.35, T: 0.32 };
export const MAX_BARS = 44;
export const CYCLE = 1.7; // seconds per bar
export const GRAVITY = 9.81;

export type Bar = {
  p: [number, number, number];
  v: [number, number, number];
  yaw: number; pitch: number; roll: number;
  w: [number, number, number];
  visible: boolean;
};

export type Sim = {
  t: number;
  phase: "build" | "collapse" | "rest";
  phaseT: number;
  placed: number;
  bars: Bar[];
  plan: { x: number; z: number; yaw: number }[];
  lean: { x: number; z: number };
  carry: boolean;
  tool: [number, number, number];
  toolYaw: number;
  sway: number;
};

const rand = (a: number, b: number) => a + Math.random() * (b - a);

function makePlan() {
  const dir = Math.random() * Math.PI * 2;
  let x = 0, z = 0;
  const plan = [];
  for (let i = 0; i < MAX_BARS; i++) {
    const drift = 0.0006 + i * 0.00009;
    x += Math.cos(dir) * drift + rand(-0.0015, 0.0015);
    z += Math.sin(dir) * drift + rand(-0.0015, 0.0015);
    plan.push({ x, z, yaw: (i % 2 ? Math.PI / 2 : 0) + rand(-0.06, 0.06) });
  }
  return plan;
}

export function createSim(): Sim {
  return {
    t: 0, phase: "build", phaseT: 0, placed: 0,
    bars: Array.from({ length: MAX_BARS }, () => ({
      p: [0, -10, 0], v: [0, 0, 0], yaw: 0, pitch: 0, roll: 0, w: [0, 0, 0], visible: false,
    })),
    plan: makePlan(), lean: { x: 0, z: 0 }, carry: false,
    tool: [FEED.x, FEED.y + 0.4, FEED.z], toolYaw: 0, sway: 0,
  };
}

const ss = (x: number) => { x = Math.min(1, Math.max(0, x)); return x * x * (3 - 2 * x); };
const lerp = (a: number, b: number, t: number) => a + (b - a) * t;

function stepBuild(s: Sim) {
  const i = s.placed;
  const k = s.phaseT / CYCLE;
  const target = s.plan[i]!;
  const ty = PED.H + i * BAR.H + BAR.H;
  const lift = 0.28;
  const pick: [number, number, number] = [FEED.x, FEED.y + BAR.H, FEED.z];
  const place: [number, number, number] = [target.x, ty, target.z];
  const above = (p: [number, number, number], h: number): [number, number, number] => [p[0], p[1] + h, p[2]];
  // keyframes along normalized cycle
  const keys: [number, [number, number, number], number, boolean][] = [
    [0.0, above(pick, lift), Math.PI / 2, false],
    [0.12, pick, Math.PI / 2, false],
    [0.2, pick, Math.PI / 2, true],
    [0.32, above(pick, lift + 0.15), Math.PI / 2, true],
    [0.62, above(place, lift), target.yaw, true],
    [0.76, place, target.yaw, true],
    [0.82, place, target.yaw, false],
    [1.0, above(pick, lift), Math.PI / 2, false],
  ];
  let j = 0;
  while (j < keys.length - 2 && k > keys[j + 1]![0]) j++;
  const [k0, p0, y0, c0] = keys[j]!;
  const [k1, p1, y1] = keys[j + 1]!;
  const u = ss((k - k0) / (k1 - k0));
  s.tool = [lerp(p0[0], p1[0], u), lerp(p0[1], p1[1], u), lerp(p0[2], p1[2], u)];
  s.toolYaw = lerp(y0, y1, u);
  s.carry = c0;
  const bar = s.bars[i]!;
  if (s.carry) {
    bar.visible = true;
    bar.p = [s.tool[0], s.tool[1] - BAR.H / 2, s.tool[2]];
    bar.yaw = s.toolYaw; bar.pitch = 0; bar.roll = 0;
  }
  if (s.phaseT >= CYCLE) {
    bar.p = [target.x, ty - BAR.H / 2, target.z];
    bar.yaw = target.yaw;
    s.placed++;
    s.phaseT = 0;
    if (s.placed >= MAX_BARS) { s.phase = "collapse"; s.phaseT = 0; startCollapse(s); }
  }
  s.sway = Math.max(0, s.placed - 18) * 0.00025;
}

function startCollapse(s: Sim) {
  const last = s.plan[s.placed - 1]!;
  const len = Math.hypot(last.x, last.z) || 1;
  const dx = last.x / len, dz = last.z / len;
  for (let i = 0; i < s.placed; i++) {
    const b = s.bars[i]!;
    const h = (i + 0.5) * BAR.H;
    const sp = h * 4 + rand(0, 0.15);
    b.v = [dx * sp + rand(-0.1, 0.1), rand(-0.05, 0.1), dz * sp + rand(-0.1, 0.1)];
    b.w = [rand(-3, 3) * (h * 5 + 0.2), rand(-2, 2), rand(-3, 3) * (h * 5 + 0.2)];
  }
}

function stepCollapse(s: Sim, dt: number) {
  for (let i = 0; i < s.placed; i++) {
    const b = s.bars[i]!;
    // delay lower bars a little: the tower peels from the top
    const delay = (1 - i / s.placed) * 0.25;
    if (s.phaseT < delay) continue;
    b.v[1] -= GRAVITY * dt;
    b.p[0] += b.v[0] * dt; b.p[1] += b.v[1] * dt; b.p[2] += b.v[2] * dt;
    b.yaw += b.w[1] * dt; b.pitch += b.w[0] * dt; b.roll += b.w[2] * dt;
    const onPed = Math.abs(b.p[0]) < PED.S / 2 && Math.abs(b.p[2]) < PED.S / 2;
    const floor = (onPed ? PED.H : 0) + BAR.H / 2;
    if (b.p[1] < floor && (onPed ? b.p[1] > PED.H - 0.05 : true)) {
      b.p[1] = floor;
      b.v[1] *= -0.25;
      b.v[0] *= 0.6; b.v[2] *= 0.6;
      b.w = [b.w[0] * 0.4, b.w[1] * 0.5, b.w[2] * 0.4];
      // settle flat
      const snap = (a: number) => Math.round(a / Math.PI) * Math.PI;
      b.pitch = lerp(b.pitch, snap(b.pitch), 0.35);
      b.roll = lerp(b.roll, snap(b.roll), 0.35);
    }
  }
  s.tool = [FEED.x + 0.3, FEED.y + 0.7, FEED.z * 0.5];
  if (s.phaseT > 4.5) { s.phase = "rest"; s.phaseT = 0; }
}

export function stepSim(s: Sim, dtRaw: number) {
  const dt = Math.min(dtRaw, 0.05);
  s.t += dt; s.phaseT += dt;
  if (s.phase === "build") stepBuild(s);
  else if (s.phase === "collapse") stepCollapse(s, dt);
  else if (s.phaseT > 1.5) {
    const fresh = createSim();
    Object.assign(s, fresh, { t: s.t });
  }
}

/** Analytic IK: returns joint angles for a downward-pointing tool tip at P. */
export function solveIK(P: [number, number, number]) {
  const dx = P[0] - ROBOT_BASE.x, dz = P[2] - ROBOT_BASE.z;
  const yaw = Math.atan2(-dz, dx);
  const r = Math.hypot(dx, dz) - ARM.a1;
  const h = P[1] + ARM.T - ARM.h0;
  const D = Math.min(Math.hypot(r, h), ARM.L1 + ARM.L2 - 1e-3);
  const a = Math.atan2(h, r) + Math.acos((ARM.L1 ** 2 + D ** 2 - ARM.L2 ** 2) / (2 * ARM.L1 * D));
  const b = -(Math.PI - Math.acos((ARM.L1 ** 2 + ARM.L2 ** 2 - D ** 2) / (2 * ARM.L1 * ARM.L2)));
  const c = -Math.PI / 2 - a - b;
  return { yaw, a, b, c };
}
