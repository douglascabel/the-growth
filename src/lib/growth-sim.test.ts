import { describe, expect, it } from "vitest";
import { BAR, PED, ROBOT_BASE, ROBOT_SCALE, createSim, stepSim, solveIK } from "./growth-sim";

describe("sculpture placement", () => {
  it("retains the installation dimensions", () => {
    expect(BAR).toEqual({ L: 0.15, W: 0.05, H: 0.01 });
    expect(PED).toEqual({ S: 1.2, H: 1 });
  });
  it("seats the complete robot footprint on the pedestal", () => {
    expect(ROBOT_BASE.y - 0.003181 * ROBOT_SCALE).toBeCloseTo(PED.H, 8);
    for (const x of [-0.596768, 0.596768]) expect(Math.abs(ROBOT_BASE.x + x * ROBOT_SCALE)).toBeLessThan(PED.S / 2);
    for (const z of [-0.596273, 0.596273]) expect(Math.abs(ROBOT_BASE.z + z * ROBOT_SCALE)).toBeLessThan(PED.S / 2);
  });
  it("releases a lingot onto the pedestal rather than leaving it at the gripper", () => {
    const s = createSim();
    for (let i = 0; i < 90; i++) stepSim(s, 1 / 60);
    const bar = s.bars[0];
    expect(bar?.visible).toBe(true);
    expect(bar?.p[1]).toBeCloseTo(1.005, 8);
    expect(s.carry).toBe(false);
    expect(Object.values(solveIK(s.tool, s.toolYaw)).every(Number.isFinite)).toBe(true);
  });
});