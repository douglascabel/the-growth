import { Canvas, useFrame } from "@react-three/fiber";
import { ContactShadows, Environment, Lightformer, OrbitControls, RoundedBox } from "@react-three/drei";
import { useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { ARM, BAR, FEED, MAX_BARS, PED, ROBOT_BASE, createSim, solveIK, stepSim } from "@/lib/growth-sim";

const KUKA_ORANGE = "#f26a10";

function useConcreteTexture() {
  return useMemo(() => {
    const c = document.createElement("canvas");
    c.width = c.height = 512;
    const g = c.getContext("2d")!;
    g.fillStyle = "#8c8a86";
    g.fillRect(0, 0, 512, 512);
    for (let i = 0; i < 18000; i++) {
      const v = 110 + Math.random() * 60;
      g.fillStyle = `rgba(${v},${v - 2},${v - 5},${Math.random() * 0.25})`;
      g.fillRect(Math.random() * 512, Math.random() * 512, 1 + Math.random() * 3, 1 + Math.random() * 3);
    }
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(8, 8);
    t.colorSpace = THREE.SRGBColorSpace;
    return t;
  }, []);
}

const gold = new THREE.MeshPhysicalMaterial({ color: "#e8b54a", metalness: 1, roughness: 0.18, clearcoat: 0.3 });
const orange = new THREE.MeshPhysicalMaterial({ color: KUKA_ORANGE, metalness: 0.15, roughness: 0.38, clearcoat: 0.6, clearcoatRoughness: 0.25 });
const dark = new THREE.MeshStandardMaterial({ color: "#222326", metalness: 0.6, roughness: 0.45 });
const steel = new THREE.MeshStandardMaterial({ color: "#b9bcc0", metalness: 1, roughness: 0.25 });

function Robot({ sim }: { sim: ReturnType<typeof createSim> }) {
  const base = useRef<THREE.Group>(null!);
  const sh = useRef<THREE.Group>(null!);
  const el = useRef<THREE.Group>(null!);
  const wr = useRef<THREE.Group>(null!);
  const roll = useRef<THREE.Group>(null!);
  const fingers = useRef<THREE.Group[]>([]);
  useFrame(() => {
    const ik = solveIK(sim.tool);
    base.current.rotation.y = ik.yaw;
    sh.current.rotation.z = ik.a;
    el.current.rotation.z = ik.b;
    wr.current.rotation.z = ik.c;
    roll.current.rotation.x = -(sim.toolYaw - ik.yaw);
    const open = sim.carry ? 0.028 : 0.05;
    fingers.current.forEach((f, i) => f && (f.position.z = (i ? 1 : -1) * open));
  });
  return (
    <group position={[ROBOT_BASE.x, 0, ROBOT_BASE.z]}>
      {/* foot */}
      <mesh material={dark} castShadow receiveShadow position={[0, 0.06, 0]}>
        <boxGeometry args={[0.95, 0.12, 0.95]} />
      </mesh>
      <mesh material={orange} castShadow position={[0, 0.2, 0]}>
        <cylinderGeometry args={[0.38, 0.44, 0.18, 48]} />
      </mesh>
      <group ref={base} position={[0, 0.29, 0]}>
        {/* rotating column */}
        <mesh material={orange} castShadow position={[0.1, 0.22, 0]}>
          <cylinderGeometry args={[0.34, 0.36, 0.44, 48]} />
        </mesh>
        <RoundedBox args={[0.55, 0.42, 0.62]} radius={0.06} material={orange} castShadow position={[ARM.a1 - 0.08, ARM.h0 - 0.29 - 0.1, 0]} />
        {/* counterbalance */}
        <mesh material={dark} castShadow position={[-0.22, 0.55, 0.36]} rotation={[0, 0, 0.5]}>
          <cylinderGeometry args={[0.07, 0.07, 0.65, 24]} />
        </mesh>
        <group ref={sh} position={[ARM.a1, ARM.h0 - 0.29, 0]}>
          <mesh material={dark} castShadow rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.24, 0.24, 0.72, 48]} />
          </mesh>
          <RoundedBox args={[ARM.L1, 0.3, 0.34]} radius={0.08} material={orange} castShadow position={[ARM.L1 / 2, 0, 0.12]} />
          <group ref={el} position={[ARM.L1, 0, 0]}>
            <mesh material={orange} castShadow rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[0.22, 0.22, 0.56, 48]} />
            </mesh>
            <RoundedBox args={[0.5, 0.36, 0.42]} radius={0.08} material={orange} castShadow position={[-0.12, 0.06, -0.08]} />
            <mesh material={orange} castShadow position={[ARM.L2 / 2 + 0.05, 0, -0.08]} rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.1, 0.15, ARM.L2 - 0.2, 32]} />
            </mesh>
            <mesh material={dark} castShadow position={[-0.25, 0.06, -0.08]} rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.13, 0.13, 0.2, 32]} />
            </mesh>
            <group ref={wr} position={[ARM.L2, 0, -0.08]}>
              <mesh material={orange} castShadow rotation={[Math.PI / 2, 0, 0]}>
                <cylinderGeometry args={[0.09, 0.09, 0.22, 32]} />
              </mesh>
              <group ref={roll}>
                <mesh material={dark} castShadow position={[0.1, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
                  <cylinderGeometry args={[0.06, 0.06, 0.08, 32]} />
                </mesh>
                <mesh material={steel} castShadow position={[0.2, 0, 0]}>
                  <boxGeometry args={[0.07, 0.07, 0.16]} />
                </mesh>
                {[0, 1].map((i) => (
                  <group key={i} ref={(g) => { if (g) fingers.current[i] = g; }} position={[0.27, 0, 0]}>
                    <mesh material={steel} castShadow>
                      <boxGeometry args={[0.09, 0.05, 0.012]} />
                    </mesh>
                  </group>
                ))}
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

function Bars({ sim }: { sim: ReturnType<typeof createSim> }) {
  const refs = useRef<THREE.Mesh[]>([]);
  const tower = useRef<THREE.Group>(null!);
  useFrame(() => {
    const sway = sim.phase === "build" ? sim.sway : 0;
    tower.current.rotation.x = Math.sin(sim.t * 1.3) * sway;
    tower.current.rotation.z = Math.cos(sim.t * 1.1) * sway;
    sim.bars.forEach((b, i) => {
      const m = refs.current[i];
      if (!m) return;
      m.visible = b.visible;
      m.position.set(b.p[0], b.p[1] - PED.H, b.p[2]);
      m.rotation.set(b.pitch, b.yaw, b.roll);
    });
  });
  return (
    <group ref={tower} position={[0, PED.H, 0]}>
      {Array.from({ length: MAX_BARS }, (_, i) => (
        <mesh key={i} ref={(m) => { if (m) refs.current[i] = m; }} material={gold} castShadow receiveShadow visible={false}>
          <boxGeometry args={[BAR.L, BAR.H, BAR.W]} />
        </mesh>
      ))}
    </group>
  );
}

function Stepper({ sim, onReady }: { sim: ReturnType<typeof createSim>; onReady: () => void }) {
  const n = useRef(0);
  useFrame((_, d) => {
    stepSim(sim, d);
    if (++n.current === 3) onReady();
  });
  return null;
}

function World() {
  const floorTex = useConcreteTexture();
  const sim = useMemo(() => createSim(), []);
  const [, setReady] = useState(false);
  return (
    <>
      <Stepper sim={sim} onReady={() => { document.querySelector("#growth")?.setAttribute("data-scene-ready", "1"); setReady(true); }} />
      <color attach="background" args={["#d9d6d0"]} />
      <fog attach="fog" args={["#d9d6d0", 9, 22]} />
      <ambientLight intensity={0.15} />
      <directionalLight
        position={[3, 6, 2.5]} intensity={2.2} castShadow
        shadow-mapSize={[2048, 2048]} shadow-bias={-0.0002} shadow-normalBias={0.02}
        shadow-camera-left={-4} shadow-camera-right={4} shadow-camera-top={4} shadow-camera-bottom={-4}
      />
      <Environment resolution={256}>
        <Lightformer intensity={2.5} position={[0, 6, 0]} rotation-x={Math.PI / 2} scale={[10, 6, 1]} />
        <Lightformer intensity={1.2} color="#fff3e0" position={[-6, 2, 2]} rotation-y={Math.PI / 2} scale={[12, 3, 1]} />
        <Lightformer intensity={0.8} color="#dfe8ff" position={[6, 2, -2]} rotation-y={-Math.PI / 2} scale={[12, 3, 1]} />
        <Lightformer intensity={0.6} position={[0, 1, 8]} scale={[10, 2, 1]} />
      </Environment>

      {/* floor */}
      <mesh rotation-x={-Math.PI / 2} receiveShadow>
        <planeGeometry args={[40, 40]} />
        <meshStandardMaterial map={floorTex} roughness={0.85} metalness={0} />
      </mesh>
      <ContactShadows position={[0, 0.002, 0]} scale={10} blur={2.5} opacity={0.5} far={2} />

      {/* pedestal */}
      <RoundedBox args={[PED.S, PED.H, PED.S]} radius={0.005} position={[0, PED.H / 2, 0]} castShadow receiveShadow>
        <meshPhysicalMaterial color="#f4f2ee" roughness={0.55} clearcoat={0.1} />
      </RoundedBox>

      {/* feeder table with a stock of bars */}
      <group position={[FEED.x, 0, FEED.z]}>
        <mesh material={dark} castShadow receiveShadow position={[0, (FEED.y - 0.06) / 2, 0]}>
          <boxGeometry args={[0.45, FEED.y - 0.06, 0.35]} />
        </mesh>
        {Array.from({ length: 5 }, (_, i) => (
          <mesh key={i} material={gold} castShadow position={[0, FEED.y - 0.06 + (i + 0.5) * 0.012, 0]} rotation-y={Math.PI / 2}>
            <boxGeometry args={[BAR.L, BAR.H, BAR.W]} />
          </mesh>
        ))}
      </group>

      <Robot sim={sim} />
      <Bars sim={sim} />

      <OrbitControls
        target={[-0.6, 1.0, 0.2]} enableDamping dampingFactor={0.08}
        minDistance={0.6} maxDistance={12} maxPolarAngle={Math.PI / 2 - 0.03}
        autoRotate autoRotateSpeed={0.35}
      />
    </>
  );
}

export default function GrowthScene() {
  return (
    <div id="growth" className="h-full w-full">
      <Canvas
        shadows
        dpr={[1, 2]}
        camera={{ position: [3.2, 2.2, 4.2], fov: 38, near: 0.05, far: 60 }}
        gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.05 }}
      >
        <World />
      </Canvas>
    </div>
  );
}
