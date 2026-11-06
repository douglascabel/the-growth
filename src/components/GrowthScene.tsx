import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import robotObj from "@/assets/industrial_robot_arm.obj.asset.json";
import colorTex from "@/assets/robot_arm_color.jpg.asset.json";
import normalTex from "@/assets/robot_arm_normal.jpg.asset.json";
import roughTex from "@/assets/robot_arm_roughness.jpg.asset.json";
import { ContactShadows, Environment, useTexture, Lightformer, OrbitControls, RoundedBox } from "@react-three/drei";
import { Suspense, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { RIG, BAR, FEED, MAX_BARS, PED, ROBOT_BASE, createSim, solveIK, stepSim } from "@/lib/growth-sim";


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
const dark = new THREE.MeshStandardMaterial({ color: "#222326", metalness: 0.6, roughness: 0.45 });

function Robot({ sim }: { sim: ReturnType<typeof createSim> }) {
  const obj = useLoader(OBJLoader, robotObj.url);
  const [map, normalMap, roughnessMap] = useTexture([colorTex.url, normalTex.url, roughTex.url]);
  const parts = useMemo(() => {
    map.colorSpace = THREE.SRGBColorSpace;
    for (const t of [map, normalMap, roughnessMap]) t.anisotropy = 8;
    const mat = new THREE.MeshPhysicalMaterial({
      map, normalMap, roughnessMap, roughness: 1, metalness: 0.2, clearcoat: 0.35, clearcoatRoughness: 0.3,
    });
    const out: Record<string, THREE.Mesh> = {};
    obj.traverse((o) => {
      if ((o as THREE.Mesh).isMesh) {
        const m = (o as THREE.Mesh).clone();
        m.material = mat; m.castShadow = true; m.receiveShadow = true;
        out[o.name] = m;
      }
    });
    return out;
  }, [obj, map, normalMap, roughnessMap]);
  const base = useRef<THREE.Group>(null!);
  const sh = useRef<THREE.Group>(null!);
  const el = useRef<THREE.Group>(null!);
  const wr = useRef<THREE.Group>(null!);
  const roll = useRef<THREE.Group>(null!);
  const f1 = useRef<THREE.Group>(null!);
  const f2 = useRef<THREE.Group>(null!);
  useFrame(() => {
    const ik = solveIK(sim.tool, sim.toolYaw);
    base.current.rotation.y = ik.yaw;
    sh.current.rotation.x = ik.shoulder;
    el.current.rotation.x = ik.elbow;
    wr.current.rotation.x = ik.wrist;
    roll.current.rotation.y = ik.roll;
    const close = sim.carry ? 0.01 : 0;
    f1.current.position.x = close;
    f2.current.position.x = -close;
  });
  const sub = (a: number[], b: number[]) => [a[0]! - b[0]!, a[1]! - b[1]!, a[2]! - b[2]!] as [number, number, number];
  const neg = (a: number[]) => [-a[0]!, -a[1]!, -a[2]!] as [number, number, number];
  const P = (name: string, pivot: number[]) => parts[name] && <primitive object={parts[name]} position={neg(pivot)} />;
  return (
    <group position={[ROBOT_BASE.x, 0, ROBOT_BASE.z]}>
      <group position={neg(RIG.yaw)}>{P("robot_arm_base", [0, 0, 0])}</group>
      <group ref={base}>
        {P("robot_arm", RIG.yaw)}
        <group ref={sh} position={sub(RIG.shoulder, RIG.yaw)}>
          {P("robot_arm_large", RIG.shoulder)}
          <group ref={el} position={sub(RIG.elbow, RIG.shoulder)}>
            {P("Robot_arm_2", RIG.elbow)}
            <group ref={wr} position={sub(RIG.wrist, RIG.elbow)}>
              <group ref={roll}>
                {P("robot_arm_end", RIG.wrist)}
                {P("Grabber", RIG.wrist)}
                <group ref={f1}>{P("Grabber_hand_1", RIG.wrist)}</group>
                <group ref={f2}>{P("Grabber_hand_2", RIG.wrist)}</group>
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

      <Suspense fallback={null}>
        <Robot sim={sim} />
      </Suspense>
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
