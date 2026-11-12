import { Canvas, useFrame, useLoader, useThree } from "@react-three/fiber";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import lingotAsset from "@/assets/gold-filipe-light.stl.asset.json";
import robotObj from "@/assets/industrial_robot_arm.obj.asset.json";
import colorTex from "@/assets/robot_arm_color.jpg.asset.json";
import normalTex from "@/assets/robot_arm_normal.jpg.asset.json";
import roughTex from "@/assets/robot_arm_roughness.jpg.asset.json";
import { ContactShadows, Environment, useTexture, Lightformer, OrbitControls } from "@react-three/drei";
import { Suspense, useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { RIG, TAB, MAX_BARS, ROBOT_BASE, ROBOT_SCALE, createSim, solveIK, stepSim } from "@/lib/growth-sim";
import { createDynamics } from "@/lib/growth-physics";
import { lingotHullPoints, normalizeLingot } from "@/lib/lingot-geometry";
import GalleryRoom from "./GalleryRoom";


function useConcreteTexture() {
  return useMemo(() => {
    const c = document.createElement("canvas");
    c.width = c.height = 512;
    const g = c.getContext("2d");
    if (!g) return new THREE.Texture();
    g.fillStyle = "#a9ada9";
    g.fillRect(0, 0, 512, 512);
    for (let i = 0; i < 18000; i++) {
      const v = 140 + Math.random() * 50;
      g.fillStyle = `rgba(${v},${v - 2},${v - 5},${Math.random() * 0.25})`;
      g.fillRect(Math.random() * 512, Math.random() * 512, 1 + Math.random() * 3, 1 + Math.random() * 3);
    }
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(18, 18);
    t.colorSpace = THREE.SRGBColorSpace;
    return t;
  }, []);
}

const gold = new THREE.MeshPhysicalMaterial({ color: "#e8b54a", metalness: 1, roughness: 0.24, clearcoat: 0.3 });


function Robot({ sim }: { sim: ReturnType<typeof createSim> }) {
  const obj = useLoader(OBJLoader, robotObj.url);
  const tex = useTexture([colorTex.url, normalTex.url, roughTex.url]) as unknown as THREE.Texture[];
  const [map, normalMap, roughnessMap] = tex;
  const parts = useMemo(() => {
    if (!map || !normalMap || !roughnessMap) return {};
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
  const base = useRef<THREE.Group>(null);
  const sh = useRef<THREE.Group>(null);
  const el = useRef<THREE.Group>(null);
  const wr = useRef<THREE.Group>(null);
  const roll = useRef<THREE.Group>(null);
  const f1 = useRef<THREE.Group>(null);
  const f2 = useRef<THREE.Group>(null);
  useFrame(() => {
    if (!base.current || !sh.current || !el.current || !wr.current || !roll.current || !f1.current || !f2.current) return;
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
  const sub = (a: [number, number, number], b: [number, number, number]): [number, number, number] => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const neg = (a: [number, number, number]): [number, number, number] => [-a[0], -a[1], -a[2]];
  const P = (name: string, pivot: [number, number, number]) => parts[name] && <primitive object={parts[name]} position={neg(pivot)} />;
  return (
    <group position={[ROBOT_BASE.x, ROBOT_BASE.y, ROBOT_BASE.z]} scale={ROBOT_SCALE}>
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
  const source = useLoader(STLLoader, lingotAsset.url);
  const geometry = useMemo(() => normalizeLingot(source), [source]);
  const instances = useRef<THREE.InstancedMesh>(null);
  const dummy = useMemo(() => new THREE.Object3D(), []);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useFrame(() => {
    const mesh = instances.current;
    if (!mesh) return;
    sim.bars.forEach((b, i) => {
      dummy.position.set(...b.p);
      if (b.q) dummy.quaternion.set(b.q.x, b.q.y, b.q.z, b.q.w);
      else dummy.rotation.set(b.pitch, b.yaw, b.roll);
      dummy.scale.setScalar(b.visible ? 1 : 0);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    });
    mesh.instanceMatrix.needsUpdate = true;
  });
  return (
    <instancedMesh ref={instances} args={[geometry, gold, MAX_BARS]} castShadow receiveShadow frustumCulled={false} />
  );
}

const steel = new THREE.MeshStandardMaterial({ color: "#3a3d40", metalness: 0.8, roughness: 0.35 });
/** Fingertip push tab, visible only while it is in use. */
function PushTab({ sim }: { sim: ReturnType<typeof createSim> }) {
  const ref = useRef<THREE.Mesh>(null);
  useFrame(() => {
    const m = ref.current; if (!m) return;
    m.visible = !!sim.push;
    if (sim.push) { m.position.set(...sim.push.p); m.rotation.set(0, sim.push.yaw, 0); }
  });
  return <mesh ref={ref} material={steel} castShadow><boxGeometry args={[TAB.L, TAB.H, TAB.W]} /></mesh>;
}

function Stepper({ sim }: { sim: ReturnType<typeof createSim> }) {
  const source = useLoader(STLLoader, lingotAsset.url);
  const geometry = useMemo(() => normalizeLingot(source), [source]);
  const n = useRef(0);
  const ready = useRef(false);
  useEffect(() => {
    let live = true;
    createDynamics(lingotHullPoints(geometry)).then(physics => { if (live) {sim.physics = physics; ready.current = true;} });
    return () => {live = false;};
  }, [sim, geometry]);
  useEffect(() => {
    if (!import.meta.env.DEV) return;
    const debug = {
      state: sim,
      sim: (seconds: number) => { for (let i = 0; i < seconds * 60; i++) stepSim(sim, 1 / 60); },
    };
    Object.assign(window, { __growth: debug });
    return () => { Reflect.deleteProperty(window, "__growth"); };
  }, [sim]);
  useFrame((_, d) => {
    if (ready.current) stepSim(sim, d);
    if (ready.current && ++n.current === 3) document.querySelector("#growth")?.setAttribute("data-scene-ready", "1");
  }, -1);
  return null;
}

function World() {
  const floorTex = useConcreteTexture();
  const sim = useMemo(() => createSim(), []);
  const { camera, size } = useThree();
  useEffect(() => {
    if (!(camera instanceof THREE.PerspectiveCamera)) return;
    camera.fov = size.width < 600 ? 60 : 50;
    camera.updateProjectionMatrix();
  }, [camera, size.width]);
  return (
    <>
      <color attach="background" args={["#e8eae6"]} />
      <fog attach="fog" args={["#e8eae6", 15, 35]} />
      <group scale={0.65}><GalleryRoom /></group>
      <ambientLight intensity={0.5} />
      <directionalLight
        position={[3, 7, -4]} intensity={2.2} castShadow
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
        <meshStandardMaterial map={floorTex} roughness={0.42} metalness={0} />
      </mesh>
      <ContactShadows position={[0, 0.002, 0]} scale={10} blur={2.5} opacity={0.5} far={2} />

      <Suspense fallback={null}>
        <Robot sim={sim} />
        <Bars sim={sim} />
        <PushTab sim={sim} />
        <Stepper sim={sim} />
      </Suspense>

      <OrbitControls
        target={[0, 0.65, 0.15]} enableDamping dampingFactor={0.08}
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
        camera={{ position: [2.25, 1.4, 2.9], fov: 42, near: 0.05, far: 60 }}
        gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.05 }}
      >
        <World />
      </Canvas>
    </div>
  );
}
