import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { MeshoptSimplifier } from "three/examples/jsm/libs/meshopt_simplifier.module.js";
import { mergeVertices } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { ContactShadows, Environment, Lightformer, OrbitControls } from "@react-three/drei";
import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { GRIPPER_CLOSE_TRAVEL, RIG, TAB, MAX_BARS, ROBOT_BASE, ROBOT_SCALE, createSim, solveIK, stepSim } from "@/lib/growth-sim";
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

const gold = new THREE.MeshPhysicalMaterial({ color: "#e8b13b", metalness: 0.96, roughness: 0.22, clearcoat: 0.32, clearcoatRoughness: 0.18 });
const modelBase = `${import.meta.env.BASE_URL}models/`;
const robotObjUrl = `${modelBase}robot-arm/industrial-robot-arm.obj`;
const goldBarObjUrl = `${modelBase}gold-bar/gold-bar.obj`;
const goldBarGlbUrl = `${modelBase}gold-bar/gold-bar.glb`;
const robotTextureUrls = [
  `${modelBase}robot-arm/robot-color.png`,
  `${modelBase}robot-arm/robot-normal.png`,
  `${modelBase}robot-arm/robot-roughness.png`,
];

function useOptionalRobot() {
  const [object, setObject] = useState<THREE.Group | null>(null);
  useEffect(() => {
    let live = true;
    new OBJLoader().load(robotObjUrl, loaded => { if (live) setObject(loaded); }, undefined, () => { if (live) setObject(null); });
    return () => { live = false; };
  }, []);
  return object;
}

function useOptionalRobotTextures() {
  const [textures, setTextures] = useState<THREE.Texture[] | null>(null);
  useEffect(() => {
    let live = true, failed = false, remaining = robotTextureUrls.length;
    const loaded: (THREE.Texture | undefined)[] = [];
    const loader = new THREE.TextureLoader();
    robotTextureUrls.forEach((url, index) => loader.load(url, texture => {
      if (!live) { texture.dispose(); return; }
      loaded[index] = texture;
      remaining--;
      if (!failed && remaining === 0) setTextures(loaded as THREE.Texture[]);
    }, undefined, () => {
      failed = true;
      if (live) setTextures(null);
    }));
    return () => { live = false; loaded.forEach(texture => texture?.dispose()); };
  }, []);
  return textures;
}

function useLingotGeometry() {
  const fallback = useMemo(() => new RoundedBoxGeometry(155.37, 41.45, 62.14, 6, 3), []);
  const [source, setSource] = useState<THREE.BufferGeometry | null>(null);
  useEffect(() => {
    let live = true;
    const optimize = (geometry: THREE.BufferGeometry) => {
      void (async () => {
        let reduced: THREE.BufferGeometry | undefined;
        try {
          const positionOnly = new THREE.BufferGeometry()
            .setAttribute("position", geometry.getAttribute("position").clone());
          if (geometry.index) positionOnly.setIndex(geometry.index.clone());
          geometry.dispose();
          reduced = mergeVertices(positionOnly, 1e-6);
          positionOnly.dispose();
          await MeshoptSimplifier.ready;
          const positions = reduced.getAttribute("position");
          const vertexData = new Float32Array(positions.count * 3);
          for (let i = 0; i < positions.count; i++) {
            vertexData[i * 3] = positions.getX(i);
            vertexData[i * 3 + 1] = positions.getY(i);
            vertexData[i * 3 + 2] = positions.getZ(i);
          }
          const index = reduced.getIndex();
          if (index && positions.count > 12_000) {
            const target = Math.max(3, Math.floor(index.count * 12_000 / positions.count / 3) * 3);
            const sourceIndices = index.array instanceof Uint32Array ? index.array : Uint32Array.from(index.array);
            const [simplified] = MeshoptSimplifier.simplify(sourceIndices, vertexData, 3, target, 0.01, ["Prune"]);
            reduced.setIndex(new THREE.BufferAttribute(simplified, 1));
          }
          reduced.computeVertexNormals();
          reduced.userData["preserveNormals"] = true;
          if (live) setSource(reduced);
          else reduced.dispose();
        } catch {
          if (reduced) reduced.dispose();
          else geometry.dispose();
          if (live) setSource(null);
        }
      })();
    };
    const loader = new GLTFLoader();
    loader.setMeshoptDecoder(MeshoptDecoder);
    loader.load(goldBarGlbUrl, gltf => {
      gltf.scene.updateMatrixWorld(true);
      let geometry: THREE.BufferGeometry | undefined;
      gltf.scene.traverse(child => {
        if (!geometry && (child as THREE.Mesh).isMesh) {
          const mesh = child as THREE.Mesh;
          geometry = mesh.geometry.clone().applyMatrix4(mesh.matrixWorld);
          // The source mesh is authored Y-up; cancel the embedded Z-up conversion so its embossed face is uppermost.
          geometry.rotateX(Math.PI / 2);
        }
      });
      if (geometry) optimize(geometry);
      else if (live) setSource(null);
    }, undefined, () => {
      new OBJLoader().load(goldBarObjUrl, object => {
        let geometry: THREE.BufferGeometry | undefined;
        object.traverse(child => {
          if (!geometry && (child as THREE.Mesh).isMesh) geometry = (child as THREE.Mesh).geometry.clone();
        });
        if (geometry) optimize(geometry);
        else if (live) setSource(null);
      }, undefined, () => { if (live) setSource(null); });
    });
    return () => { live = false; };
  }, []);
  const geometry = useMemo(() => normalizeLingot(source ?? fallback, source?.userData["preserveNormals"] === true), [source, fallback]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => fallback.dispose(), [fallback]);
  useEffect(() => () => source?.dispose(), [source]);
  return geometry;
}

function AxisLink({ to, radius, material }: { to: [number, number, number]; radius: number; material: THREE.Material }) {
  const pose = useMemo(() => {
    const vector = new THREE.Vector3(...to), length = vector.length();
    return {
      position: [vector.x / 2, vector.y / 2, vector.z / 2] as [number, number, number],
      quaternion: new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), vector.normalize()),
      length,
    };
  }, [to[0], to[1], to[2]]);
  return <mesh position={pose.position} quaternion={pose.quaternion} material={material} castShadow receiveShadow>
    <cylinderGeometry args={[radius, radius, pose.length, 16]} />
  </mesh>;
}


function Robot({ sim }: { sim: ReturnType<typeof createSim> }) {
  const obj = useOptionalRobot();
  const tex = useOptionalRobotTextures();
  const [map, normalMap, roughnessMap] = tex ?? [];
  const material = useMemo(() => {
    if (map && normalMap && roughnessMap) {
      map.colorSpace = THREE.SRGBColorSpace;
      for (const t of [map, normalMap, roughnessMap]) t.anisotropy = 8;
      return new THREE.MeshPhysicalMaterial({
        map, normalMap, roughnessMap, roughness: 1, metalness: 0.2, clearcoat: 0.35, clearcoatRoughness: 0.3,
      });
    }
    return new THREE.MeshPhysicalMaterial({ color: "#737d82", roughness: 0.34, metalness: 0.72, clearcoat: 0.3 });
  }, [map, normalMap, roughnessMap]);
  const parts = useMemo(() => {
    if (!obj) return {};
    const out: Record<string, THREE.Mesh> = {};
    obj.traverse((o) => {
      if ((o as THREE.Mesh).isMesh) {
        const m = (o as THREE.Mesh).clone();
        m.material = material; m.castShadow = true; m.receiveShadow = true;
        out[o.name] = m;
      }
    });
    return out;
  }, [obj, material]);
  useEffect(() => () => material.dispose(), [material]);
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
    const close = (sim.carry || sim.closed) ? GRIPPER_CLOSE_TRAVEL : 0;
    f1.current.position.x = close;
    f2.current.position.x = -close;
  });
  const sub = (a: [number, number, number], b: [number, number, number]): [number, number, number] => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const neg = (a: [number, number, number]): [number, number, number] => [-a[0], -a[1], -a[2]];
  const P = (name: string, pivot: [number, number, number]) => parts[name] && <primitive object={parts[name]} position={neg(pivot)} />;
  return (
    <group position={[ROBOT_BASE.x, ROBOT_BASE.y, ROBOT_BASE.z]} scale={ROBOT_SCALE}>
      <group position={neg(RIG.yaw)}>{obj ? P("robot_arm_base", [0, 0, 0]) : <mesh position={[0, .07, 0]} material={material} castShadow><cylinderGeometry args={[.25, .3, .14, 24]} /></mesh>}</group>
      <group ref={base}>
        {obj ? P("robot_arm", RIG.yaw) : <AxisLink to={sub(RIG.shoulder, RIG.yaw)} radius={.14} material={material} />}
        <group ref={sh} position={sub(RIG.shoulder, RIG.yaw)}>
          {obj ? P("robot_arm_large", RIG.shoulder) : <AxisLink to={sub(RIG.elbow, RIG.shoulder)} radius={.12} material={material} />}
          <group ref={el} position={sub(RIG.elbow, RIG.shoulder)}>
            {obj ? P("Robot_arm_2", RIG.elbow) : <AxisLink to={sub(RIG.wrist, RIG.elbow)} radius={.085} material={material} />}
            <group ref={wr} position={sub(RIG.wrist, RIG.elbow)}>
              <group ref={roll}>
                {obj ? P("robot_arm_end", RIG.wrist) : <AxisLink to={sub(RIG.roll, RIG.wrist)} radius={.06} material={material} />}
                {obj ? P("Grabber", RIG.wrist) : <mesh position={[0, -.04, 0]} material={material} castShadow><boxGeometry args={[.24, .08, .18]} /></mesh>}
                <group ref={f1}>{obj ? P("Grabber_hand_1", RIG.wrist) : <mesh position={[0, -.13, 0]} material={material} castShadow><boxGeometry args={[.035, .2, .035]} /></mesh>}</group>
                <group ref={f2}>{obj ? P("Grabber_hand_2", RIG.wrist) : <mesh position={[0, -.13, 0]} material={material} castShadow><boxGeometry args={[.035, .2, .035]} /></mesh>}</group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

function Bars({ sim, geometry }: { sim: ReturnType<typeof createSim>; geometry: THREE.BufferGeometry }) {
  const instances = useRef<THREE.InstancedMesh>(null);
  const dummy = useMemo(() => new THREE.Object3D(), []);
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
/** The contact surface belongs to the closed hand; it is not separate visible tooling. */
function PushTab({ sim }: { sim: ReturnType<typeof createSim> }) {
  const ref = useRef<THREE.Mesh>(null);
  useFrame(() => {
    const m = ref.current; if (!m) return;
    m.visible = false;
    if (sim.push) { m.position.set(...sim.push.p); m.rotation.set(0, sim.push.yaw, 0); }
  });
  return <mesh ref={ref} material={steel} castShadow><boxGeometry args={[TAB.L, TAB.H, TAB.W]} /></mesh>;
}

function Stepper({ sim, geometry }: { sim: ReturnType<typeof createSim>; geometry: THREE.BufferGeometry }) {
  const n = useRef(0);
  const ready = useRef(false);
  useEffect(() => {
    let live = true;
    ready.current = false;
    createDynamics(lingotHullPoints(geometry)).then(physics => { if (live) {sim.physics = physics; ready.current = true;} else physics.reset(); });
    return () => {live = false;ready.current = false;sim.physics?.reset();};
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
  const lingotGeometry = useLingotGeometry();
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
        <Bars sim={sim} geometry={lingotGeometry} />
        <PushTab sim={sim} />
        <Stepper sim={sim} geometry={lingotGeometry} />
      </Suspense>

      <OrbitControls
        target={[0.13, 0.65, -0.28]} enableDamping dampingFactor={0.08}
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
        camera={{ position: [1.4, 1.4, 2.6], fov: 42, near: 0.05, far: 60 }}
        gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.05 }}
      >
        <World />
      </Canvas>
    </div>
  );
}

