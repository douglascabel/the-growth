import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader.js";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import { MeshoptDecoder } from "three/examples/jsm/libs/meshopt_decoder.module.js";
import { MeshoptSimplifier } from "three/examples/jsm/libs/meshopt_simplifier.module.js";
import { mergeVertices } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { ContactShadows, Environment, Lightformer, OrbitControls, useTexture } from "@react-three/drei";
import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { GRIPPER_CLOSE_TRAVEL, RIG, TAB, ROBOT_BASE, ROBOT_SCALE, createSim, rigProfileFromBones, solveIK, stepSim, type RigProfile, type Vec } from "@/lib/growth-sim";
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

const modelBase = `${import.meta.env.BASE_URL}models/`;
const robotGlbUrl = `${modelBase}kuka-kr210.glb?v=2`;
const goldBarGlbUrl = `${modelBase}gold-bar.glb`;
const goldTextureUrls = [
  `${modelBase}gold-bar/material/color.jpg`,
  `${modelBase}gold-bar/material/normal.jpg`,
  `${modelBase}gold-bar/material/roughness.jpg`,
  `${modelBase}gold-bar/material/metalness.jpg`,
] as string[];
const robotJointNames = ["Track", "Base", "A1", "A2", "A3", "A4", "A5", "A6", "Tool", "JawL", "JawR"] as const;
type RobotJointName = typeof robotJointNames[number];
type RobotRigData = {
  joints:Map<RobotJointName,THREE.Object3D>;
  restQuaternion:Map<RobotJointName,THREE.Quaternion>;
  restPosition:Map<RobotJointName,THREE.Vector3>;
  axes:{
    yaw:THREE.Vector3; shoulder:THREE.Vector3; elbow:THREE.Vector3; wrist:THREE.Vector3; roll:THREE.Vector3;
    jawL:THREE.Vector3; jawR:THREE.Vector3;
  };
};
const robotRigProfiles = new WeakMap<THREE.Group, RigProfile>();
const robotRigData = new WeakMap<THREE.Group, RobotRigData>();

function prepareRobot(scene: THREE.Group) {
  scene.updateMatrixWorld(true);
  const joints = new Map<RobotJointName, THREE.Object3D>();
  scene.traverse((object) => {
    if (robotJointNames.includes(object.name as RobotJointName)) {
      joints.set(object.name as RobotJointName, object);
    }
    if ((object as THREE.Mesh).isMesh) {
      const mesh = object as THREE.Mesh;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      const source = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material;
      if (source instanceof THREE.MeshStandardMaterial) {
        const material = source.clone();
        const grip = /Grabber|Jaw/i.test(mesh.name);
        material.color.set(grip ? "#202326" : "#f26a21");
        material.metalness = grip ? 0.82 : 0.18;
        material.roughness = grip ? 0.32 : 0.38;
        mesh.material = material;
      }
    }
  });
  for (const name of robotJointNames) {
    if (!joints.has(name)) throw new Error(`Missing ${name} joint in ${robotGlbUrl}`);
  }
  const joint = (name:RobotJointName) => joints.get(name)!;
  const worldPosition = (name:RobotJointName) => joint(name).getWorldPosition(new THREE.Vector3());
  const tuple = (v:THREE.Vector3):Vec => [v.x,v.y,v.z];
  const rigPoints = {
    track:tuple(worldPosition("Track")),
    a1:tuple(worldPosition("A1")),
    a2:tuple(worldPosition("A2")),
    a3:tuple(worldPosition("A3")),
    a5:tuple(worldPosition("A5")),
    tool:tuple(worldPosition("Tool")),
  };
  robotRigProfiles.set(scene,rigProfileFromBones(rigPoints));
  const axisInJointFrame = (name:RobotJointName,axis:THREE.Vector3) =>
    axis.clone().applyQuaternion(joint(name).getWorldQuaternion(new THREE.Quaternion()).invert()).normalize();
  const meshCenter = (name:string) => {
    const mesh = scene.getObjectByName(name) as THREE.Mesh | undefined;
    if (!mesh?.isMesh) throw new Error(`Missing ${name} mesh in ${robotGlbUrl}`);
    mesh.geometry.computeBoundingBox();
    return mesh.geometry.boundingBox!.getCenter(new THREE.Vector3()).applyMatrix4(mesh.matrixWorld);
  };
  const jawAxis = meshCenter("KUKA_JawR").sub(meshCenter("KUKA_JawL")).normalize();
  const toParent = (name:RobotJointName,axis:THREE.Vector3) =>
    axis.clone().applyQuaternion(joint(name).parent?.getWorldQuaternion(new THREE.Quaternion()).invert() ?? new THREE.Quaternion()).normalize();
  robotRigData.set(scene,{
    joints,
    restQuaternion:new Map(robotJointNames.map(name=>[name,joint(name).quaternion.clone()])),
    restPosition:new Map(robotJointNames.map(name=>[name,joint(name).position.clone()])),
    axes:{
      yaw:axisInJointFrame("A1",new THREE.Vector3(0,1,0)),
      shoulder:axisInJointFrame("A2",new THREE.Vector3(0,0,1)),
      elbow:axisInJointFrame("A3",new THREE.Vector3(0,0,1)),
      wrist:axisInJointFrame("A5",new THREE.Vector3(0,0,1)),
      roll:axisInJointFrame("A6",worldPosition("Tool").sub(worldPosition("A5")).normalize()),
      jawL:toParent("JawL",jawAxis),
      jawR:toParent("JawR",jawAxis),
    },
  });
  return scene;
}

function useGoldMaterial() {
  const textures = useTexture(goldTextureUrls) as THREE.Texture[];
  const [map, normalMap, roughnessMap, metalnessMap] = textures;
  const colorMap = map!, normal = normalMap!, roughness = roughnessMap!, metalness = metalnessMap!;
  return useMemo(() => {
    colorMap.colorSpace = THREE.SRGBColorSpace;
    for (const texture of [colorMap, normal, roughness, metalness]) {
      texture.anisotropy = 8;
    }
    return new THREE.MeshPhysicalMaterial({
      // The supplied metal maps provide surface detail; this tint gives the
      // material its gold alloy colour without an invented procedural texture.
      color: "#d9a625", map: colorMap, normalMap: normal, roughnessMap: roughness, metalnessMap: metalness,
      normalScale: new THREE.Vector2(.5, .5), metalness: 1, roughness: .28,
      clearcoat: .08, clearcoatRoughness: .22,
    });
  }, [colorMap, normal, roughness, metalness]);
}

function useOptionalRobot() {
  const [object, setObject] = useState<THREE.Group | null>(null);
  useEffect(() => {
    let live = true;
    const dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath(`${import.meta.env.BASE_URL}draco/gltf/`);
    const loader = new GLTFLoader();
    loader.setDRACOLoader(dracoLoader);
    loader.load(robotGlbUrl, (gltf) => {
      try {
        const scene = prepareRobot(gltf.scene);
        if (live) setObject(scene);
      } catch (error) {
        console.error(`Could not prepare the KUKA rig from ${robotGlbUrl}`, error);
        if (live) setObject(null);
      }
    }, undefined, (error) => {
      console.error(`Could not load the KUKA rig from ${robotGlbUrl}`, error);
      if (live) setObject(null);
    });
    return () => {
      live = false;
      dracoLoader.dispose();
    };
  }, []);
  return object;
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
        } catch (error) {
          if (reduced) reduced.dispose();
          else geometry.dispose();
          console.warn(`Could not optimize the gold-bar mesh from ${goldBarGlbUrl}; using fallback geometry.`, error);
          if (live) setSource(null);
        }
      })();
    };
    const dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath(`${import.meta.env.BASE_URL}draco/gltf/`);
    const loader = new GLTFLoader();
    loader.setDRACOLoader(dracoLoader);
    loader.setMeshoptDecoder(MeshoptDecoder);
    loader.load(goldBarGlbUrl, gltf => {
      gltf.scene.updateMatrixWorld(true);
      let geometry: THREE.BufferGeometry | undefined;
      gltf.scene.traverse(child => {
        if (!geometry && (child as THREE.Mesh).isMesh) {
          const mesh = child as THREE.Mesh;
          geometry = mesh.geometry.clone().applyMatrix4(mesh.matrixWorld);
          // The decimated export is authored Y-up, matching normalizeLingot's inscription-up convention.
        }
      });
      if (geometry) optimize(geometry);
      else if (live) setSource(null);
    }, undefined, () => {
      console.error(`Could not load the supplied gold-bar model from ${goldBarGlbUrl}`);
      if (live) setSource(null);
    });
    return () => { live = false; dracoLoader.dispose(); };
  }, []);
  const geometry = useMemo(() => normalizeLingot(source ?? fallback, source?.userData["preserveNormals"] === true), [source, fallback]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => fallback.dispose(), [fallback]);
  useEffect(() => () => source?.dispose(), [source]);
  return geometry;
}

function Robot({ sim, obj }: { sim: ReturnType<typeof createSim>; obj:THREE.Group|null }) {
  const rig = useMemo(() => {
    if (!obj) return null;
    const data = robotRigData.get(obj);
    if (!data) throw new Error(`Missing live joint data in ${robotGlbUrl}`);
    const find = (name:RobotJointName) => data.joints.get(name)!;
    return {
      A1: find("A1"), A2: find("A2"), A3: find("A3"), A5: find("A5"), A6: find("A6"),
      jawL: find("JawL"), jawR: find("JawR"), axes:data.axes,
      rest: {
        A1: data.restQuaternion.get("A1")!, A2: data.restQuaternion.get("A2")!,
        A3: data.restQuaternion.get("A3")!, A5: data.restQuaternion.get("A5")!,
        A6: data.restQuaternion.get("A6")!, jawL: data.restPosition.get("JawL")!,
        jawR: data.restPosition.get("JawR")!,
      },
    };
  }, [obj]);
  const rotation = useMemo(() => new THREE.Quaternion(), []);
  useFrame(() => {
    const ik = solveIK(sim.tool, sim.toolYaw,sim.rig);
    const close = (sim.carry || sim.closed) ? GRIPPER_CLOSE_TRAVEL : 0;
    if (rig) {
      rotation.setFromAxisAngle(rig.axes.yaw, ik.yaw);
      rig.A1.quaternion.copy(rig.rest.A1).multiply(rotation);
      rotation.setFromAxisAngle(rig.axes.shoulder, ik.shoulder);
      rig.A2.quaternion.copy(rig.rest.A2).multiply(rotation);
      rotation.setFromAxisAngle(rig.axes.elbow, ik.elbow);
      rig.A3.quaternion.copy(rig.rest.A3).multiply(rotation);
      rotation.setFromAxisAngle(rig.axes.wrist, ik.wrist);
      rig.A5.quaternion.copy(rig.rest.A5).multiply(rotation);
      rotation.setFromAxisAngle(rig.axes.roll, ik.roll);
      rig.A6.quaternion.copy(rig.rest.A6).multiply(rotation);
      rig.jawL.position.copy(rig.rest.jawL).addScaledVector(rig.axes.jawL, close);
      rig.jawR.position.copy(rig.rest.jawR).addScaledVector(rig.axes.jawR, -close);
    }
  });
  return (
    <group position={[ROBOT_BASE.x, ROBOT_BASE.y, ROBOT_BASE.z]} scale={ROBOT_SCALE}>
      {obj ? <group rotation-y={Math.PI / 2}><primitive object={obj} /></group> : null}
    </group>
  );
}

function Bars({ sim, geometry }: { sim: ReturnType<typeof createSim>; geometry: THREE.BufferGeometry }) {
  const material = useGoldMaterial();
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
    <instancedMesh ref={instances} args={[geometry, material, sim.bars.length]} castShadow receiveShadow frustumCulled={false} />
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
  const obj = useOptionalRobot();
  const sim = useMemo(() => createSim(Math.random,obj ? robotRigProfiles.get(obj) ?? RIG : RIG), [obj]);
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
        <Robot sim={sim} obj={obj} />
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
        camera={{ position: [3.94, 2.9, 8.36], fov: 42, near: 0.05, far: 60 }}
        gl={{ antialias: true, toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.05 }}
      >
        <World />
      </Canvas>
    </div>
  );
}

