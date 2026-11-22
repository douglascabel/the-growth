import { useMemo } from 'react';
import * as THREE from 'three';
function archShape(width:number,height:number) {
  const r=width/2,s=new THREE.Shape();s.moveTo(-r,0);s.lineTo(r,0);s.lineTo(r,height-r);s.absarc(0,height-r,r,0,Math.PI,false);s.lineTo(-r,0);return s;
}
function Window({x,z,side=false}:{x:number;z:number;side?:boolean}) {
  const shape=useMemo(()=>archShape(1.65,3.2),[]);
  return <group position={[x,.3,z]} rotation-y={side?Math.PI/2:0}>
    <mesh><shapeGeometry args={[shape]}/><meshStandardMaterial color="#dfe8e5" emissive="#f3f5ed" emissiveIntensity={.55} roughness={.2}/></mesh>
    {[-.81,0,.81].map(v=><mesh key={v} position={[v,1.3,.025]}><boxGeometry args={[.045,2.6,.045]}/><meshStandardMaterial color="#343c36"/></mesh>)}
    <mesh position={[0,1.4,.03]}><boxGeometry args={[1.65,.04,.04]}/><meshStandardMaterial color="#343c36"/></mesh>
    <mesh position={[0,0,.02]}><boxGeometry args={[1.8,.07,.2]}/><meshStandardMaterial color="#e6e5df"/></mesh>
  </group>;
}
function Beam({a,b}:{a:[number,number,number];b:[number,number,number]}) {
  const v=new THREE.Vector3(...b).sub(new THREE.Vector3(...a));const q=new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0,1,0),v.clone().normalize());
  return <mesh position={new THREE.Vector3(...a).add(new THREE.Vector3(...b)).multiplyScalar(.5)} quaternion={q}><boxGeometry args={[.09,v.length(),.09]}/><meshStandardMaterial color="#39433a" roughness={.6} metalness={.6}/></mesh>;
}
export default function GalleryRoom() {
  const timber=useMemo(()=>{
    const c=document.createElement('canvas');c.width=c.height=256;const g=c.getContext('2d');if(!g)return new THREE.Texture();
    g.fillStyle='#918774';g.fillRect(0,0,256,256);
    for(let y=0;y<256;y++){g.fillStyle=`rgba(45,37,23,${.03+Math.random()*.12})`;g.fillRect(0,y,256,1);}
    for(let x=0;x<256;x+=32){g.fillStyle='#665f52';g.fillRect(x,0,2,256);}
    const t=new THREE.CanvasTexture(c);t.colorSpace=THREE.SRGBColorSpace;t.wrapS=t.wrapT=THREE.RepeatWrapping;t.repeat.set(7,5);return t;
  },[]);
  return <group>
    {[-1, 1].map(side => (
      <group key={`glass-${side}`}>
        {[-4.5, -1.5, 1.5, 4.5].map(z => (
          <mesh key={z} position={[side * 6.55, 5.8, z]} rotation-z={side * -0.1} castShadow>
            <boxGeometry args={[0.025, 4.6, 2.2]} />
            <meshPhysicalMaterial color="#dfe8e5" roughness={0.08} transmission={0.72} transparent opacity={0.42} side={THREE.DoubleSide} />
          </mesh>
        ))}
      </group>
    ))}
    {/* White exhibition hall, open front and roof clerestory daylight. */}
    <mesh position={[0,2.8,-7]} receiveShadow><boxGeometry args={[14,5.6,.15]}/><meshStandardMaterial color="#eeeee8" roughness={.85}/></mesh>
    {[-1,1].map(side=><group key={side}>
      <mesh position={[side*7,2.8,0]} receiveShadow><boxGeometry args={[.15,5.6,14]}/><meshStandardMaterial color="#eeeee8" roughness={.85}/></mesh>
      {[-4.8,-1.6,1.6,4.8].map(z=><group key={z}>
        <Window x={side*6.9} z={z} side/>
        <mesh position={[side*6.83,2.6,z-1.25]}><boxGeometry args={[.3,5.2,.3]}/><meshStandardMaterial color="#eeeee8"/></mesh>
      </group>)}
      <mesh position={[side*3.5,6.9,0]} rotation-z={side*-.38} receiveShadow><boxGeometry args={[7.55,.12,14]}/><meshStandardMaterial map={timber} roughness={.8} side={THREE.DoubleSide}/></mesh>
    </group>)}
    {[-4.4,-1.8,1.8,4.4].map(x=><Window key={x} x={x} z={-6.9}/>)}
    {[-5,-2,1,4].map(z=><group key={z}>
      <Beam a={[-7,5.4,z]} b={[0,8.3,z]}/><Beam a={[0,8.3,z]} b={[7,5.4,z]}/>
      <Beam a={[-7,5.4,z]} b={[7,5.4,z]}/>
      <Beam a={[0,5.4,z]} b={[0,8.3,z]}/>
      {[-1,1].map(side=>Array.from({length:12},(_,i)=>{
        const x=side*(.3+i*.53),y=8.1-Math.abs(x)*.41;
        return <mesh key={`${side}-${i}`} position={[x,y-.19,z]} rotation-x={Math.PI/2}><torusGeometry args={[.19,.016,6,16]}/><meshStandardMaterial color="#39433a" metalness={.6} roughness={.5}/></mesh>;
      }))}
    </group>)}
    {[-3.5,3.5].map(x=><mesh key={x} position={[x,7,0]} rotation-z={x<0?.38:-.38}><boxGeometry args={[1.4,.035,2.5]}/><meshStandardMaterial color="#f4f7f1" emissive="#f4f7f1" emissiveIntensity={2}/></mesh>)}
  </group>;
}
