import * as THREE from 'three';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';
import { BAR } from './growth-sim';

// Supplied STL: +X length, +Y embossed top, +Z width. Do not rotate X.
export function normalizeLingot(source:THREE.BufferGeometry) {
  const g=source.clone();g.computeBoundingBox();
  const bounds=g.boundingBox;if(!bounds)return g;
  const size=bounds.getSize(new THREE.Vector3()),center=bounds.getCenter(new THREE.Vector3());
  g.translate(-center.x,-center.y,-center.z);
  g.scale(BAR.L/size.x,BAR.H/size.y,BAR.W/size.z);g.computeVertexNormals();return g;
}
/** Unique convex-hull corner points; full STL vertex sets make Rapier hulls extremely slow. */
export function lingotHullPoints(geometry: THREE.BufferGeometry): Float32Array {
  const hull = new ConvexHull().setFromObject(new THREE.Mesh(geometry));
  const seen = new Map<string, THREE.Vector3>();
  for (const face of hull.faces) {
    let e = face.edge;
    do { const p = e.head().point; seen.set(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`, p); e = e.next; } while (e !== face.edge);
  }
  return new Float32Array([...seen.values()].flatMap((p) => [p.x, p.y, p.z]));
}
