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
  // Snap to a 3 mm grid first: keeps the bevelled silhouette with a few dozen corners.
  const src = geometry.getAttribute('position'), grid = new Map<string, THREE.Vector3>(), q = 0.003;
  for (let i = 0; i < src.count; i++) { const v = new THREE.Vector3(Math.round(src.getX(i) / q) * q, Math.round(src.getY(i) / q) * q, Math.round(src.getZ(i) / q) * q); grid.set(`${v.x},${v.y},${v.z}`, v); }
  const hull = new ConvexHull().setFromPoints([...grid.values()]);
  const seen = new Map<string, THREE.Vector3>();
  for (const face of hull.faces) {
    let e = face.edge;
    do { const p = e.head().point; seen.set(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`, p); e = e.next; } while (e !== face.edge);
  }
  return new Float32Array([...seen.values()].flatMap((p) => [p.x, p.y, p.z]));
}
