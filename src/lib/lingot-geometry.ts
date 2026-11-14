import * as THREE from 'three';
import { mergeVertices, toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';
import { BAR } from './growth-sim';

// Supplied STL: +X length, +Y embossed top, +Z width. Do not rotate X.
export function normalizeLingot(source:THREE.BufferGeometry,preserveNormals=false) {
  const g=source.clone();g.computeBoundingBox();
  const bounds=g.boundingBox;if(!bounds)return g;
  const size=bounds.getSize(new THREE.Vector3()),center=bounds.getCenter(new THREE.Vector3());
  g.translate(-center.x,-center.y,-center.z);
  g.scale(BAR.L/size.x,BAR.H/size.y,BAR.W/size.z);
  // Preserve precomputed normals for dense GLB meshes; rebuild OBJ/fallback normals with crisp bar edges.
  if(preserveNormals&&g.getAttribute('normal'))return g;
  g.deleteAttribute('normal');return toCreasedNormals(mergeVertices(g,1e-6),Math.PI/3);
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
  // Rescale into the true bounds (slightly inset) so neighbours never start interpenetrating.
  geometry.computeBoundingBox();
  const box = geometry.boundingBox!, size = box.getSize(new THREE.Vector3()), mid = box.getCenter(new THREE.Vector3());
  const pts = [...seen.values()], hb = new THREE.Box3().setFromPoints(pts), hs = hb.getSize(new THREE.Vector3()), hc = hb.getCenter(new THREE.Vector3());
  return new Float32Array(pts.flatMap((p) => [mid.x + (p.x - hc.x) * size.x / hs.x * 0.995, mid.y + (p.y - hc.y) * size.y / hs.y * 0.998, mid.z + (p.z - hc.z) * size.z / hs.z * 0.99]));
}

