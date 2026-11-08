import * as THREE from 'three';
import { BAR } from './growth-sim';

// Supplied STL: +X length, +Y embossed top, +Z width. Do not rotate X.
export function normalizeLingot(source:THREE.BufferGeometry) {
  const g=source.clone();g.computeBoundingBox();
  const bounds=g.boundingBox;if(!bounds)return g;
  const size=bounds.getSize(new THREE.Vector3()),center=bounds.getCenter(new THREE.Vector3());
  g.translate(-center.x,-center.y,-center.z);
  g.scale(BAR.L/size.x,BAR.H/size.y,BAR.W/size.z);g.computeVertexNormals();return g;
}