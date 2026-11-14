declare module "three/examples/jsm/libs/meshopt_simplifier.module.js" {
  type SimplifierFlag = "LockBorder" | "Sparse" | "ErrorAbsolute" | "Prune" | "Regularize" | "Permissive";

  export const MeshoptSimplifier: {
    readonly ready: Promise<void>;
    simplify(
      indices: Uint32Array,
      vertexPositions: Float32Array,
      vertexPositionsStride: number,
      targetIndexCount: number,
      targetError: number,
      flags?: SimplifierFlag[],
    ): [Uint32Array, number];
  };
}
