<!-- LOVABLE:BEGIN -->
> [!IMPORTANT]
> This project is connected to [Lovable](https://lovable.dev). Avoid rewriting
> published git history — force pushing, or rebasing/amending/squashing commits
> that are already pushed — as it rewrites history on Lovable's side and the
> user will likely lose their project history.
>
> Commits you push to the connected branch sync back to Lovable and show up in
> the editor, so keep the branch in a working state.

## What Lovable manages

Rules outside this block are the project's own. Change the pieces below in
Lovable, not by hand. With the [Lovable MCP](https://mcp.lovable.dev)
connected, `send_message` asks Lovable to do it; otherwise ask the user to
use the editor.

- **Database.** Do not add or edit `supabase/migrations/` or `drizzle/`.
  Ask Lovable to change the schema.
- **Generated files.** Do not edit a file whose header says it is automatically
  generated.
- **AI gateway.** Keep model calls on Lovable's gateway, and keep
  `LOVABLE_API_KEY` out of browser code.
- **Connectors.** Lovable provides native connectors. `list_connectors` shows
  which are enabled; `add_connector` only returns the dashboard link. When a
  connector is enabled, calls go through the connector gateway and need
  `LOVABLE_API_KEY`. When Lovable does not provide one, users can bring their
  own key and store it in Secrets.
- **Secrets.** Lovable manages them. Do not put secret values in the repo.
<!-- LOVABLE:END -->

## Scene architecture
- Keep placement and robot scaling in the pure growth-sim module so IK and rendering use the same coordinate transform.
- Normalize a cloned supplied STL once to BAR dimensions and share it through instancing for tower and feeder bars to avoid redundant geometry and draw calls.
- Development-only window.__growth exposes state and sim(seconds); #growth[data-scene-ready] is set after loaded artwork draws, enabling reproducible scene checks.
- Use Rapier in a browser-initialized dynamics adapter with metre-scaled contact tolerances; keep choreography and eligibility rules independent of rendering so Jenga collapse is physical and testable.
- Model the gallery as navigable geometry from the room reference rather than a flat photo backdrop, preserving free orbit and zoom.
- Derive tower height from the shared rig reach envelope; preserve bar identity and dynamic poses across collapse/recovery, so new towers reuse the actual pile.
- Normalize supplied lingots through one helper shared with the convex physics hull; the source positive Y face is the embossed top.
- Tower construction is fully dynamic with inset cuboid contacts; the tapered artwork remains visual geometry because its narrow top cannot support the requested tall three-bar pattern reliably.
- Use pure robot-kinematics capsule poses from the KR 16 R2010 GLB pivot hierarchy for articulated Rapier contacts; exclude only the grasped bar from robot contact so held bars still interact with the tower.
- Drive `public/models/kuka-kr16-r2010.glb` by its authored node names (A1–A6, `robot contact surface`, the gripper levers). Its lever gripper is scaled by `GRIPPER_SCALE` so the open pads clear a bar; the pure `jawPose` sets both rendered levers and physical fingers from the shared `sim.jaw`.
- End-grip bars, set them down `NUDGE` beyond their neighbours' ends and slide them home with the leading closed fingertip, so fingers never enter a neighbour. There is no hidden push tab: every push is a real finger capsule contact. The selected bar uses slide friction only while pushed, carried or being extracted.
- Deconstruction is a horizontal-hand side push of a bar with a free lane beside it (outer bars, or a middle bar once a neighbour is gone), a crossing over the tower top, a grip of the protruding end from the opposite side, a straight pull and a drop to the floor. Bars are never restacked on top.
- IK is six-axis (A4 forearm roll, A5 bend kept ≤ 0, A6 flange roll) driven by `toolYaw` and `toolTilt`; renderer and Rapier contacts both use it.
- Hide the authored arrow meshes `Clamp.lever` and `A6-Lever`; they are annotations, not hardware.
- Rank removals by support margins and probe Rapier snapshots before extraction; preserve the visible world's poses. Jitter the ranking, the push side and the pick among stable probes with `sim.random` so choices feel human, not optimal.
- Removal choice is a generator (`planRemoval`, physics `probe`) advanced within `sim.planBudget` ms per frame; never run route checks or probes as one blocking call in the live scene.
- Metal stack physics: stiff contacts (natural frequency 800 Hz, 32 iterations) so the tower neither sags nor sways; damping only for bars in the tower, near zero for loose bars. Concrete floor: friction 0.5, restitution 0.18 (max-combined). Released bars get a nose-down tumble and a little of the pull.
