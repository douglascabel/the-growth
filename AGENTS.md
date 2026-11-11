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
- Use pure robot-kinematics capsule poses from the OBJ pivot hierarchy for articulated Rapier contacts; exclude only the grasped bar from robot contact so held bars still interact with the tower.
- Rank removals by support margins and probe Rapier snapshots before extraction; preserve the visible world's poses while choosing the least destabilizing support.
