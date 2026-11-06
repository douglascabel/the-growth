import { createFileRoute } from "@tanstack/react-router";
import GrowthScene from "@/components/GrowthScene";

export const Route = createFileRoute("/")({
  ssr: false,
  head: () => ({
    meta: [
      { title: "The Growth — Filipe Vilas-Boas & Douglas Cabel" },
      { name: "description", content: "3D prototype of The Growth: a robot arm endlessly stacks gold bars until the tower falls." },
      { property: "og:title", content: "The Growth — robotic sculpture" },
      { property: "og:description", content: "A robot stacks gold bars, the tower falls, it starts again. A loop on the myth of infinite growth." },
      { property: "og:type", content: "website" },
      { name: "twitter:card", content: "summary_large_image" },
    ],
  }),
  component: Index,
});

function Index() {
  return (
    <main className="relative h-screen w-screen overflow-hidden bg-background">
      <GrowthScene />
      <div className="pointer-events-none absolute left-6 top-6 text-foreground">
        <h1 className="font-serif text-3xl tracking-tight">The Growth</h1>
        <p className="mt-1 text-sm text-muted-foreground">Filipe Vilas-Boas &amp; Douglas Cabel</p>
      </div>
      <p className="pointer-events-none absolute bottom-5 left-6 text-xs text-muted-foreground">
        Drag to orbit · scroll or pinch to zoom
      </p>
    </main>
  );
}
