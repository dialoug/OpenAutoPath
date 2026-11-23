import { readFile } from "node:fs/promises";
import { defineConfig } from "vite";

const browserPackages = ["react", "react-dom", "scheduler", "three", "lucide-react"];

export default defineConfig({
    plugins: [{
        name: "browser-licenses",
        async generateBundle() {
            const licenses = await Promise.all(browserPackages.map(async (name) => {
                const file = new URL(`./node_modules/${name}/LICENSE`, import.meta.url);
                return `${name}\n\n${(await readFile(file, "utf8")).trimEnd()}`;
            }));
            this.emitFile({
                type: "asset",
                fileName: "third-party-licenses.txt",
                source: licenses.join("\n\n") + "\n",
            });
        },
    }],
});
