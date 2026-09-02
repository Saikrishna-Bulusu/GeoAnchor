/**
 * Static export on purpose.
 *
 * The same bundle has to serve two places that could not be more different:
 * FastAPI on the Jetson, where it talks to a live WebSocket on the LAN, and
 * Vercel, where there is no Jetson to reach and it reads an exported
 * session.json instead. A static bundle is the only artefact both can host.
 */
const nextConfig = {
  output: 'export',
  images: { unoptimized: true },
  trailingSlash: true,
  reactStrictMode: true,
};
export default nextConfig;
