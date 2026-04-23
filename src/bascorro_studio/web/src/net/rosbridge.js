const LOOPBACK_HOSTS = new Set([
  "localhost",
  "127.0.0.1",
  "::1",
  "0.0.0.0",
  "::",
]);

function cleanHost(rawHost) {
  return String(rawHost || "")
    .trim()
    .toLowerCase()
    .replace(/^\[/, "")
    .replace(/\]$/, "");
}

function cleanUrlString(url) {
  if (url.pathname === "/" && !url.search && !url.hash) {
    url.pathname = "";
  }
  return url.toString();
}

export function isLoopbackHost(hostname) {
  const host = cleanHost(hostname);
  return LOOPBACK_HOSTS.has(host);
}

export function normalizeRosbridgeUrl(rawValue, fallback = "ws://localhost:9090") {
  const rawText = String(rawValue ?? "").trim();
  const fallbackText = String(fallback ?? "").trim() || "ws://localhost:9090";
  const candidate = rawText || fallbackText;
  const withScheme = /^[a-z]+:\/\//i.test(candidate) ? candidate : `ws://${candidate}`;

  try {
    const parsed = new URL(withScheme);
    if (parsed.protocol !== "ws:" && parsed.protocol !== "wss:") {
      parsed.protocol = "ws:";
    }
    if (!parsed.port) {
      parsed.port = "9090";
    }
    return cleanUrlString(parsed);
  } catch {
    return normalizeRosbridgeUrl(fallbackText, "ws://localhost:9090");
  }
}

export function rewriteLoopbackToCurrentHost(rawValue, locationLike) {
  const safeLocation = locationLike || (typeof window !== "undefined" ? window.location : null);
  const normalized = normalizeRosbridgeUrl(rawValue);
  let parsed;
  try {
    parsed = new URL(normalized);
  } catch {
    return normalized;
  }

  const pageHost = cleanHost(safeLocation?.hostname);
  if (!isLoopbackHost(parsed.hostname) || !pageHost || isLoopbackHost(pageHost)) {
    return cleanUrlString(parsed);
  }

  parsed.hostname = pageHost;
  return cleanUrlString(parsed);
}

export function computeDefaultRosbridgeUrl(envValue, locationLike) {
  const safeLocation = locationLike || (typeof window !== "undefined" ? window.location : null);
  const defaultUrl = normalizeRosbridgeUrl(envValue || "ws://localhost:9090");
  return rewriteLoopbackToCurrentHost(defaultUrl, safeLocation);
}
