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

// Storage keys. v2 records WHY a URL is in use, not just what it is -- see below.
export const ROS_URL_STORAGE_KEY = "bascorro.ros_url.v2";
export const LEGACY_SHARED_ROS_URL_KEY = "bascorro.shared_ros_url.v1";
export const LEGACY_ACTION_ROS_URL_KEY = "op3RosUrl";

const LEGACY_KEYS = [LEGACY_SHARED_ROS_URL_KEY, LEGACY_ACTION_ROS_URL_KEY];

/**
 * Read the saved rosbridge preference, falling back to one derived from the page.
 *
 * The v1 format stored the URL alone, and App.jsx wrote it back on every change --
 * including the auto-derived one. That turned a value nobody chose into a value
 * nothing could correct: rewriteLoopbackToCurrentHost() only repairs loopback
 * hostnames, so an auto-derived LAN address outlived the DHCP lease it came from.
 * Open the UI once at 10.104.28.246, get a new lease tomorrow, and the studio keeps
 * dialling the dead address forever with no hint of why.
 *
 * v2 stores `manual`, so intent survives instead of an address: a URL the user typed
 * is honoured as typed, while an auto-derived one is recomputed from window.location
 * on every load and therefore tracks the current host by construction.
 *
 * Legacy values are read as auto (recomputed, not honoured). They were written
 * indiscriminately, so they carry no evidence of intent -- and keeping them is
 * exactly the staleness this replaces. A genuine manual override survives one
 * re-entry in a field that shows its own value; a stale one is silent and permanent.
 */
export function readRosUrlPreference(storage, locationLike, envValue) {
  const autoUrl = computeDefaultRosbridgeUrl(envValue, locationLike);
  if (!storage) return { url: autoUrl, manual: false };

  try {
    const raw = storage.getItem(ROS_URL_STORAGE_KEY);
    if (raw) {
      const saved = JSON.parse(raw);
      if (saved && saved.manual === true && saved.url) {
        return {
          url: rewriteLoopbackToCurrentHost(normalizeRosbridgeUrl(saved.url), locationLike),
          manual: true,
        };
      }
    }
  } catch {
    // Unreadable or malformed storage: the derived default is always safe.
  }

  return { url: autoUrl, manual: false };
}

/** Persist the preference, retiring the v1 keys as we go. */
export function writeRosUrlPreference(storage, url, manual) {
  if (!storage) return;
  try {
    storage.setItem(ROS_URL_STORAGE_KEY, JSON.stringify({ url, manual: manual === true }));
    for (const key of LEGACY_KEYS) storage.removeItem(key);
  } catch {
    // Ignore storage failures in restrictive browser contexts.
  }
}
