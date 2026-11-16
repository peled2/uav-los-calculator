const APP_CACHE = 'uav-los-shell-v2.0';
const TILE_CACHE = 'uav-los-tiles-v2.0';
const TERRARIUM_CACHE = 'uav-los-terrarium-v2.0';

const CORE_ASSETS = [
  '/',
  '/index.html',
  '/css/styles.css',
  '/js/los-math.js',
  '/js/viewshed-worker.js',
  '/js/chart-renderer.js',
  '/js/app.js',
  '/manifest.webmanifest',
  '/icons/icon.svg',
  '/icons/icon-192.png',
  '/icons/icon-512.png',
  '/icons/apple-touch-icon.png',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(APP_CACHE).then((cache) =>
      Promise.allSettled(
        CORE_ASSETS.map((url) => cache.add(url).catch(() => null))
      )
    ).then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  const allowed = [APP_CACHE, TILE_CACHE, TERRARIUM_CACHE];
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys.map((key) => (!allowed.includes(key) ? caches.delete(key) : null))
      )
    ).then(() => self.clients.claim())
  );
});

function isMapTile(url) {
  return (
    url.includes('opentopomap.org') ||
    url.includes('israelhiking.osm.org.il') ||
    url.includes('arcgisonline.com')
  );
}

function isTerrariumTile(url) {
  return url.includes('s3.amazonaws.com/elevation-tiles-prod/terrarium/');
}

self.addEventListener('fetch', (event) => {
  const { request } = event;
  if (request.method !== 'GET') return;

  const url = request.url;

  // AWS Terrarium SRTM DEM Tiles: Cache-First for rapid offline & repeat scans
  if (isTerrariumTile(url)) {
    event.respondWith(
      caches.open(TERRARIUM_CACHE).then(async (cache) => {
        const cached = await cache.match(request);
        if (cached) return cached;
        const response = await fetch(request);
        if (response && response.status === 200) {
          cache.put(request, response.clone());
        }
        return response;
      })
    );
    return;
  }

  // Map Tiles (OpenTopoMap, Israel Hiking Map, Esri World Imagery)
  if (isMapTile(url)) {
    event.respondWith(
      caches.open(TILE_CACHE).then(async (cache) => {
        const cached = await cache.match(request);
        const networkFetch = fetch(request)
          .then((response) => {
            if (response && (response.status === 200 || response.type === 'opaque')) {
              cache.put(request, response.clone());
            }
            return response;
          })
          .catch(() => cached);
        return cached || networkFetch;
      })
    );
    return;
  }

  // App Shell: Network-first with cache fallback
  event.respondWith(
    fetch(request)
      .then((response) => {
        if (response && response.status === 200) {
          const copy = response.clone();
          caches.open(APP_CACHE).then((cache) => cache.put(request, copy));
        }
        return response;
      })
      .catch(async () => {
        const cached = await caches.match(request);
        if (cached) return cached;
        if (request.mode === 'navigate') {
          return caches.match('/index.html');
        }
        throw new Error('Offline');
      })
  );
});
