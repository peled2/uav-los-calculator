const APP_CACHE = 'uav-los-shell-v1.2';
const TILE_CACHE = 'uav-los-tiles-v1.2';
const ELEVATION_CACHE = 'uav-los-dem-v1.2';

const CORE_ASSETS = [
  '/',
  '/index.html',
  '/css/styles.css',
  '/js/los-math.js',
  '/js/elevation-service.js',
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
    caches.open(APP_CACHE).then((cache) => {
      return Promise.allSettled(
        CORE_ASSETS.map((url) =>
          cache.add(url).catch(() => null)
        )
      );
    }).then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  const allowedCaches = [APP_CACHE, TILE_CACHE, ELEVATION_CACHE];
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys.map((key) => {
          if (!allowedCaches.includes(key)) {
            return caches.delete(key);
          }
          return null;
        })
      )
    ).then(() => self.clients.claim())
  );
});

function isMapTile(url) {
  return (
    url.includes('opentopomap.org') ||
    url.includes('arcgisonline.com') ||
    url.includes('tile.openstreetmap.fr')
  );
}

function isElevationApi(url) {
  return (
    url.includes('api.open-meteo.com/v1/elevation') ||
    url.includes('api.open-elevation.com')
  );
}

self.addEventListener('fetch', (event) => {
  const { request } = event;
  if (request.method !== 'GET') return;

  const url = request.url;

  // Map tiles: Cache-first with background refresh
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

  // Elevation API: Network-first with cache fallback
  if (isElevationApi(url)) {
    event.respondWith(
      caches.open(ELEVATION_CACHE).then(async (cache) => {
        try {
          const response = await fetch(request);
          if (response && response.ok) {
            cache.put(request, response.clone());
          }
          return response;
        } catch (err) {
          const cached = await cache.match(request);
          if (cached) return cached;
          throw err;
        }
      })
    );
    return;
  }

  // App shell & assets: Network-first for HTML/JS/CSS, fallback to cache
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
        throw new Error('Offline and not in cache');
      })
  );
});
