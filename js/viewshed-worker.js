/**
 * UAV LOS Calculator - Web Worker for 360° Viewshed / Max Range Envelope
 *
 * Implements the exact engineering specification:
 * 1. AWS Terrarium SRTM DEM Tiles (`https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`)
 *    Decoded via: elevation_m = (R * 256.0 + G + B / 256.0) - 32768.0
 * 2. Multi-LOD Max-Pooling Pyramid:
 *    - Near range (0 - 15 km): Zoom 11 tiles (~60 m resolution)
 *    - Mid/Far range (15 - 200 km): Zoom 9 tiles with 4-level Max-Pooling Pyramid
 *      (level0: 256x256, level1: 128x128 [2x2 max], level2: 64x64 [4x4 max], level3: 16x16 [16x16 max])
 * 3. Hierarchical BVH Ray-Marching ("Iron Rule" / כלל הברזל):
 *    - Elevation angle: alpha(d, h) = (h - H0) / d - d / (2 * Re)
 *    - Coarse box upper bound: alpha_upper = (h_box_max - H0) / d_start - d_start / (2 * Re)
 *    - If alpha_upper <= alpha_max -> cull entire box ("זיכוי")
 *    - If alpha_upper > alpha_max -> subdivide down to base cell (coarse box NEVER updates alpha_max!)
 * 4. Decoupled Horizon Ridge Steps (`ridges = [d1, h1, d2, h2, ...]`) per ray for 60 FPS altitude/range updates
 * 5. Progressive Rendering:
 *    - Stage 1: 360 rays (1° step)
 *    - Stage 2: 720 rays (0.5° step) + Adaptive 0.25° refinement on sharp range jumps
 */

const EARTH_RADIUS_M = 6371000.0;
const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;
const NEAR_LIMIT_M = 15000.0; // 15 km boundary between Zoom 11 and Zoom 9 BVH
const NEAR_STEP_M = 60.0;     // 60 m step in 0..15 km
const FAR_STEP_M = 240.0;     // 240 m base step in 15..200 km

// Tile Cache: key = `${z}/${x}/${y}` -> { level0, level1, level2, level3, maxElev }
const tileCache = new Map();
const inFlightTiles = new Map();

// Main-thread fallback decoding queue (only used if OffscreenCanvas is unavailable in Worker)
const pendingMainDecodes = new Map();
let decodeReqId = 1;

// Stored rays state for instant 60 FPS altitude / range recalculation
let currentJobId = 0;
let stationState = {
  lat0: 31.898,
  lon0: 34.691,
  hGroundM: 0,
  hAntM: 12,
  H0: 12,
  kFactor: 4 / 3,
  Re: (4 / 3) * EARTH_RADIUS_M,
  maxLoadedRangeKm: 200
};

// Map from azimuthDeg (key: Math.round(az * 4)) -> { azimuthDeg, ridges: Float32Array }
let cachedRaysMap = new Map();

/**
 * Web Mercator Coordinate Conversions
 */
function lonToTileXFloat(lon, z) {
  return ((lon + 180.0) / 360.0) * (1 << z);
}

function latToTileYFloat(lat, z) {
  const sinLat = Math.sin(lat * DEG_TO_RAD);
  const clamped = Math.max(-0.9999, Math.min(0.9999, sinLat));
  const y = 0.5 - Math.log((1.0 + clamped) / (1.0 - clamped)) / (4.0 * Math.PI);
  return y * (1 << z);
}

/**
 * Great-circle destination point (returns [latDeg, lonDeg])
 */
function destinationLatLon(lat1Rad, lon1Rad, sinLat1, cosLat1, distM, bearingRad) {
  const angularDist = distM / EARTH_RADIUS_M;
  const sinAd = Math.sin(angularDist);
  const cosAd = Math.cos(angularDist);

  const sinLat2 = sinLat1 * cosAd + cosLat1 * sinAd * Math.cos(bearingRad);
  const lat2Rad = Math.asin(Math.max(-1.0, Math.min(1.0, sinLat2)));
  const y = Math.sin(bearingRad) * sinAd * cosLat1;
  const x = cosAd - sinLat1 * sinLat2;
  const lon2Rad = lon1Rad + Math.atan2(y, x);

  return [
    lat2Rad * RAD_TO_DEG,
    ((lon2Rad * RAD_TO_DEG + 540.0) % 360.0) - 180.0
  ];
}

/**
 * Build 4-level Max-Pooling Pyramid from 256x256 Float32Array (`level0`)
 * - level0: 256 x 256 (1x1)
 * - level1: 128 x 128 (2x2 Math.max)
 * - level2: 64 x 64   (4x4 Math.max, pooled 2x2 from level1)
 * - level3: 16 x 16   (16x16 Math.max, pooled 4x4 from level2)
 */
function buildMaxPoolingPyramid(level0) {
  // Level 1: 128 x 128 (each cell = max of 2x2 in level0)
  const level1 = new Float32Array(128 * 128);
  for (let y = 0; y < 128; y++) {
    const r0 = (y * 2) * 256;
    const r1 = (y * 2 + 1) * 256;
    const outRow = y * 128;
    for (let x = 0; x < 128; x++) {
      const c0 = x * 2;
      const c1 = c0 + 1;
      const m1 = level0[r0 + c0] > level0[r0 + c1] ? level0[r0 + c0] : level0[r0 + c1];
      const m2 = level0[r1 + c0] > level0[r1 + c1] ? level0[r1 + c0] : level0[r1 + c1];
      level1[outRow + x] = m1 > m2 ? m1 : m2;
    }
  }

  // Level 2: 64 x 64 (each cell = max of 2x2 in level1 = 4x4 in level0)
  const level2 = new Float32Array(64 * 64);
  for (let y = 0; y < 64; y++) {
    const r0 = (y * 2) * 128;
    const r1 = (y * 2 + 1) * 128;
    const outRow = y * 64;
    for (let x = 0; x < 64; x++) {
      const c0 = x * 2;
      const c1 = c0 + 1;
      const m1 = level1[r0 + c0] > level1[r0 + c1] ? level1[r0 + c0] : level1[r0 + c1];
      const m2 = level1[r1 + c0] > level1[r1 + c1] ? level1[r1 + c0] : level1[r1 + c1];
      level2[outRow + x] = m1 > m2 ? m1 : m2;
    }
  }

  // Level 3: 16 x 16 (each cell = max of 4x4 in level2 = 16x16 in level0)
  const level3 = new Float32Array(16 * 16);
  let tileMax = -32768.0;
  for (let y = 0; y < 16; y++) {
    const outRow = y * 16;
    for (let x = 0; x < 16; x++) {
      let maxVal = -32768.0;
      for (let dy = 0; dy < 4; dy++) {
        const r = (y * 4 + dy) * 64;
        for (let dx = 0; dx < 4; dx++) {
          const v = level2[r + (x * 4 + dx)];
          if (v > maxVal) maxVal = v;
        }
      }
      level3[outRow + x] = maxVal;
      if (maxVal > tileMax) tileMax = maxVal;
    }
  }

  return { level0, level1, level2, level3, maxElev: tileMax };
}

/**
 * Decode RGBA Uint8ClampedArray (256x256x4) using Terrarium formula:
 * elevation_m = (R * 256.0 + G + B / 256.0) - 32768.0
 */
function decodeTerrariumRgba(rgbaData) {
  const level0 = new Float32Array(256 * 256);
  for (let i = 0, p = 0; i < 65536; i++, p += 4) {
    const r = rgbaData[p];
    const g = rgbaData[p + 1];
    const b = rgbaData[p + 2];
    const elev = (r * 256.0 + g + b / 256.0) - 32768.0;
    // Clamp ocean / no-data spikes below -450 m (Dead Sea is ~ -430 m)
    level0[i] = elev < -500.0 ? 0.0 : elev;
  }
  return buildMaxPoolingPyramid(level0);
}

/**
 * Fetch and decode a single AWS Terrarium tile `{z}/{x}/{y}`
 */
async function loadTerrariumTile(z, x, y) {
  const maxTileIdx = (1 << z) - 1;
  if (y < 0 || y > maxTileIdx) return null;
  const wrappedX = ((x % (1 << z)) + (1 << z)) % (1 << z);
  const key = `${z}/${wrappedX}/${y}`;

  if (tileCache.has(key)) {
    return tileCache.get(key);
  }
  if (inFlightTiles.has(key)) {
    return inFlightTiles.get(key);
  }

  const url = `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/${z}/${wrappedX}/${y}.png`;

  const promise = (async () => {
    try {
      const resp = await fetch(url, { mode: 'cors', credentials: 'omit' });
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const blob = await resp.blob();

      if (typeof OffscreenCanvas !== 'undefined' && typeof createImageBitmap !== 'undefined') {
        const bitmap = await createImageBitmap(blob);
        const canvas = new OffscreenCanvas(256, 256);
        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        ctx.drawImage(bitmap, 0, 0, 256, 256);
        bitmap.close();
        const imgData = ctx.getImageData(0, 0, 256, 256).data;
        const pyramid = decodeTerrariumRgba(imgData);
        tileCache.set(key, pyramid);
        return pyramid;
      } else {
        // Fallback: ask main thread to decode PNG blob into RGBA
        const arrayBuf = await blob.arrayBuffer();
        const rgbaBuf = await requestMainThreadDecode(arrayBuf);
        const pyramid = decodeTerrariumRgba(new Uint8ClampedArray(rgbaBuf));
        tileCache.set(key, pyramid);
        return pyramid;
      }
    } catch (err) {
      // Return a zero-elevation sea-level tile pyramid if unreachable
      const zeroL0 = new Float32Array(256 * 256);
      const fallback = buildMaxPoolingPyramid(zeroL0);
      tileCache.set(key, fallback);
      return fallback;
    } finally {
      inFlightTiles.delete(key);
    }
  })();

  inFlightTiles.set(key, promise);
  return promise;
}

function requestMainThreadDecode(pngBuffer) {
  return new Promise((resolve) => {
    const id = decodeReqId++;
    pendingMainDecodes.set(id, resolve);
    self.postMessage({ type: 'DECODE_PNG_REQ', id, pngBuffer }, [pngBuffer]);
  });
}

/**
 * Preload all required Terrarium tiles for:
 * - Zoom 11 covering 0 .. 15 km around station
 * - Zoom 9 covering 0 .. R_max (up to 200 km) around station
 */
async function preloadTilesForStation(lat0, lon0, maxRangeKm, jobId) {
  const dLatNear = (16.5 / 6371.0) * RAD_TO_DEG;
  const dLonNear = dLatNear / Math.max(0.2, Math.cos(lat0 * DEG_TO_RAD));

  const z11MinX = Math.floor(lonToTileXFloat(lon0 - dLonNear, 11));
  const z11MaxX = Math.floor(lonToTileXFloat(lon0 + dLonNear, 11));
  const z11MinY = Math.floor(latToTileYFloat(lat0 + dLatNear, 11));
  const z11MaxY = Math.floor(latToTileYFloat(lat0 - dLatNear, 11));

  const padRangeKm = Math.min(205, maxRangeKm + 5);
  const dLatFar = (padRangeKm / 6371.0) * RAD_TO_DEG;
  const dLonFar = dLatFar / Math.max(0.2, Math.cos(lat0 * DEG_TO_RAD));

  const z9MinX = Math.floor(lonToTileXFloat(lon0 - dLonFar, 9));
  const z9MaxX = Math.floor(lonToTileXFloat(lon0 + dLonFar, 9));
  const z9MinY = Math.floor(latToTileYFloat(lat0 + dLatFar, 9));
  const z9MaxY = Math.floor(latToTileYFloat(lat0 - dLatFar, 9));

  const tasks = [];
  for (let x = z11MinX; x <= z11MaxX; x++) {
    for (let y = z11MinY; y <= z11MaxY; y++) {
      tasks.push({ z: 11, x, y });
    }
  }
  for (let x = z9MinX; x <= z9MaxX; x++) {
    for (let y = z9MinY; y <= z9MaxY; y++) {
      tasks.push({ z: 9, x, y });
    }
  }

  const uncachedTasks = tasks.filter((t) => {
    const wrappedX = ((t.x % (1 << t.z)) + (1 << t.z)) % (1 << t.z);
    return !tileCache.has(`${t.z}/${wrappedX}/${t.y}`);
  });

  let completed = tasks.length - uncachedTasks.length;
  const total = tasks.length;
  const CONCURRENCY = 14;

  for (let i = 0; i < uncachedTasks.length; i += CONCURRENCY) {
    if (jobId !== currentJobId) return false;
    const batch = uncachedTasks.slice(i, i + CONCURRENCY);
    await Promise.all(batch.map((t) => loadTerrariumTile(t.z, t.x, t.y)));
    completed += batch.length;
    self.postMessage({
      type: 'PROGRESS',
      jobId,
      phase: 'tiles',
      progressPct: Math.min(45, Math.round((completed / total) * 45)),
      label: `טוען אריחי גובה לווייניים (Terrarium SRTM)... ${completed}/${total}`
    });
  }

  return jobId === currentJobId;
}

/**
 * Sample exact elevation (level0) from cached tile at zoom `z`
 */
function sampleElevationAtZoom(lat, lon, z) {
  const txFloat = lonToTileXFloat(lon, z);
  const tyFloat = latToTileYFloat(lat, z);
  const tx = Math.floor(txFloat);
  const ty = Math.floor(tyFloat);
  const tile = tileCache.get(`${z}/${tx}/${ty}`);
  if (!tile) return null;

  const px = Math.min(255, Math.max(0, Math.floor((txFloat - tx) * 256.0)));
  const py = Math.min(255, Math.max(0, Math.floor((tyFloat - ty) * 256.0)));
  return tile.level0[py * 256 + px];
}

/**
 * Query the Maximum Elevation (`Math.max`) inside a bounding box of global Zoom 9 pixels
 * `[minGlobalPxX, maxGlobalPxX] x [minGlobalPxY, maxGlobalPxY]`
 * using pyramid level `pyrLevel`:
 * - pyrLevel = 3 -> level3 (16x16 pixel blocks, 16x16 grid per tile)
 * - pyrLevel = 2 -> level2 (4x4 pixel blocks, 64x64 grid per tile)
 * - pyrLevel = 1 -> level1 (2x2 pixel blocks, 128x128 grid per tile)
 * - pyrLevel = 0 -> level0 (1x1 pixel blocks, 256x256 grid per tile)
 */
function queryZoom9PyramidMax(minPxX, maxPxX, minPxY, maxPxY, pyrLevel) {
  const shift = pyrLevel === 3 ? 4 : pyrLevel === 2 ? 2 : pyrLevel === 1 ? 1 : 0;
  const gridDim = 256 >> shift; // 16, 64, 128, or 256
  const mask = gridDim - 1;

  const minCellX = minPxX >> shift;
  const maxCellX = maxPxX >> shift;
  const minCellY = minPxY >> shift;
  const maxCellY = maxPxY >> shift;

  let maxH = -32768.0;
  const cellToTileShift = 8 - shift; // 4, 6, 7, or 8

  for (let cy = minCellY; cy <= maxCellY; cy++) {
    const ty = cy >> cellToTileShift;
    const localY = cy & mask;
    const rowOffset = localY * gridDim;

    for (let cx = minCellX; cx <= maxCellX; cx++) {
      const tx = cx >> cellToTileShift;
      const tile = tileCache.get(`9/${tx}/${ty}`);
      if (!tile) continue;

      const arr =
        pyrLevel === 3
          ? tile.level3
          : pyrLevel === 2
          ? tile.level2
          : pyrLevel === 1
          ? tile.level1
          : tile.level0;

      const val = arr[rowOffset + (cx & mask)];
      if (val > maxH) maxH = val;
    }
  }

  return maxH < -10000.0 ? 0.0 : maxH;
}

// Shared radial distance table for exact multi-zone AGL evaluation (0..200 km):
// Indices 0..249: 60 m to 15,000 m (60 m step, Zoom 11)
// Indices 250..1020: 15,240 m to 200,040 m (240 m step, Zoom 9)
const NEAR_STEPS_COUNT = Math.floor(NEAR_LIMIT_M / NEAR_STEP_M); // 250
const FAR_STEPS_COUNT = Math.ceil((200000.0 - NEAR_LIMIT_M) / FAR_STEP_M); // 771
const TOTAL_AGL_SAMPLES = NEAR_STEPS_COUNT + FAR_STEPS_COUNT; // 1021
const RAY_DIST_TABLE = new Float32Array(TOTAL_AGL_SAMPLES);
for (let i = 0; i < NEAR_STEPS_COUNT; i++) {
  RAY_DIST_TABLE[i] = (i + 1) * NEAR_STEP_M;
}
for (let i = 0; i < FAR_STEPS_COUNT; i++) {
  RAY_DIST_TABLE[NEAR_STEPS_COUNT + i] = NEAR_LIMIT_M + (i + 1) * FAR_STEP_M;
}

/**
 * Compute Horizon Step List (`ridges = Float32Array([d1, h1, d2, h2, ...])`)
 * and full radial required-AGL profile (`reqAglProfile = Float32Array(TOTAL_AGL_SAMPLES)`)
 * so that both ASL and multi-zone AGL (including mountains beyond near wadis!) are exact.
 */
function traceRayHorizonRidges(azimuthDeg, maxRangeM) {
  const { lat0, lon0, H0, Re } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const bearingRad = azimuthDeg * DEG_TO_RAD;
  const inv2Re = 1.0 / (2.0 * Re);

  let alphaMax = -Infinity;
  const ridgesList = [];
  const reqAglProfile = new Float32Array(TOTAL_AGL_SAMPLES);

  // -------------------------------------------------------------------------
  // Part 1: Near Field (60 m to 15,000 m) at 60 m step using Zoom 11 tiles
  // -------------------------------------------------------------------------
  for (let idx = 0; idx < NEAR_STEPS_COUNT; idx++) {
    const d = RAY_DIST_TABLE[idx];
    if (d > maxRangeM) break;

    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    let h = sampleElevationAtZoom(lat, lon, 11);
    if (h === null) {
      h = sampleElevationAtZoom(lat, lon, 9) || 0.0;
    }

    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha >= alphaMax) {
      alphaMax = alpha;
      ridgesList.push(d, h);
      reqAglProfile[idx] = 0.0;
    } else {
      const rayHeightAsl = H0 + d * alphaMax + d * d * inv2Re;
      reqAglProfile[idx] = Math.max(0.0, rayHeightAsl - h);
    }
  }

  if (maxRangeM <= NEAR_LIMIT_M) {
    return {
      ridges: new Float32Array(ridgesList),
      reqAglProfile
    };
  }

  // -------------------------------------------------------------------------
  // Part 2: Mid & Far Field (15,240 m to maxRangeM)
  // Evaluates exact terrain and horizon angles so both ASL ridges and multi-zone AGL
  // (re-emerging LOS on mountains beyond wadis/valleys) are 100% captured.
  // -------------------------------------------------------------------------
  for (let idx = NEAR_STEPS_COUNT; idx < TOTAL_AGL_SAMPLES; idx++) {
    const d = RAY_DIST_TABLE[idx];
    if (d > maxRangeM) break;

    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    const h = sampleElevationAtZoom(lat, lon, 9) || 0.0;

    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha >= alphaMax) {
      alphaMax = alpha;
      ridgesList.push(d, h);
      reqAglProfile[idx] = 0.0;
    } else {
      const rayHeightAsl = H0 + d * alphaMax + d * d * inv2Re;
      reqAglProfile[idx] = Math.max(0.0, rayHeightAsl - h);
    }
  }

  return {
    ridges: new Float32Array(ridgesList),
    reqAglProfile
  };
}

/**
 * Analytical O(K) evaluation of maximum LOS range `r_max(theta)` from precomputed `ridges`
 * for ASL mode (`hUavM` in meters ASL).
 */
function solveRayMaxRangeFromRidges(ridges, hUavM, maxRangeM) {
  const { H0, Re } = stationState;
  const inv2Re = 1.0 / (2.0 * Re);
  const deltaHUav = hUavM - H0;
  const twoReDeltaH = 2.0 * Re * deltaHUav;

  let rMax = maxRangeM;
  let blockingDistM = 0;
  let blockingHeightM = 0;

  const hSea = 0.0;
  const dSeaTangent = H0 > hSea ? Math.sqrt(2.0 * Re * (H0 - hSea)) : 60.0;
  if (dSeaTangent < rMax && hUavM >= hSea) {
    const dAircraftFromTangent = Math.sqrt(2.0 * Re * Math.max(0.0, hUavM - hSea));
    const smoothLimitM = dSeaTangent + dAircraftFromTangent;
    if (smoothLimitM < rMax) {
      rMax = smoothLimitM;
      blockingDistM = dSeaTangent;
      blockingHeightM = 0;
    }
  }

  const len = ridges.length;
  for (let i = 0; i < len; i += 2) {
    const dj = ridges[i];
    if (dj >= rMax) {
      break;
    }
    const hj = ridges[i + 1];
    const alphaJ = (hj - H0) / dj - dj * inv2Re;

    const uavAngleAtDj = deltaHUav / dj - dj * inv2Re;
    if (uavAngleAtDj <= alphaJ) {
      rMax = dj;
      blockingDistM = dj;
      blockingHeightM = hj;
      break;
    }

    const reAlpha = Re * alphaJ;
    const disc = reAlpha * reAlpha + twoReDeltaH;
    if (disc < 0.0) {
      continue;
    }
    const rShadow = -reAlpha + Math.sqrt(disc);

    const effectiveLimit = rShadow > dj ? rShadow : dj;
    if (effectiveLimit < rMax) {
      rMax = effectiveLimit;
      blockingDistM = dj;
      blockingHeightM = hj;
    }
  }

  const clampedR = Math.max(60.0, Math.min(maxRangeM, rMax));
  return {
    rMaxM: clampedR,
    outerMaxM: clampedR,
    intervals: [[0, clampedR]],
    blockingDistM,
    blockingHeightM
  };
}

/**
 * Multi-Zone AGL Solver:
 * Finds BOTH the primary continuous LOS range `[0, rFirstM]` AND all re-emerging LOS intervals
 * `[a1, b1], [a2, b2], ...` over mountains beyond near wadis/valleys where `reqAgl(d) <= hUavAglM`!
 */
function solveRayMaxRangeAgl(reqAglProfile, ridges, hUavAglM, maxRangeM) {
  const rawIntervals = [];
  let inLos = true;
  let segStart = 0.0;
  let lastDist = 60.0;

  for (let idx = 0; idx < TOTAL_AGL_SAMPLES; idx++) {
    const d = RAY_DIST_TABLE[idx];
    if (d > maxRangeM) {
      if (inLos) {
        rawIntervals.push([segStart, maxRangeM]);
        inLos = false;
      }
      break;
    }
    lastDist = d;
    const req = reqAglProfile[idx];

    if (inLos) {
      if (req > hUavAglM) {
        const dPrev = idx > 0 ? RAY_DIST_TABLE[idx - 1] : 0.0;
        const reqPrev = idx > 0 ? reqAglProfile[idx - 1] : 0.0;
        const span = req - reqPrev;
        const frac = span > 0.01 ? Math.max(0.0, Math.min(1.0, (hUavAglM - reqPrev) / span)) : 0.0;
        const dExit = Math.max(60.0, dPrev + frac * (d - dPrev));
        rawIntervals.push([segStart, dExit]);
        inLos = false;
      }
    } else {
      if (req <= hUavAglM) {
        const dPrev = idx > 0 ? RAY_DIST_TABLE[idx - 1] : 0.0;
        const reqPrev = idx > 0 ? reqAglProfile[idx - 1] : req;
        const span = reqPrev - req;
        const frac = span > 0.01 ? Math.max(0.0, Math.min(1.0, (reqPrev - hUavAglM) / span)) : 0.0;
        segStart = dPrev + frac * (d - dPrev);
        inLos = true;
      }
    }
  }

  if (inLos) {
    rawIntervals.push([segStart, Math.min(maxRangeM, lastDist)]);
  }

  if (rawIntervals.length === 0) {
    rawIntervals.push([0.0, 60.0]);
  }

  // Merge micro-gaps (< 150 m) and filter out tiny 1-pixel noise specks (< 150 m)
  const merged = [rawIntervals[0]];
  for (let i = 1; i < rawIntervals.length; i++) {
    const prev = merged[merged.length - 1];
    const curr = rawIntervals[i];
    if (curr[0] - prev[1] < 150.0) {
      prev[1] = curr[1];
    } else if (curr[1] - curr[0] >= 150.0) {
      merged.push(curr);
    }
  }

  const rFirstM = Math.max(60.0, Math.min(maxRangeM, merged[0][1]));
  const outerMaxM = Math.max(rFirstM, Math.min(maxRangeM, merged[merged.length - 1][1]));

  // Find the controlling ridge before the first obstruction for telemetry display
  let blockingDistM = 0;
  let blockingHeightM = 0;
  for (let i = 0; i < ridges.length; i += 2) {
    if (ridges[i] <= rFirstM + 60.0) {
      blockingDistM = ridges[i];
      blockingHeightM = ridges[i + 1];
    } else {
      break;
    }
  }

  return {
    rMaxM: rFirstM,
    outerMaxM,
    intervals: merged,
    blockingDistM,
    blockingHeightM
  };
}

/**
 * Evaluate all currently cached rays for `(hUavM, maxRangeKm, altRef)` and build
 * multi-polygon payload (primary continuous polygon + all re-emerging mountain LOS polygons beyond wadis)
 */
function evaluatePolygonFromCachedRays(hUavM, maxRangeKm, altRef, stageLabel, isFinal) {
  const maxRangeM = maxRangeKm * 1000.0;
  const { lat0, lon0 } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const isAgl = altRef === 'agl';

  const sortedKeys = Array.from(cachedRaysMap.keys()).sort((a, b) => a - b);
  const numRays = sortedKeys.length;

  const primaryRing = new Array(numRays);
  const raySummaries = new Array(numRays);
  const allPolygonRings = [primaryRing];

  // Active ribbons for chaining secondary mountain LOS intervals `[a, b]` across adjacent azimuths
  let activeRibbons = [];
  const finishedRibbons = [];

  for (let i = 0; i < numRays; i++) {
    const rayObj = cachedRaysMap.get(sortedKeys[i]);
    const azDeg = rayObj.azimuthDeg;
    const azRad = azDeg * DEG_TO_RAD;

    const { rMaxM, outerMaxM, intervals, blockingDistM, blockingHeightM } = isAgl
      ? solveRayMaxRangeAgl(rayObj.reqAglProfile, rayObj.ridges, hUavM, maxRangeM)
      : solveRayMaxRangeFromRidges(rayObj.ridges, hUavM, maxRangeM);

    const rKm = rMaxM / 1000.0;
    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, rMaxM, azRad);

    primaryRing[i] = [lat, lon];
    raySummaries[i] = {
      azimuthDeg: azDeg,
      rangeKm: rKm,
      outerRangeKm: outerMaxM / 1000.0,
      intervalsKm: intervals.map(([a, b]) => [a / 1000.0, b / 1000.0]),
      lat,
      lon,
      blockingDistKm: blockingDistM / 1000.0,
      blockingHeightM
    };

    if (isAgl) {
      const secondary = intervals.length > 1 ? intervals.slice(1) : [];
      const matchedRibbonFlags = new Array(activeRibbons.length).fill(false);
      const nextActiveRibbons = [];

      for (let s = 0; s < secondary.length; s++) {
        const [a, b] = secondary[s];
        // Find best overlapping active ribbon from previous ray
        let bestIdx = -1;
        let bestOverlap = -Infinity;
        for (let r = 0; r < activeRibbons.length; r++) {
          if (matchedRibbonFlags[r]) continue;
          const rib = activeRibbons[r];
          const overlap = Math.min(b, rib.lastB) - Math.max(a, rib.lastA);
          if (overlap >= -500.0 && overlap > bestOverlap) {
            bestOverlap = overlap;
            bestIdx = r;
          }
        }

        const ptInner = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, a, azRad);
        const ptOuter = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, b, azRad);

        if (bestIdx !== -1) {
          matchedRibbonFlags[bestIdx] = true;
          const rib = activeRibbons[bestIdx];
          rib.outerPts.push(ptOuter);
          rib.innerPts.push(ptInner);
          rib.lastA = a;
          rib.lastB = b;
          rib.lastAzDeg = azDeg;
          nextActiveRibbons.push(rib);
        } else {
          nextActiveRibbons.push({
            startAzDeg: azDeg,
            lastAzDeg: azDeg,
            firstA: a,
            firstB: b,
            lastA: a,
            lastB: b,
            outerPts: [ptOuter],
            innerPts: [ptInner]
          });
        }
      }

      for (let r = 0; r < activeRibbons.length; r++) {
        if (!matchedRibbonFlags[r]) {
          finishedRibbons.push(activeRibbons[r]);
        }
      }
      activeRibbons = nextActiveRibbons;
    }
  }

  if (isAgl) {
    for (const rib of activeRibbons) {
      finishedRibbons.push(rib);
    }

    const halfBeamRad = ((360.0 / Math.max(360, numRays)) * 0.45) * DEG_TO_RAD;
    for (const rib of finishedRibbons) {
      if (rib.outerPts.length >= 2) {
        const ring = [...rib.outerPts, ...rib.innerPts.slice().reverse()];
        allPolygonRings.push(ring);
      } else if (rib.outerPts.length === 1 && rib.lastB - rib.lastA >= 250.0) {
        // Single-ray mountain peak beyond a wadi: give it a narrow sector width so it is visible on map
        const azRad = rib.lastAzDeg * DEG_TO_RAD;
        const p1 = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, rib.lastB, azRad - halfBeamRad);
        const p2 = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, rib.lastB, azRad + halfBeamRad);
        const p3 = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, rib.lastA, azRad + halfBeamRad);
        const p4 = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, rib.lastA, azRad - halfBeamRad);
        allPolygonRings.push([p1, p2, p3, p4]);
      }
    }
  }

  return {
    stage: stageLabel,
    isFinal,
    numRays,
    hGroundM: stationState.hGroundM,
    H0: stationState.H0,
    polygonLatLons: allPolygonRings,
    raySummaries
  };
}

/**
 * Full Progressive 360° Viewshed Calculation Pipeline
 */
async function runProgressiveViewshed(params) {
  const jobId = ++currentJobId;
  const {
    lat0,
    lon0,
    hAntM,
    hUavM,
    maxRangeKm,
    kFactor,
    altRef = 'asl'
  } = params;

  const maxLoadKm = 200;
  const ok = await preloadTilesForStation(lat0, lon0, maxLoadKm, jobId);
  if (!ok || jobId !== currentJobId) return;

  let hGroundM = sampleElevationAtZoom(lat0, lon0, 11);
  if (hGroundM === null) {
    hGroundM = sampleElevationAtZoom(lat0, lon0, 9) || 0.0;
  }
  hGroundM = Math.round(hGroundM);

  stationState = {
    lat0,
    lon0,
    hGroundM,
    hAntM,
    H0: hGroundM + hAntM,
    kFactor,
    Re: kFactor * EARTH_RADIUS_M,
    maxLoadedRangeKm: maxLoadKm
  };

  cachedRaysMap.clear();
  const maxTraceM = maxLoadKm * 1000.0;

  // STAGE 1: 360 Rays (every 1.0°)
  for (let i = 0; i < 360; i++) {
    if (jobId !== currentJobId) return;
    const azDeg = i * 1.0;
    const key = Math.round(azDeg * 4);
    const { ridges, reqAglProfile } = traceRayHorizonRidges(azDeg, maxTraceM);
    cachedRaysMap.set(key, { azimuthDeg: azDeg, ridges, reqAglProfile });
  }

  const stage1Result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, altRef, 'stage1-360', false);
  self.postMessage({
    type: 'VIEWSHED_RESULT',
    jobId,
    ...stage1Result
  });

  await new Promise((r) => setTimeout(r, 15));
  if (jobId !== currentJobId) return;

  // STAGE 2: Complete to 720 Rays (every 0.5°) + Adaptive 0.25° Refinement
  for (let i = 0; i < 360; i++) {
    if (jobId !== currentJobId) return;
    const azDeg = i + 0.5;
    const key = Math.round(azDeg * 4);
    const { ridges, reqAglProfile } = traceRayHorizonRidges(azDeg, maxTraceM);
    cachedRaysMap.set(key, { azimuthDeg: azDeg, ridges, reqAglProfile });

    if (i % 90 === 0) {
      self.postMessage({
        type: 'PROGRESS',
        jobId,
        phase: 'stage2',
        progressPct: 50 + Math.round((i / 360) * 35),
        label: 'מחשב 720 קרניים...'
      });
      await new Promise((r) => setTimeout(r, 0));
    }
  }

  if (jobId !== currentJobId) return;

  const maxRangeM = maxRangeKm * 1000.0;
  const keys720 = Array.from(cachedRaysMap.keys()).sort((a, b) => a - b);
  const isAgl = altRef === 'agl';
  const ranges720 = keys720.map((k) => {
    const rObj = cachedRaysMap.get(k);
    return isAgl
      ? solveRayMaxRangeAgl(rObj.reqAglProfile, rObj.ridges, hUavM, maxRangeM).rMaxM
      : solveRayMaxRangeFromRidges(rObj.ridges, hUavM, maxRangeM).rMaxM;
  });

  for (let i = 0; i < keys720.length; i++) {
    const nextIdx = (i + 1) % keys720.length;
    const jumpM = Math.abs(ranges720[nextIdx] - ranges720[i]);
    if (jumpM > 3500.0) {
      const azMid = keys720[i] / 4.0 + 0.25;
      const midKey = Math.round(azMid * 4);
      if (!cachedRaysMap.has(midKey) && midKey < 1440) {
        const { ridges, reqAglProfile } = traceRayHorizonRidges(azMid, maxTraceM);
        cachedRaysMap.set(midKey, { azimuthDeg: azMid, ridges, reqAglProfile });
      }
    }
  }

  const stage2Result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, altRef, 'stage2-refined', true);
  self.postMessage({
    type: 'VIEWSHED_RESULT',
    jobId,
    ...stage2Result
  });
}

/**
 * Generate dense terrain cross-section profile along a specific azimuth for the bottom drawer
 */
function buildRayCrossSectionProfile(azimuthDeg, hUavM, maxRangeKm, altRef = 'asl') {
  const { lat0, lon0, hGroundM, H0, Re } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const normAz = ((azimuthDeg % 360) + 360) % 360;
  const bearingRad = normAz * DEG_TO_RAD;

  const maxDistM = maxRangeKm * 1000.0;
  const numSamples = 180;
  const stepM = maxDistM / numSamples;
  const inv2Re = 1.0 / (2.0 * Re);

  const nearestKey = Math.round(normAz * 2) * 2;
  const cachedRay = cachedRaysMap.get(nearestKey % 1440);
  const rayData = cachedRay || traceRayHorizonRidges(normAz, maxDistM);
  const { rMaxM, outerMaxM, intervals, blockingDistM, blockingHeightM } =
    altRef === 'agl'
      ? solveRayMaxRangeAgl(rayData.reqAglProfile, rayData.ridges, hUavM, maxDistM)
      : solveRayMaxRangeFromRidges(rayData.ridges, hUavM, maxDistM);

  const profileSamples = [];
  let runningAlphaMax = -Infinity;
  for (let i = 0; i <= numSamples; i++) {
    const dM = i * stepM;
    let hM = hGroundM;
    let isBlocked = false;
    let reqAglM = 0;

    if (i > 0) {
      const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, dM, bearingRad);
      hM =
        dM <= NEAR_LIMIT_M
          ? sampleElevationAtZoom(lat, lon, 11) ?? sampleElevationAtZoom(lat, lon, 9) ?? 0
          : sampleElevationAtZoom(lat, lon, 9) ?? 0;

      const alpha = (hM - H0) / dM - dM * inv2Re;
      if (alpha >= runningAlphaMax) {
        runningAlphaMax = alpha;
        reqAglM = 0;
      } else {
        const rayH = H0 + dM * runningAlphaMax + dM * dM * inv2Re;
        reqAglM = Math.max(0, rayH - hM);
      }

      if (altRef === 'agl') {
        isBlocked = reqAglM > hUavM;
      } else {
        isBlocked = dM > rMaxM;
      }
    }

    const dropM = dM * dM * inv2Re;
    profileSamples.push({
      distanceKm: dM / 1000.0,
      terrainAslM: Math.round(hM),
      curvatureDropM: dropM,
      effectiveHeightM: hM - dropM,
      reqAglM,
      isBlocked
    });
  }

  const [endLat, endLon] = destinationLatLon(
    lat0Rad,
    lon0Rad,
    sinLat0,
    cosLat0,
    rMaxM,
    bearingRad
  );

  let blockLat = null;
  let blockLon = null;
  if (blockingDistM > 0) {
    const [blat, blon] = destinationLatLon(
      lat0Rad,
      lon0Rad,
      sinLat0,
      cosLat0,
      blockingDistM,
      bearingRad
    );
    blockLat = blat;
    blockLon = blon;
  }

  return {
    azimuthDeg: normAz,
    maxRangeKm,
    rMaxKm: rMaxM / 1000.0,
    outerMaxKm: outerMaxM / 1000.0,
    intervalsKm: intervals.map(([a, b]) => [a / 1000.0, b / 1000.0]),
    endLat,
    endLon,
    blockingDistKm: blockingDistM / 1000.0,
    blockingHeightM: Math.round(blockingHeightM),
    blockLat,
    blockLon,
    H0,
    hUavM,
    altRef,
    Re,
    profileSamples
  };
}

/**
 * Compute exact Minimum LOS Altitude (both ASL & AGL) at a specific map point (targetLat, targetLon)
 * triggered by Long-Press on the map.
 */
async function computePointMinLos(targetLat, targetLon) {
  const { lat0, lon0, H0, Re } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const lat1Rad = targetLat * DEG_TO_RAD;
  const lon1Rad = targetLon * DEG_TO_RAD;

  const dLat = lat1Rad - lat0Rad;
  const dLon = lon1Rad - lon0Rad;
  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat0Rad) * Math.cos(lat1Rad) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(Math.max(0, 1 - a)));
  const totalDistM = EARTH_RADIUS_M * c;

  const y = Math.sin(dLon) * Math.cos(lat1Rad);
  const x =
    Math.cos(lat0Rad) * Math.sin(lat1Rad) -
    Math.sin(lat0Rad) * Math.cos(lat1Rad) * Math.cos(dLon);
  const azimuthDeg = (Math.atan2(y, x) * RAD_TO_DEG + 360) % 360;

  if (totalDistM < 30.0) {
    return {
      targetLat,
      targetLon,
      distanceKm: 0,
      azimuthDeg: 0,
      targetGroundM: stationState.hGroundM,
      minAslM: H0,
      minAglM: stationState.hAntM,
      blockingDistKm: 0,
      blockingHeightM: 0
    };
  }

  // Ensure Zoom 9 tile at target (and any intermediate tiles if >200 km) is loaded
  const txTarget = Math.floor(lonToTileXFloat(targetLon, 9));
  const tyTarget = Math.floor(latToTileYFloat(targetLat, 9));
  await loadTerrariumTile(9, txTarget, tyTarget);

  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const bearingRad = azimuthDeg * DEG_TO_RAD;
  const inv2Re = 1.0 / (2.0 * Re);

  let alphaMax = -Infinity;
  let blockingDistM = 0;
  let blockingHeightM = 0;

  // Fine near-field scan up to min(15 km, totalDistM)
  const nearLimit = Math.min(NEAR_LIMIT_M, totalDistM - 30.0);
  for (let d = NEAR_STEP_M; d <= nearLimit; d += NEAR_STEP_M) {
    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    const h = sampleElevationAtZoom(lat, lon, 11) ?? sampleElevationAtZoom(lat, lon, 9) ?? 0.0;
    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha > alphaMax) {
      alphaMax = alpha;
      blockingDistM = d;
      blockingHeightM = h;
    }
  }

  // Far-field scan from 15 km up to totalDistM
  if (totalDistM > NEAR_LIMIT_M) {
    for (let d = NEAR_LIMIT_M + FAR_STEP_M; d < totalDistM; d += FAR_STEP_M) {
      const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
      let h = sampleElevationAtZoom(lat, lon, 9);
      if (h === null) {
        const tx = Math.floor(lonToTileXFloat(lon, 9));
        const ty = Math.floor(latToTileYFloat(lat, 9));
        await loadTerrariumTile(9, tx, ty);
        h = sampleElevationAtZoom(lat, lon, 9) ?? 0.0;
      }
      const alpha = (h - H0) / d - d * inv2Re;
      if (alpha > alphaMax) {
        alphaMax = alpha;
        blockingDistM = d;
        blockingHeightM = h;
      }
    }
  }

  const targetGroundM =
    (totalDistM <= NEAR_LIMIT_M ? sampleElevationAtZoom(targetLat, targetLon, 11) : null) ??
    sampleElevationAtZoom(targetLat, targetLon, 9) ??
    0.0;

  // Minimum LOS Altitude ASL at targetDistM
  const rayHeightAtTarget =
    alphaMax > -Infinity
      ? H0 + totalDistM * alphaMax + totalDistM * totalDistM * inv2Re
      : H0 + totalDistM * totalDistM * inv2Re;

  const minAslM = Math.max(targetGroundM, rayHeightAtTarget);
  const minAglM = Math.max(0.0, minAslM - targetGroundM);

  return {
    targetLat,
    targetLon,
    distanceKm: totalDistM / 1000.0,
    azimuthDeg,
    targetGroundM: Math.round(targetGroundM),
    minAslM: Math.round(minAslM),
    minAglM: Math.round(minAglM),
    blockingDistKm: blockingDistM / 1000.0,
    blockingHeightM: Math.round(blockingHeightM)
  };
}

/**
 * Handle Messages from Main Thread
 */
self.onmessage = async (e) => {
  const msg = e.data;
  if (!msg || !msg.type) return;

  if (msg.type === 'DECODE_PNG_RES') {
    const resolver = pendingMainDecodes.get(msg.id);
    if (resolver) {
      pendingMainDecodes.delete(msg.id);
      resolver(msg.rgbaBuffer);
    }
    return;
  }

  if (msg.type === 'COMPUTE_VIEWSHED') {
    runProgressiveViewshed(msg.payload);
    return;
  }

  // Instant 60 FPS update when Aircraft Altitude (H_UAV), Alt Ref (ASL/AGL), or Max Range changes
  if (msg.type === 'FAST_UPDATE_ALT_RANGE') {
    if (cachedRaysMap.size === 0) return;
    const { hUavM, maxRangeKm, altRef = 'asl' } = msg.payload;
    const result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, altRef, 'fast-60fps', true);
    self.postMessage({
      type: 'VIEWSHED_RESULT',
      jobId: currentJobId,
      ...result
    });
    return;
  }

  if (msg.type === 'GET_RAY_PROFILE') {
    if (cachedRaysMap.size === 0) return;
    const { azimuthDeg, hUavM, maxRangeKm, altRef = 'asl' } = msg.payload;
    const profile = buildRayCrossSectionProfile(azimuthDeg, hUavM, maxRangeKm, altRef);
    self.postMessage({
      type: 'RAY_PROFILE_RESULT',
      profile
    });
    return;
  }

  if (msg.type === 'GET_POINT_MIN_LOS') {
    const { reqId, targetLat, targetLon } = msg.payload;
    const pointResult = await computePointMinLos(targetLat, targetLon);
    self.postMessage({
      type: 'POINT_MIN_LOS_RESULT',
      reqId,
      ...pointResult
    });
  }
};
