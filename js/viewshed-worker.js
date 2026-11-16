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

/**
 * Compute Horizon Step List (`ridges = Float32Array([d1, h1, d2, h2, ...])`)
 * for a single azimuth ray using:
 * 1) Near range (0..15 km): Zoom 11 at 60 m step
 * 2) Far range (15..200 km): Zoom 9 with 4-level Hierarchical Ray-Boxes (BVH) and the Iron Rule!
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

  // -------------------------------------------------------------------------
  // Part 1: Near Field (60 m to 15,000 m) at 60 m step using Zoom 11 tiles
  // -------------------------------------------------------------------------
  const nearEndM = Math.min(NEAR_LIMIT_M, maxRangeM);
  for (let d = NEAR_STEP_M; d <= nearEndM; d += NEAR_STEP_M) {
    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    let h = sampleElevationAtZoom(lat, lon, 11);
    if (h === null) {
      h = sampleElevationAtZoom(lat, lon, 9) || 0.0;
    }

    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha > alphaMax) {
      alphaMax = alpha;
      ridgesList.push(d, h);
    }
  }

  if (maxRangeM <= NEAR_LIMIT_M) {
    return new Float32Array(ridgesList);
  }

  // -------------------------------------------------------------------------
  // Part 2: Mid & Far Field (15,000 m to maxRangeM) using Hierarchical BVH Boxes
  // Level 3 super-box = 64 base steps (15,360 m)
  // Level 2 mid-box   = 16 base steps (3,840 m)
  // Level 1 mini-box  = 4 base steps  (960 m)
  // Level 0 base cell = 1 base step   (240 m)
  // -------------------------------------------------------------------------
  const totalFarSteps = Math.ceil((maxRangeM - NEAR_LIMIT_M) / FAR_STEP_M);

  const getStepPixelZ9 = (stepIdx) => {
    const d = NEAR_LIMIT_M + stepIdx * FAR_STEP_M;
    const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    const gx = Math.floor(lonToTileXFloat(lon, 9) * 256.0);
    const gy = Math.floor(latToTileYFloat(lat, 9) * 256.0);
    return [d, gx, gy];
  };

  const traverseHierarchyBox = (stepStart, stepEnd, dStart, gxStart, gyStart, dEnd, gxEnd, gyEnd, pyrLevel) => {
    if (pyrLevel === 0 || stepEnd - stepStart <= 1) {
      for (let s = stepStart + 1; s <= stepEnd; s++) {
        const [d, gx, gy] = s === stepEnd ? [dEnd, gxEnd, gyEnd] : getStepPixelZ9(s);
        const h = queryZoom9PyramidMax(gx, gx, gy, gy, 0);
        const alpha = (h - H0) / d - d * inv2Re;
        if (alpha > alphaMax) {
          alphaMax = alpha;
          ridgesList.push(d, h);
        }
      }
      return;
    }

    const minPxX = (gxStart < gxEnd ? gxStart : gxEnd) - 1;
    const maxPxX = (gxStart > gxEnd ? gxStart : gxEnd) + 1;
    const minPxY = (gyStart < gyEnd ? gyStart : gyEnd) - 1;
    const maxPxY = (gyStart > gyEnd ? gyStart : gyEnd) + 1;

    const hBoxMax = queryZoom9PyramidMax(minPxX, maxPxX, minPxY, maxPxY, pyrLevel);

    // IRON RULE: Strict upper bound angle at the start of the box
    const dForSlope = hBoxMax >= H0 ? dStart : dEnd;
    const alphaUpper = (hBoxMax - H0) / dForSlope - dStart * inv2Re;

    if (alphaUpper <= alphaMax) {
      // "זיכוי" (Culled!): No point in this box can possibly exceed the current horizon alphaMax
      return;
    }

    // "חשד לחסימה" (Subdivide!): Coarse box is NEVER allowed to update alphaMax.
    // Split into 4 sub-boxes of the next finer pyramid level (pyrLevel - 1)
    const span = stepEnd - stepStart;
    const subSpan = Math.max(1, span >> 2);
    const nextLevel = pyrLevel - 1;

    let curStep = stepStart;
    let curD = dStart;
    let curGx = gxStart;
    let curGy = gyStart;

    while (curStep < stepEnd) {
      const nextStep = Math.min(stepEnd, curStep + subSpan);
      const [nextD, nextGx, nextGy] =
        nextStep === stepEnd ? [dEnd, gxEnd, gyEnd] : getStepPixelZ9(nextStep);

      traverseHierarchyBox(
        curStep,
        nextStep,
        curD,
        curGx,
        curGy,
        nextD,
        nextGx,
        nextGy,
        nextLevel
      );

      curStep = nextStep;
      curD = nextD;
      curGx = nextGx;
      curGy = nextGy;
    }
  };

  // Traverse far field in Level-3 super-boxes of 64 steps (~15.36 km each)
  const SUPER_BOX_STEPS = 64;
  let [curD, curGx, curGy] = getStepPixelZ9(0);

  for (let s0 = 0; s0 < totalFarSteps; s0 += SUPER_BOX_STEPS) {
    const s1 = Math.min(totalFarSteps, s0 + SUPER_BOX_STEPS);
    const [nextD, nextGx, nextGy] = getStepPixelZ9(s1);

    traverseHierarchyBox(s0, s1, curD, curGx, curGy, nextD, nextGx, nextGy, 3);

    curD = nextD;
    curGx = nextGx;
    curGy = nextGy;
  }

  return new Float32Array(ridgesList);
}

/**
 * Analytical O(K) evaluation of maximum LOS range `r_max(theta)` from precomputed `ridges`
 * for a given aircraft altitude `hUavM` (ASL in meters) and `maxRangeM`.
 *
 * Implements the exact quadratic shadow intersection equation from Section 4.3:
 *   alpha_j = (h_j - H_0) / d_j - d_j / (2 * R_e)
 *   r_shadow = -R_e * alpha_j + sqrt((R_e * alpha_j)^2 + 2 * R_e * (H_UAV - H_0))
 *   r_max = min(r_max, max(d_j, r_shadow))
 */
function solveRayMaxRangeFromRidges(ridges, hUavM, maxRangeM) {
  const { H0, Re } = stationState;
  const inv2Re = 1.0 / (2.0 * Re);
  const deltaHUav = hUavM - H0;
  const twoReDeltaH = 2.0 * Re * deltaHUav;

  // Theoretical smooth-Earth sea-level horizon if no ridges block
  let rMax = maxRangeM;
  let blockingDistM = 0;
  let blockingHeightM = 0;

  // Also account for smooth sea-level (h=0) horizon when aircraft is far away
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

    // Angle of the aircraft itself if it were at distance dj:
    const uavAngleAtDj = deltaHUav / dj - dj * inv2Re;
    if (uavAngleAtDj <= alphaJ) {
      // Mountain peak at dj is directly at or above the aircraft altitude!
      rMax = dj;
      blockingDistM = dj;
      blockingHeightM = hj;
      break;
    }

    // Solve quadratic equation for r_shadow > dj where tangent ray at angle alphaJ meets H_UAV:
    const reAlpha = Re * alphaJ;
    const disc = reAlpha * reAlpha + twoReDeltaH;
    let rShadow;
    if (disc < 0.0) {
      // Tangent ray dips and rises without reaching H_UAV (only possible if H_UAV < H0)
      continue;
    } else {
      rShadow = -reAlpha + Math.sqrt(disc);
    }

    const effectiveLimit = rShadow > dj ? rShadow : dj;
    if (effectiveLimit < rMax) {
      rMax = effectiveLimit;
      blockingDistM = dj;
      blockingHeightM = hj;
    }
  }

  return {
    rMaxM: Math.max(60.0, Math.min(maxRangeM, rMax)),
    blockingDistM,
    blockingHeightM
  };
}

/**
 * Evaluate all currently cached rays for `(hUavM, maxRangeKm)` and build polygon payload
 */
function evaluatePolygonFromCachedRays(hUavM, maxRangeKm, stageLabel, isFinal) {
  const maxRangeM = maxRangeKm * 1000.0;
  const { lat0, lon0 } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);

  const sortedKeys = Array.from(cachedRaysMap.keys()).sort((a, b) => a - b);
  const numRays = sortedKeys.length;

  const polygonLatLons = new Array(numRays);
  const raySummaries = new Array(numRays);
  let sumRangeKm = 0.0;
  let minRangeKm = maxRangeKm;
  let maxObservedRangeKm = 0.0;

  for (let i = 0; i < numRays; i++) {
    const rayObj = cachedRaysMap.get(sortedKeys[i]);
    const azDeg = rayObj.azimuthDeg;
    const { rMaxM, blockingDistM, blockingHeightM } = solveRayMaxRangeFromRidges(
      rayObj.ridges,
      hUavM,
      maxRangeM
    );

    const rKm = rMaxM / 1000.0;
    sumRangeKm += rKm;
    if (rKm < minRangeKm) minRangeKm = rKm;
    if (rKm > maxObservedRangeKm) maxObservedRangeKm = rKm;

    const [lat, lon] = destinationLatLon(
      lat0Rad,
      lon0Rad,
      sinLat0,
      cosLat0,
      rMaxM,
      azDeg * DEG_TO_RAD
    );

    polygonLatLons[i] = [lat, lon];
    raySummaries[i] = {
      azimuthDeg: azDeg,
      rangeKm: rKm,
      lat,
      lon,
      blockingDistKm: blockingDistM / 1000.0,
      blockingHeightM
    };
  }

  const avgRangeKm = numRays > 0 ? sumRangeKm / numRays : 0;

  return {
    stage: stageLabel,
    isFinal,
    numRays,
    hGroundM: stationState.hGroundM,
    H0: stationState.H0,
    avgRangeKm,
    minRangeKm,
    maxObservedRangeKm,
    polygonLatLons,
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
    kFactor
  } = params;

  const maxLoadKm = 200; // Always trace ridges out to 200 km so slider changes up to 200 km are instant!
  const ok = await preloadTilesForStation(lat0, lon0, maxLoadKm, jobId);
  if (!ok || jobId !== currentJobId) return;

  // Determine ground elevation at station from Zoom 11 Terrarium tile
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

  // =========================================================================
  // STAGE 1: 360 Rays (every 1.0°) -> immediate initial polygon (< 1 second)
  // =========================================================================
  for (let i = 0; i < 360; i++) {
    if (jobId !== currentJobId) return;
    const azDeg = i * 1.0;
    const key = Math.round(azDeg * 4);
    const ridges = traceRayHorizonRidges(azDeg, maxTraceM);
    cachedRaysMap.set(key, { azimuthDeg: azDeg, ridges });
  }

  const stage1Result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, 'stage1-360', false);
  self.postMessage({
    type: 'VIEWSHED_RESULT',
    jobId,
    ...stage1Result
  });

  // Yield briefly so browser renders Stage 1 polygon immediately
  await new Promise((r) => setTimeout(r, 15));
  if (jobId !== currentJobId) return;

  // =========================================================================
  // STAGE 2: Complete to 720 Rays (every 0.5°) + Adaptive 0.25° Refinement
  // =========================================================================
  for (let i = 0; i < 360; i++) {
    if (jobId !== currentJobId) return;
    const azDeg = i + 0.5;
    const key = Math.round(azDeg * 4);
    const ridges = traceRayHorizonRidges(azDeg, maxTraceM);
    cachedRaysMap.set(key, { azimuthDeg: azDeg, ridges });

    if (i % 90 === 0) {
      self.postMessage({
        type: 'PROGRESS',
        jobId,
        phase: 'stage2',
        progressPct: 50 + Math.round((i / 360) * 35),
        label: `מחשב 720 קרניים (0.5°)...`
      });
      await new Promise((r) => setTimeout(r, 0));
    }
  }

  if (jobId !== currentJobId) return;

  // Adaptive 0.25° angular refinement in sectors with sharp range jumps (> 3.5 km)
  const maxRangeM = maxRangeKm * 1000.0;
  const keys720 = Array.from(cachedRaysMap.keys()).sort((a, b) => a - b);
  const ranges720 = keys720.map((k) => {
    const rObj = cachedRaysMap.get(k);
    return solveRayMaxRangeFromRidges(rObj.ridges, hUavM, maxRangeM).rMaxM;
  });

  for (let i = 0; i < keys720.length; i++) {
    const nextIdx = (i + 1) % keys720.length;
    const jumpM = Math.abs(ranges720[nextIdx] - ranges720[i]);
    if (jumpM > 3500.0) {
      const azMid = keys720[i] / 4.0 + 0.25;
      const midKey = Math.round(azMid * 4);
      if (!cachedRaysMap.has(midKey) && midKey < 1440) {
        const ridges = traceRayHorizonRidges(azMid, maxTraceM);
        cachedRaysMap.set(midKey, { azimuthDeg: azMid, ridges });
      }
    }
  }

  const stage2Result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, 'stage2-refined', true);
  self.postMessage({
    type: 'VIEWSHED_RESULT',
    jobId,
    ...stage2Result
  });
}

/**
 * Generate dense terrain cross-section profile along a specific azimuth for the bottom drawer
 */
function buildRayCrossSectionProfile(azimuthDeg, hUavM, maxRangeKm) {
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

  // Find or trace exact ridges on this azimuth to get rMax and blocking ridge
  const nearestKey = Math.round(normAz * 2) * 2;
  const cachedRay = cachedRaysMap.get(nearestKey % 1440);
  const ridges = cachedRay ? cachedRay.ridges : traceRayHorizonRidges(normAz, maxDistM);
  const { rMaxM, blockingDistM, blockingHeightM } = solveRayMaxRangeFromRidges(
    ridges,
    hUavM,
    maxDistM
  );

  const profileSamples = [];
  for (let i = 0; i <= numSamples; i++) {
    const dM = i * stepM;
    let hM = hGroundM;
    if (i > 0) {
      const [lat, lon] = destinationLatLon(lat0Rad, lon0Rad, sinLat0, cosLat0, dM, bearingRad);
      hM =
        dM <= NEAR_LIMIT_M
          ? sampleElevationAtZoom(lat, lon, 11) ?? sampleElevationAtZoom(lat, lon, 9) ?? 0
          : sampleElevationAtZoom(lat, lon, 9) ?? 0;
    }
    const dropM = dM * dM * inv2Re;
    profileSamples.push({
      distanceKm: dM / 1000.0,
      terrainAslM: Math.round(hM),
      curvatureDropM: dropM,
      effectiveHeightM: hM - dropM
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
    endLat,
    endLon,
    blockingDistKm: blockingDistM / 1000.0,
    blockingHeightM: Math.round(blockingHeightM),
    blockLat,
    blockLon,
    H0,
    hUavM,
    Re,
    profileSamples
  };
}

/**
 * Handle Messages from Main Thread
 */
self.onmessage = (e) => {
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

  // Instant 60 FPS update when only aircraft altitude (H_UAV) or max range (R_max) changes!
  if (msg.type === 'FAST_UPDATE_ALT_RANGE') {
    if (cachedRaysMap.size === 0) return;
    const { hUavM, maxRangeKm } = msg.payload;
    const result = evaluatePolygonFromCachedRays(hUavM, maxRangeKm, 'fast-60fps', true);
    self.postMessage({
      type: 'VIEWSHED_RESULT',
      jobId: currentJobId,
      ...result
    });
    return;
  }

  // Request dense cross-section profile along a specific hovered/clicked azimuth
  if (msg.type === 'GET_RAY_PROFILE') {
    if (cachedRaysMap.size === 0) return;
    const { azimuthDeg, hUavM, maxRangeKm } = msg.payload;
    const profile = buildRayCrossSectionProfile(azimuthDeg, hUavM, maxRangeKm);
    self.postMessage({
      type: 'RAY_PROFILE_RESULT',
      profile
    });
  }
};
