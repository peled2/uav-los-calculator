/**
 * UAV LOS Calculator - Geodesic, Earth Curvature (RF/Optical), Fresnel & Terrain Math Engine
 * Supports ranges up to 200 km with exact spherical geodesy and effective Earth radius k-factor.
 */

export const EARTH_RADIUS_M = 6371000; // Mean physical Earth radius (6,371 km)
export const EARTH_RADIUS_KM = 6371;
export const MAX_RANGE_KM = 200;
export const M_TO_FT = 3.280839895;
export const FT_TO_M = 0.3048;
export const KM_TO_NM = 1 / 1.852;
export const SPEED_OF_LIGHT_MPS = 299792458;

export const PROPAGATION_PRESETS = {
  rf: {
    id: 'rf',
    labelHe: 'RF אלחוטי (k = 4/3)',
    shortHe: 'RF (4/3)',
    kFactor: 4 / 3, // ~1.3333
    useFresnel: true,
    descriptionHe: 'שבירה אטמוספרית סטנדרטית לגלי רדיו (רדיוס כדה"א אפקטיבי: 8,495 ק"מ)'
  },
  optical: {
    id: 'optical',
    labelHe: 'אופטי גיאומטרי (k = 1.0)',
    shortHe: 'אופטי (1.0)',
    kFactor: 1.0,
    useFresnel: false,
    descriptionHe: 'קו ראייה גיאומטרי ישר ללא כיפוף אטמוספרי (רדיוס כדה"א פיזי: 6,371 ק"מ)'
  },
  optical_atm: {
    id: 'optical_atm',
    labelHe: 'אופטי אטמוספרי (k = 1.15)',
    shortHe: 'אופטי שבירה (1.15)',
    kFactor: 1.15,
    useFresnel: false,
    descriptionHe: 'קו ראייה אופטי/אינפרא-אדום עם שבירה אטמוספרית מתונה (רדיוס אפקטיבי: 7,327 ק"מ)'
  },
  custom: {
    id: 'custom',
    labelHe: 'מותאם אישית (k ידני)',
    shortHe: 'מותאם',
    kFactor: 1.3333,
    useFresnel: true,
    descriptionHe: 'הגדרה ידנית של מקדם השבירה האטמוספרי k לתנאי תיעול (Ducting) או תת-שבירה'
  }
};

export function toRadians(deg) {
  return (deg * Math.PI) / 180;
}

export function toDegrees(rad) {
  return (rad * 180) / Math.PI;
}

export function metersToFeet(m) {
  return m * M_TO_FT;
}

export function feetToMeters(ft) {
  return ft * FT_TO_M;
}

/**
 * Great-circle distance between two coordinates using Haversine formula (returns km)
 */
export function haversineDistanceKm(lat1, lon1, lat2, lon2) {
  const dLat = toRadians(lat2 - lat1);
  const dLon = toRadians(lon2 - lon1);
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(rLat1) * Math.cos(rLat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(Math.max(0, 1 - a)));
  return EARTH_RADIUS_KM * c;
}

/**
 * Forward azimuth (initial bearing) in degrees [0, 360) from point 1 to point 2
 */
export function initialBearingDeg(lat1, lon1, lat2, lon2) {
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);
  const dLon = toRadians(lon2 - lon1);

  const y = Math.sin(dLon) * Math.cos(rLat2);
  const x =
    Math.cos(rLat1) * Math.sin(rLat2) -
    Math.sin(rLat1) * Math.cos(rLat2) * Math.cos(dLon);
  const brng = toDegrees(Math.atan2(y, x));
  return (brng + 360) % 360;
}

/**
 * Destination coordinate given starting point, distance (km), and bearing (degrees)
 */
export function destinationPoint(lat, lon, distanceKm, bearingDeg) {
  const angularDist = distanceKm / EARTH_RADIUS_KM;
  const brng = toRadians(bearingDeg);
  const lat1 = toRadians(lat);
  const lon1 = toRadians(lon);

  const sinLat1 = Math.sin(lat1);
  const cosLat1 = Math.cos(lat1);
  const sinAd = Math.sin(angularDist);
  const cosAd = Math.cos(angularDist);

  const sinLat2 = sinLat1 * cosAd + cosLat1 * sinAd * Math.cos(brng);
  const lat2 = Math.asin(Math.max(-1, Math.min(1, sinLat2)));
  const y = Math.sin(brng) * sinAd * cosLat1;
  const x = cosAd - sinLat1 * sinLat2;
  const lon2 = lon1 + Math.atan2(y, x);

  return {
    lat: toDegrees(lat2),
    lon: ((toDegrees(lon2) + 540) % 360) - 180
  };
}

/**
 * Interpolate great-circle points between (lat1, lon1) and (lat2, lon2)
 */
export function interpolateGreatCircle(lat1, lon1, lat2, lon2, numSamples = 100) {
  const totalDistKm = haversineDistanceKm(lat1, lon1, lat2, lon2);
  if (totalDistKm < 1e-5) {
    return [{ lat: lat1, lon: lon1, distanceKm: 0, fraction: 0 }];
  }

  const phi1 = toRadians(lat1);
  const lam1 = toRadians(lon1);
  const phi2 = toRadians(lat2);
  const lam2 = toRadians(lon2);
  const delta = totalDistKm / EARTH_RADIUS_KM;
  const sinDelta = Math.sin(delta);

  const points = [];
  const steps = Math.max(2, numSamples);
  for (let i = 0; i < steps; i++) {
    const f = i / (steps - 1);
    if (sinDelta < 1e-7) {
      points.push({
        lat: lat1 + f * (lat2 - lat1),
        lon: lon1 + f * (lon2 - lon1),
        distanceKm: f * totalDistKm,
        fraction: f
      });
      continue;
    }
    const A = Math.sin((1 - f) * delta) / sinDelta;
    const B = Math.sin(f * delta) / sinDelta;

    const x = A * Math.cos(phi1) * Math.cos(lam1) + B * Math.cos(phi2) * Math.cos(lam2);
    const y = A * Math.cos(phi1) * Math.sin(lam1) + B * Math.cos(phi2) * Math.sin(lam2);
    const z = A * Math.sin(phi1) + B * Math.sin(phi2);

    const phi = Math.atan2(z, Math.sqrt(x * x + y * y));
    const lam = Math.atan2(y, x);

    points.push({
      lat: toDegrees(phi),
      lon: toDegrees(lam),
      distanceKm: f * totalDistKm,
      fraction: f
    });
  }
  return points;
}

/**
 * Earth curvature bulge (m) at distance x_m along a chord of length totalD_m
 * with effective Earth radius factor k.
 */
export function earthCurvatureBulgeM(xM, totalDM, kFactor = 4 / 3) {
  if (totalDM <= 0 || xM <= 0 || xM >= totalDM) return 0;
  const rEff = kFactor * EARTH_RADIUS_M;
  return (xM * (totalDM - xM)) / (2 * rEff);
}

/**
 * Earth curvature drop (m) from the local horizontal tangent plane at distance d_m
 */
export function earthCurvatureDropM(distanceM, kFactor = 4 / 3) {
  if (distanceM <= 0) return 0;
  const rEff = kFactor * EARTH_RADIUS_M;
  return (distanceM * distanceM) / (2 * rEff);
}

/**
 * Theoretical smooth-Earth horizon distance (km) for given GCS height (m ASL) and UAV height (m ASL)
 */
export function theoreticalHorizonKm(gcsHeightM, uavHeightM, kFactor = 4 / 3) {
  const rEff = kFactor * EARTH_RADIUS_M;
  const h1 = Math.max(0, gcsHeightM);
  const h2 = Math.max(0, uavHeightM);
  const d1M = Math.sqrt(2 * rEff * h1);
  const d2M = Math.sqrt(2 * rEff * h2);
  return (d1M + d2M) / 1000;
}

/**
 * Single-station radio/optical horizon distance (km) to sea-level tangent point
 */
export function singleHorizonKm(heightM, kFactor = 4 / 3) {
  const rEff = kFactor * EARTH_RADIUS_M;
  return Math.sqrt(2 * rEff * Math.max(0, heightM)) / 1000;
}

/**
 * First Fresnel Zone radius F1 (m) at distance x_m along total path D_m for frequency f_MHz
 */
export function firstFresnelRadiusM(xM, totalDM, freqMHz = 2400) {
  if (totalDM <= 0 || xM <= 0 || xM >= totalDM || freqMHz <= 0) return 0;
  const wavelengthM = SPEED_OF_LIGHT_MPS / (freqMHz * 1e6);
  const d1 = xM;
  const d2 = totalDM - xM;
  return Math.sqrt((wavelengthM * d1 * d2) / totalDM);
}

/**
 * Free-Space Path Loss (FSPL) in dB
 */
export function calculateFsplDb(slantRangeKm, freqMHz = 2400) {
  if (slantRangeKm <= 0.001 || freqMHz <= 0) return 0;
  return 20 * Math.log10(slantRangeKm) + 20 * Math.log10(freqMHz) + 32.44;
}

/**
 * Perform full LOS & Terrain Profile Analysis between GCS and UAV
 *
 * @param {Object} params
 * @param {Array<{lat:number, lon:number, distanceKm:number, elevationM:number, manualObstacleM?:number}>} params.samples
 * @param {number} params.gcsGroundM - GCS ground elevation (m ASL)
 * @param {number} params.gcsAntennaAglM - GCS mast/antenna height above ground (m AGL)
 * @param {number} params.uavAltitudeAslM - UAV altitude above sea level (m ASL)
 * @param {number} params.kFactor - Effective Earth radius factor (e.g., 4/3 for RF, 1.0 for Optical)
 * @param {boolean} params.useFresnel - Whether to include Fresnel clearance requirement
 * @param {number} params.freqMHz - RF frequency in MHz
 * @param {number} params.fresnelClearancePct - Required 1st Fresnel clearance percentage (e.g., 60)
 * @param {number} params.safetyMarginM - Additional vertical safety clearance margin (m)
 */
export function analyzeLosProfile(params) {
  const {
    samples,
    gcsGroundM,
    gcsAntennaAglM,
    uavAltitudeAslM,
    kFactor = 4 / 3,
    useFresnel = true,
    freqMHz = 2400,
    fresnelClearancePct = 60,
    safetyMarginM = 0
  } = params;

  if (!samples || samples.length < 2) {
    return null;
  }

  const totalDistanceKm = samples[samples.length - 1].distanceKm;
  const totalDistanceM = totalDistanceKm * 1000;
  const gcsTotalAslM = gcsGroundM + gcsAntennaAglM;
  const uavGroundM = samples[samples.length - 1].elevationM || 0;
  const uavAglM = uavAltitudeAslM - uavGroundM;
  const fresnelFactor = useFresnel ? Math.max(0, fresnelClearancePct) / 100 : 0;

  let minDirectClearanceM = Infinity;
  let minEffectiveClearanceM = Infinity;
  let criticalSampleIndex = -1;
  let firstBlockedIndex = -1;
  let maxRequiredUavAslM = uavGroundM + safetyMarginM;

  const profilePoints = samples.map((s, idx) => {
    const xKm = s.distanceKm;
    const xM = xKm * 1000;
    const frac = totalDistanceM > 0 ? xM / totalDistanceM : 0;

    const terrainAslM = Number.isFinite(s.elevationM) ? s.elevationM : 0;
    const obstacleExtraM = Number.isFinite(s.manualObstacleM) ? s.manualObstacleM : 0;
    const rawSurfaceM = terrainAslM + obstacleExtraM;

    // Chord bulge and tangent drop
    const bulgeM = earthCurvatureBulgeM(xM, totalDistanceM, kFactor);
    const dropM = earthCurvatureDropM(xM, kFactor);

    // Effective terrain height in chord frame (curved Earth surface relative to straight ray)
    const effectiveTerrainM = rawSurfaceM + bulgeM;

    // Straight LOS beam altitude in chord frame
    const losBeamM = gcsTotalAslM + frac * (uavAltitudeAslM - gcsTotalAslM);

    // First Fresnel Zone radius at x
    const fresnelR1M = firstFresnelRadiusM(xM, totalDistanceM, freqMHz);
    const requiredFresnelM = fresnelR1M * fresnelFactor;
    const totalRequiredClearanceM = requiredFresnelM + safetyMarginM;

    // Clearances at this point
    const directClearanceM = losBeamM - effectiveTerrainM;
    const effectiveClearanceM = directClearanceM - totalRequiredClearanceM;

    // Exclude the exact endpoints (idx === 0) for intermediate obstruction checks
    if (idx > 0 && idx < samples.length - 1) {
      if (effectiveClearanceM < minEffectiveClearanceM) {
        minEffectiveClearanceM = effectiveClearanceM;
        criticalSampleIndex = idx;
      }
      if (directClearanceM < minDirectClearanceM) {
        minDirectClearanceM = directClearanceM;
      }
      if (effectiveClearanceM < 0 && firstBlockedIndex === -1) {
        firstBlockedIndex = idx;
      }
    }

    // Required UAV altitude (ASL) at distance D to clear this sample point x
    if (idx > 0 && xM > 0) {
      const targetAtX = effectiveTerrainM + totalRequiredClearanceM;
      const reqUavForPoint =
        gcsTotalAslM + (totalDistanceM / xM) * (targetAtX - gcsTotalAslM);
      if (reqUavForPoint > maxRequiredUavAslM) {
        maxRequiredUavAslM = reqUavForPoint;
      }
    }

    return {
      index: idx,
      lat: s.lat,
      lon: s.lon,
      distanceKm: xKm,
      fraction: frac,
      terrainAslM,
      obstacleExtraM,
      rawSurfaceM,
      bulgeM,
      dropM,
      effectiveTerrainM,
      losBeamM,
      fresnelR1M,
      requiredFresnelM,
      safetyMarginM,
      losLowerEnvelopeM: losBeamM - requiredFresnelM - safetyMarginM,
      directClearanceM,
      effectiveClearanceM,
      isDirectBlocked: idx > 0 && idx < samples.length - 1 && directClearanceM < 0,
      isFresnelOrMarginBlocked:
        idx > 0 && idx < samples.length - 1 && effectiveClearanceM < 0
    };
  });

  if (!Number.isFinite(minEffectiveClearanceM)) {
    minEffectiveClearanceM = uavAglM - safetyMarginM;
    minDirectClearanceM = uavAglM;
    criticalSampleIndex = Math.floor(samples.length / 2);
  }

  const isHardBlocked = minDirectClearanceM < 0;
  const isSoftBlocked = !isHardBlocked && minEffectiveClearanceM < 0;
  const status = isHardBlocked
    ? 'BLOCKED'
    : isSoftBlocked
    ? 'WARNING'
    : 'CLEAR';

  const criticalPoint =
    criticalSampleIndex >= 0 ? profilePoints[criticalSampleIndex] : profilePoints[0];
  const firstBlockedPoint =
    firstBlockedIndex >= 0 ? profilePoints[firstBlockedIndex] : null;

  // Compute maximum unobstructed range along this radial for the current UAV altitude
  // (Assuming UAV flies at uavAltitudeAslM along this radial)
  let maxUnobstructedRangeKm = totalDistanceKm;
  if (firstBlockedPoint) {
    maxUnobstructedRangeKm = firstBlockedPoint.distanceKm;
  }

  // Elevation & depression angles
  const rEff = kFactor * EARTH_RADIUS_M;
  const deltaH = uavAltitudeAslM - gcsTotalAslM;
  const elevAngleDeg =
    totalDistanceM > 0
      ? toDegrees(Math.atan(deltaH / totalDistanceM - totalDistanceM / (2 * rEff)))
      : 0;
  const depressionAngleDeg =
    totalDistanceM > 0
      ? toDegrees(Math.atan(deltaH / totalDistanceM + totalDistanceM / (2 * rEff)))
      : 0;

  // Slant range & FSPL
  const slantRangeKm = Math.sqrt(
    totalDistanceKm * totalDistanceKm + Math.pow(deltaH / 1000, 2)
  );
  const fsplDb = calculateFsplDb(slantRangeKm, freqMHz);
  const maxFresnelRadiusM = firstFresnelRadiusM(
    totalDistanceM / 2,
    totalDistanceM,
    freqMHz
  );
  const totalCurvatureDropAtUavM = earthCurvatureDropM(totalDistanceM, kFactor);
  const maxChordBulgeM = earthCurvatureBulgeM(
    totalDistanceM / 2,
    totalDistanceM,
    kFactor
  );
  const smoothHorizonKm = theoreticalHorizonKm(
    gcsTotalAslM,
    uavAltitudeAslM,
    kFactor
  );
  const smoothHorizonOpticalKm = theoreticalHorizonKm(
    gcsTotalAslM,
    uavAltitudeAslM,
    1.0
  );
  const smoothHorizonRfKm = theoreticalHorizonKm(
    gcsTotalAslM,
    uavAltitudeAslM,
    4 / 3
  );

  return {
    status, // 'CLEAR' | 'WARNING' | 'BLOCKED'
    isHardBlocked,
    isSoftBlocked,
    totalDistanceKm,
    totalDistanceNm: totalDistanceKm * KM_TO_NM,
    slantRangeKm,
    gcsGroundM,
    gcsAntennaAglM,
    gcsTotalAslM,
    uavGroundM,
    uavAltitudeAslM,
    uavAglM,
    kFactor,
    effectiveEarthRadiusKm: (kFactor * EARTH_RADIUS_KM),
    minDirectClearanceM,
    minEffectiveClearanceM,
    minRequiredUavAslM: Math.max(uavGroundM + safetyMarginM, maxRequiredUavAslM),
    minRequiredUavFt: metersToFeet(
      Math.max(uavGroundM + safetyMarginM, maxRequiredUavAslM)
    ),
    criticalPoint,
    firstBlockedPoint,
    maxUnobstructedRangeKm,
    elevAngleDeg,
    depressionAngleDeg,
    fsplDb,
    maxFresnelRadiusM,
    totalCurvatureDropAtUavM,
    maxChordBulgeM,
    smoothHorizonKm,
    smoothHorizonOpticalKm,
    smoothHorizonRfKm,
    profilePoints
  };
}

/**
 * Generate 0-200 km Horizon & Minimum Required UAV Altitude curves
 * Comparing:
 * 1) Actual Terrain + Selected Curvature (up to current radial data)
 * 2) Smooth Earth RF (k = 4/3) from 0 to 200 km
 * 3) Smooth Earth Optical (k = 1.0) from 0 to 200 km
 * 4) Smooth Earth Current k-factor from 0 to 200 km
 */
export function computeRangeHorizonCurves({
  gcsTotalAslM,
  kFactor = 4 / 3,
  safetyMarginM = 0,
  radialTerrainSamples = []
}) {
  const curvePoints = [];
  const stepKm = 2; // 0 to 200 km in 2 km steps (101 points)

  // Helper for smooth Earth min required UAV altitude (m ASL) at range D_km
  const smoothMinAltM = (dKm, k) => {
    const dM = dKm * 1000;
    const rEff = k * EARTH_RADIUS_M;
    const hGcs = Math.max(0, gcsTotalAslM - safetyMarginM);
    const dGcsHorizonM = Math.sqrt(2 * rEff * hGcs);
    if (dM <= dGcsHorizonM) {
      return safetyMarginM;
    }
    const remM = dM - dGcsHorizonM;
    return (remM * remM) / (2 * rEff) + safetyMarginM;
  };

  // Track max elevation angle (slope) seen along actual terrain radial so far
  for (let dKm = 0; dKm <= MAX_RANGE_KM; dKm += stepKm) {
    const rfSmoothM = smoothMinAltM(dKm, 4 / 3);
    const opticalSmoothM = smoothMinAltM(dKm, 1.0);
    const currentKSmoothM = smoothMinAltM(dKm, kFactor);

    let terrainRequiredM = null;
    if (radialTerrainSamples && radialTerrainSamples.length >= 2 && dKm > 0) {
      const maxSampleDistKm =
        radialTerrainSamples[radialTerrainSamples.length - 1].distanceKm;
      if (dKm <= maxSampleDistKm + 0.5) {
        const dM = dKm * 1000;
        let reqM = safetyMarginM;
        for (let i = 1; i < radialTerrainSamples.length; i++) {
          const s = radialTerrainSamples[i];
          if (s.distanceKm > dKm) break;
          const xM = s.distanceKm * 1000;
          if (xM <= 0) continue;
          const bulge = earthCurvatureBulgeM(xM, dM, kFactor);
          const obsM =
            (s.elevationM || 0) + (s.manualObstacleM || 0) + bulge + safetyMarginM;
          const reqAtD = gcsTotalAslM + (dM / xM) * (obsM - gcsTotalAslM);
          if (reqAtD > reqM) reqM = reqAtD;
        }
        terrainRequiredM = reqM;
      }
    }

    curvePoints.push({
      distanceKm: dKm,
      rfSmoothM,
      rfSmoothFt: metersToFeet(rfSmoothM),
      opticalSmoothM,
      opticalSmoothFt: metersToFeet(opticalSmoothM),
      currentKSmoothM,
      currentKSmoothFt: metersToFeet(currentKSmoothM),
      terrainRequiredM,
      terrainRequiredFt:
        terrainRequiredM !== null ? metersToFeet(terrainRequiredM) : null
    });
  }

  return curvePoints;
}
