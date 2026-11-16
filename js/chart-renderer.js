/**
 * UAV LOS Calculator - High-DPI Tactical Elevation Profile & 0-200 km Horizon Chart Renderer
 */

import { metersToFeet } from './los-math.js';

export class LosChartRenderer {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onHoverSample = options.onHoverSample || null;
    this.onClickSample = options.onClickSample || null;

    this.mode = 'profile'; // 'profile' | 'horizon200'
    this.altUnit = 'ft'; // 'ft' | 'm'
    this.analysis = null;
    this.horizonCurves = null;
    this.hoverIndex = -1;

    this._bindEvents();
  }

  setMode(mode) {
    this.mode = mode;
    this.hoverIndex = -1;
    this.render();
  }

  setAltUnit(unit) {
    this.altUnit = unit === 'm' ? 'm' : 'ft';
    this.render();
  }

  updateData(analysis, horizonCurves) {
    this.analysis = analysis;
    this.horizonCurves = horizonCurves;
    this.render();
  }

  _bindEvents() {
    const getRelPos = (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;
      return {
        x: clientX - rect.left,
        y: clientY - rect.top,
        width: rect.width,
        height: rect.height
      };
    };

    this.canvas.addEventListener('mousemove', (e) => {
      const pos = getRelPos(e);
      this._handlePointerMove(pos);
    });

    this.canvas.addEventListener('mouseleave', () => {
      this.hoverIndex = -1;
      if (this.onHoverSample) this.onHoverSample(null);
      this.render();
    });

    this.canvas.addEventListener('click', (e) => {
      if (this.mode !== 'profile' || !this.analysis) return;
      const pos = getRelPos(e);
      const idx = this._xToSampleIndex(pos.x, pos.width);
      if (idx >= 0 && this.onClickSample) {
        this.onClickSample(this.analysis.profilePoints[idx]);
      }
    });
  }

  _getPadding() {
    return { top: 26, right: 24, bottom: 32, left: 64 };
  }

  _xToSampleIndex(xPx, widthPx) {
    const pad = this._getPadding();
    const plotW = widthPx - pad.left - pad.right;
    if (plotW <= 10 || xPx < pad.left || xPx > widthPx - pad.right) return -1;
    const ratio = Math.max(0, Math.min(1, (xPx - pad.left) / plotW));
    if (this.mode === 'profile' && this.analysis) {
      const pts = this.analysis.profilePoints;
      return Math.round(ratio * (pts.length - 1));
    }
    if (this.mode === 'horizon200' && this.horizonCurves) {
      return Math.round(ratio * (this.horizonCurves.length - 1));
    }
    return -1;
  }

  _handlePointerMove(pos) {
    const idx = this._xToSampleIndex(pos.x, pos.width);
    if (idx !== this.hoverIndex) {
      this.hoverIndex = idx;
      if (this.mode === 'profile' && this.analysis && this.onHoverSample) {
        const pt = idx >= 0 ? this.analysis.profilePoints[idx] : null;
        this.onHoverSample(pt);
      }
      this.render();
    }
  }

  _formatAlt(meters) {
    if (this.altUnit === 'ft') {
      return `${Math.round(metersToFeet(meters)).toLocaleString('en-US')} ft`;
    }
    return `${Math.round(meters).toLocaleString('en-US')} מ'`;
  }

  _convertAlt(meters) {
    return this.altUnit === 'ft' ? metersToFeet(meters) : meters;
  }

  render() {
    const rect = this.canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;

    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(rect.width * dpr);
    const targetH = Math.round(rect.height * dpr);
    if (this.canvas.width !== targetW || this.canvas.height !== targetH) {
      this.canvas.width = targetW;
      this.canvas.height = targetH;
    }

    const ctx = this.ctx;
    ctx.save();
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);

    if (this.mode === 'horizon200') {
      this._renderHorizon200(ctx, w, h);
    } else {
      this._renderProfile(ctx, w, h);
    }

    ctx.restore();
  }

  _renderProfile(ctx, w, h) {
    if (!this.analysis || !this.analysis.profilePoints.length) return;

    const pad = this._getPadding();
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    if (plotW <= 20 || plotH <= 20) return;

    const pts = this.analysis.profilePoints;
    const maxDistKm = Math.max(0.1, this.analysis.totalDistanceKm);

    // Determine Y bounds in meters
    let minM = 0;
    let maxM = Math.max(
      this.analysis.gcsTotalAslM,
      this.analysis.uavAltitudeAslM,
      100
    );

    for (const p of pts) {
      if (p.terrainAslM < minM) minM = p.terrainAslM;
      const topTerrain = p.effectiveTerrainM + p.safetyMarginM;
      const topRay = p.losBeamM + p.requiredFresnelM;
      if (topTerrain > maxM) maxM = topTerrain;
      if (topRay > maxM) maxM = topRay;
    }

    const spanM = Math.max(100, maxM - minM);
    minM = Math.floor((minM - spanM * 0.08) / 50) * 50;
    maxM = Math.ceil((maxM + spanM * 0.14) / 50) * 50;

    const xToPx = (distKm) => pad.left + (distKm / maxDistKm) * plotW;
    const yToPx = (altM) =>
      pad.top + plotH - ((altM - minM) / (maxM - minM)) * plotH;

    // 1. Background Grid
    this._drawGrid(ctx, w, h, pad, plotW, plotH, minM, maxM, maxDistKm, xToPx, yToPx);

    // 2. Earth Curvature Bulge Layer (0 to p.bulgeM above minM/sea-level reference)
    ctx.beginPath();
    ctx.moveTo(xToPx(0), yToPx(minM));
    for (const p of pts) {
      ctx.lineTo(xToPx(p.distanceKm), yToPx(Math.max(minM, p.bulgeM)));
    }
    ctx.lineTo(xToPx(maxDistKm), yToPx(minM));
    ctx.closePath();
    const bulgeGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    bulgeGrad.addColorStop(0, 'rgba(14, 165, 233, 0.22)');
    bulgeGrad.addColorStop(1, 'rgba(14, 165, 233, 0.04)');
    ctx.fillStyle = bulgeGrad;
    ctx.fill();

    // Dashed Earth bulge curve line
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const px = xToPx(pts[i].distanceKm);
      const py = yToPx(pts[i].bulgeM);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
    ctx.lineWidth = 1.2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Terrain + Curvature Polygon
    ctx.beginPath();
    ctx.moveTo(xToPx(pts[0].distanceKm), yToPx(minM));
    for (const p of pts) {
      ctx.lineTo(xToPx(p.distanceKm), yToPx(p.effectiveTerrainM));
    }
    ctx.lineTo(xToPx(pts[pts.length - 1].distanceKm), yToPx(minM));
    ctx.closePath();

    const terrainGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    terrainGrad.addColorStop(0, 'rgba(71, 85, 105, 0.72)');
    terrainGrad.addColorStop(0.6, 'rgba(30, 41, 59, 0.82)');
    terrainGrad.addColorStop(1, 'rgba(15, 23, 42, 0.92)');
    ctx.fillStyle = terrainGrad;
    ctx.fill();

    // Terrain top stroke
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const px = xToPx(pts[i].distanceKm);
      const py = yToPx(pts[i].effectiveTerrainM);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.8;
    ctx.stroke();

    // 4. First Fresnel Zone Envelope (if active)
    const hasFresnel = pts.some((p) => p.requiredFresnelM > 0.2);
    if (hasFresnel) {
      ctx.beginPath();
      for (let i = 0; i < pts.length; i++) {
        const px = xToPx(pts[i].distanceKm);
        const py = yToPx(pts[i].losBeamM + pts[i].requiredFresnelM);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      for (let i = pts.length - 1; i >= 0; i--) {
        const px = xToPx(pts[i].distanceKm);
        const py = yToPx(pts[i].losBeamM - pts[i].requiredFresnelM);
        ctx.lineTo(px, py);
      }
      ctx.closePath();
      ctx.fillStyle =
        this.analysis.status === 'WARNING'
          ? 'rgba(245, 158, 11, 0.14)'
          : 'rgba(14, 165, 233, 0.11)';
      ctx.fill();

      // Lower Fresnel clearance boundary
      ctx.beginPath();
      for (let i = 0; i < pts.length; i++) {
        const px = xToPx(pts[i].distanceKm);
        const py = yToPx(pts[i].losLowerEnvelopeM);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.strokeStyle =
        this.analysis.status === 'WARNING'
          ? 'rgba(245, 158, 11, 0.7)'
          : 'rgba(56, 189, 248, 0.45)';
      ctx.lineWidth = 1.1;
      ctx.setLineDash([5, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 5. If blocked, draw dashed Minimum Required LOS Reference Line
    if (this.analysis.status !== 'CLEAR') {
      ctx.beginPath();
      ctx.moveTo(xToPx(0), yToPx(this.analysis.gcsTotalAslM));
      ctx.lineTo(
        xToPx(maxDistKm),
        yToPx(Math.min(maxM, this.analysis.minRequiredUavAslM))
      );
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.65)';
      ctx.lineWidth = 1.3;
      ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 6. Main Direct LOS Ray (split at critical point if blocked!)
    const gcsX = xToPx(0);
    const gcsY = yToPx(this.analysis.gcsTotalAslM);
    const uavX = xToPx(maxDistKm);
    const uavY = yToPx(this.analysis.uavAltitudeAslM);

    if (this.analysis.isHardBlocked && this.analysis.criticalPoint) {
      const cp = this.analysis.criticalPoint;
      const cpX = xToPx(cp.distanceKm);
      const cpY = yToPx(cp.losBeamM);

      // Clear segment from GCS to obstacle
      ctx.beginPath();
      ctx.moveTo(gcsX, gcsY);
      ctx.lineTo(cpX, cpY);
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // Blocked shadow segment from obstacle to UAV
      ctx.beginPath();
      ctx.moveTo(cpX, cpY);
      ctx.lineTo(uavX, uavY);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2.5;
      ctx.setLineDash([7, 5]);
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      ctx.beginPath();
      ctx.moveTo(gcsX, gcsY);
      ctx.lineTo(uavX, uavY);
      ctx.strokeStyle =
        this.analysis.status === 'WARNING' ? '#f59e0b' : '#10b981';
      ctx.lineWidth = 2.6;
      ctx.stroke();
    }

    // 7. Critical Point Marker
    if (this.analysis.criticalPoint && this.analysis.criticalPoint.index > 0) {
      const cp = this.analysis.criticalPoint;
      const cpX = xToPx(cp.distanceKm);
      const cpTerrY = yToPx(cp.effectiveTerrainM);
      const cpRayY = yToPx(cp.losBeamM);

      // Vertical clearance bar at critical point
      ctx.beginPath();
      ctx.moveTo(cpX, cpTerrY);
      ctx.lineTo(cpX, cpRayY);
      ctx.strokeStyle =
        cp.effectiveClearanceM < 0 ? '#ef4444' : '#38bdf8';
      ctx.lineWidth = 1.6;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(cpX, cpTerrY, 4.5, 0, Math.PI * 2);
      ctx.fillStyle =
        cp.effectiveClearanceM < 0 ? '#ef4444' : '#f59e0b';
      ctx.fill();
      ctx.strokeStyle = '#0b1118';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // 8. Endpoint Markers (GCS & UAV)
    // GCS mast vertical line
    const gcsGroundY = yToPx(this.analysis.gcsGroundM);
    ctx.beginPath();
    ctx.moveTo(gcsX, gcsGroundY);
    ctx.lineTo(gcsX, gcsY);
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(gcsX, gcsY, 5.5, 0, Math.PI * 2);
    ctx.fillStyle = '#0ea5e9';
    ctx.fill();
    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // UAV endpoint dot
    ctx.beginPath();
    ctx.arc(uavX, uavY, 6, 0, Math.PI * 2);
    ctx.fillStyle =
      this.analysis.status === 'BLOCKED'
        ? '#ef4444'
        : this.analysis.status === 'WARNING'
        ? '#f59e0b'
        : '#10b981';
    ctx.fill();
    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 1.8;
    ctx.stroke();

    // 9. Interactive Hover Crosshair & Tooltip
    if (this.hoverIndex >= 0 && this.hoverIndex < pts.length) {
      const hp = pts[this.hoverIndex];
      const hx = xToPx(hp.distanceKm);
      const hyRay = yToPx(hp.losBeamM);
      const hyTerr = yToPx(hp.effectiveTerrainM);

      ctx.beginPath();
      ctx.moveTo(hx, pad.top);
      ctx.lineTo(hx, pad.top + plotH);
      ctx.strokeStyle = 'rgba(226, 232, 240, 0.55)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.beginPath();
      ctx.arc(hx, hyRay, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = '#38bdf8';
      ctx.fill();

      ctx.beginPath();
      ctx.arc(hx, hyTerr, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = '#e2e8f0';
      ctx.fill();

      this._drawProfileTooltip(ctx, w, pad, hx, hp);
    }
  }

  _renderHorizon200(ctx, w, h) {
    if (!this.horizonCurves || !this.horizonCurves.length) return;

    const pad = this._getPadding();
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    if (plotW <= 20 || plotH <= 20) return;

    const maxDistKm = 200;
    const minM = 0;
    let maxM = 4500; // Default ~15,000 ft
    for (const c of this.horizonCurves) {
      if (c.opticalSmoothM > maxM) maxM = c.opticalSmoothM;
      if (c.terrainRequiredM !== null && c.terrainRequiredM > maxM) {
        maxM = Math.min(12000, c.terrainRequiredM);
      }
    }
    if (this.analysis && this.analysis.uavAltitudeAslM > maxM) {
      maxM = this.analysis.uavAltitudeAslM * 1.1;
    }
    maxM = Math.ceil(maxM / 500) * 500;

    const xToPx = (distKm) => pad.left + (distKm / maxDistKm) * plotW;
    const yToPx = (altM) =>
      pad.top + plotH - ((altM - minM) / (maxM - minM)) * plotH;

    this._drawGrid(ctx, w, h, pad, plotW, plotH, minM, maxM, maxDistKm, xToPx, yToPx);

    // 1. Optical Smooth Earth Curve (k = 1.0)
    ctx.beginPath();
    this.horizonCurves.forEach((c, idx) => {
      const px = xToPx(c.distanceKm);
      const py = yToPx(c.opticalSmoothM);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. RF Smooth Earth Curve (k = 4/3)
    ctx.beginPath();
    this.horizonCurves.forEach((c, idx) => {
      const px = xToPx(c.distanceKm);
      const py = yToPx(c.rfSmoothM);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth = 2.2;
    ctx.stroke();

    // 3. Actual Terrain + Selected k Curve (up to current UAV range)
    const terrainPts = this.horizonCurves.filter((c) => c.terrainRequiredM !== null);
    if (terrainPts.length > 1) {
      ctx.beginPath();
      terrainPts.forEach((c, idx) => {
        const px = xToPx(c.distanceKm);
        const py = yToPx(Math.min(maxM, c.terrainRequiredM));
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2.6;
      ctx.stroke();
    }

    // 4. Current UAV Position Marker on 0-200 km chart
    if (this.analysis) {
      const ux = xToPx(Math.min(200, this.analysis.totalDistanceKm));
      const uy = yToPx(Math.min(maxM, this.analysis.uavAltitudeAslM));

      ctx.beginPath();
      ctx.arc(ux, uy, 6.5, 0, Math.PI * 2);
      ctx.fillStyle =
        this.analysis.status === 'CLEAR' ? '#10b981' : '#ef4444';
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // 5. Hover Tooltip on 0-200 km Horizon Chart
    if (this.hoverIndex >= 0 && this.hoverIndex < this.horizonCurves.length) {
      const hc = this.horizonCurves[this.hoverIndex];
      const hx = xToPx(hc.distanceKm);

      ctx.beginPath();
      ctx.moveTo(hx, pad.top);
      ctx.lineTo(hx, pad.top + plotH);
      ctx.strokeStyle = 'rgba(226, 232, 240, 0.5)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      this._drawHorizonTooltip(ctx, w, pad, hx, hc);
    }
  }

  _drawGrid(ctx, w, h, pad, plotW, plotH, minM, maxM, maxDistKm, xToPx, yToPx) {
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const yTicks = 4;
    for (let i = 0; i <= yTicks; i++) {
      const valM = minM + (i / yTicks) * (maxM - minM);
      const py = yToPx(valM);

      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.14)';
      ctx.lineWidth = 1;
      ctx.stroke();

      const dispVal = Math.round(this._convertAlt(valM)).toLocaleString('en-US');
      ctx.fillStyle = '#94a3b8';
      ctx.fillText(
        `${dispVal}${this.altUnit === 'ft' ? "'" : 'm'}`,
        pad.left - 8,
        py
      );
    }

    const xTicks = Math.min(8, Math.max(4, Math.floor(plotW / 95)));
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let i = 0; i <= xTicks; i++) {
      const distKm = (i / xTicks) * maxDistKm;
      const px = xToPx(distKm);

      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.11)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(
        `${distKm >= 10 ? distKm.toFixed(0) : distKm.toFixed(1)} ק"מ`,
        px,
        pad.top + plotH + 8
      );
    }
  }

  _drawProfileTooltip(ctx, w, pad, hx, hp) {
    const boxW = 205;
    const boxH = 74;
    const bx = hx + 14 + boxW > w - pad.right ? hx - boxW - 14 : hx + 14;
    const by = pad.top + 6;

    ctx.fillStyle = 'rgba(11, 17, 24, 0.92)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(bx, by, boxW, boxH, 6);
    ctx.fill();
    ctx.stroke();

    ctx.font = '11.5px "Heebo", sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    const rx = bx + boxW - 10;

    ctx.fillStyle = '#f8fafc';
    ctx.fillText(
      `טווח: ${hp.distanceKm.toFixed(2)} ק"מ | קרקע: ${this._formatAlt(hp.rawSurfaceM)}`,
      rx,
      by + 8
    );
    ctx.fillStyle = '#38bdf8';
    ctx.fillText(
      `קימור כדה"א: +${Math.round(hp.bulgeM)} מ' | קרן: ${this._formatAlt(hp.losBeamM)}`,
      rx,
      by + 28
    );
    ctx.fillStyle = hp.effectiveClearanceM >= 0 ? '#10b981' : '#ef4444';
    const sign = hp.effectiveClearanceM >= 0 ? '+' : '';
    ctx.fillText(
      `מרווח נטו: ${sign}${Math.round(hp.effectiveClearanceM)} מ' (${this._formatAlt(hp.effectiveClearanceM)})`,
      rx,
      by + 48
    );
  }

  _drawHorizonTooltip(ctx, w, pad, hx, hc) {
    const boxW = 215;
    const boxH = 74;
    const bx = hx + 14 + boxW > w - pad.right ? hx - boxW - 14 : hx + 14;
    const by = pad.top + 6;

    ctx.fillStyle = 'rgba(11, 17, 24, 0.92)';
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(bx, by, boxW, boxH, 6);
    ctx.fill();
    ctx.stroke();

    ctx.font = '11.5px "Heebo", sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    const rx = bx + boxW - 10;

    ctx.fillStyle = '#f8fafc';
    ctx.fillText(`טווח מהתחנה: ${hc.distanceKm} ק"מ`, rx, by + 8);
    ctx.fillStyle = '#0ea5e9';
    ctx.fillText(
      `מינ' כדה"א חלק RF (4/3): ${this._formatAlt(hc.rfSmoothM)}`,
      rx,
      by + 28
    );
    ctx.fillStyle = '#f59e0b';
    ctx.fillText(
      `מינ' אופטי (1.0): ${this._formatAlt(hc.opticalSmoothM)}`,
      rx,
      by + 48
    );
  }
}
