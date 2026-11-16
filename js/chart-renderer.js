/**
 * UAV LOS Calculator - High-DPI Terrain Cross-Section & Blocking Ridge Renderer
 * Visualizes along any hovered/selected azimuth ray:
 * - Curved-Earth Terrain Cross-Section: y(d) = h(d) - d^2 / (2 * Re)
 * - Aircraft Constant-Altitude Trajectory Curve: y_UAV(d) = H_UAV - d^2 / (2 * Re)
 * - Straight Tangent LOS Ray from Station (0, H0) grazing the Blocking Ridge to r_max
 * - Shadow Zone beyond r_max
 */

const M_TO_FT = 3.280839895;

export class LosChartRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.profile = null;
    this.hoverIdx = -1;

    this._bindEvents();
  }

  setProfile(profile) {
    this.profile = profile;
    this.render();
  }

  _bindEvents() {
    this.canvas.addEventListener('mousemove', (e) => {
      if (!this.profile || !this.profile.profileSamples.length) return;
      const rect = this.canvas.getBoundingClientRect();
      const pad = this._getPadding();
      const plotW = rect.width - pad.left - pad.right;
      const relX = e.clientX - rect.left - pad.left;
      if (plotW <= 10 || relX < 0 || relX > plotW) {
        this.hoverIdx = -1;
      } else {
        const ratio = Math.max(0, Math.min(1, relX / plotW));
        this.hoverIdx = Math.round(ratio * (this.profile.profileSamples.length - 1));
      }
      this.render();
    });

    this.canvas.addEventListener('mouseleave', () => {
      this.hoverIdx = -1;
      this.render();
    });
  }

  _getPadding() {
    return { top: 22, right: 20, bottom: 26, left: 58 };
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

    if (!this.profile || !this.profile.profileSamples || !this.profile.profileSamples.length) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '13px "Heebo", sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('רחף או לחץ על המפה סביב התחנה להצגת חתך גבהים והרכס החוסם בכיוון הנבחר', w / 2, h / 2);
      ctx.restore();
      return;
    }

    const p = this.profile;
    const samples = p.profileSamples;
    const pad = this._getPadding();
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    if (plotW <= 20 || plotH <= 20) {
      ctx.restore();
      return;
    }

    const maxDistKm = Math.max(1, p.maxRangeKm);
    const inv2Re = 1.0 / (2.0 * p.Re);

    // Compute Y range in tangent-plane effective height y = h - d^2/(2*Re)
    let minY = Math.min(0, p.H0);
    let maxY = Math.max(p.H0 + 200, p.hUavM + 200);

    for (const s of samples) {
      if (s.effectiveHeightM < minY) minY = s.effectiveHeightM;
      if (s.terrainAslM > maxY) maxY = s.terrainAslM;
    }
    const ySpan = Math.max(200, maxY - minY);
    minY -= ySpan * 0.08;
    maxY += ySpan * 0.12;

    const xToPx = (dKm) => pad.left + (dKm / maxDistKm) * plotW;
    const yToPx = (yM) => pad.top + plotH - ((yM - minY) / (maxY - minY)) * plotH;

    // 1. Grid lines
    ctx.font = '10.5px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const yTicks = 4;
    for (let i = 0; i <= yTicks; i++) {
      const valM = minY + (i / yTicks) * (maxY - minY);
      const py = yToPx(valM);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.13)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${Math.round(valM)}m`, pad.left - 6, py);
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xTicks = 5;
    for (let i = 0; i <= xTicks; i++) {
      const dKm = (i / xTicks) * maxDistKm;
      const px = xToPx(dKm);
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.1)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${Math.round(dKm)} ק"מ`, px, pad.top + plotH + 6);
    }

    // 2. Shadow zone shading beyond rMaxKm
    if (p.rMaxKm < maxDistKm - 0.2) {
      const rMaxPx = xToPx(p.rMaxKm);
      ctx.fillStyle = 'rgba(239, 68, 68, 0.10)';
      ctx.fillRect(rMaxPx, pad.top, pad.left + plotW - rMaxPx, plotH);
    }

    // 3. Curved Terrain Polygon
    ctx.beginPath();
    ctx.moveTo(xToPx(0), yToPx(minY));
    for (const s of samples) {
      ctx.lineTo(xToPx(s.distanceKm), yToPx(s.effectiveHeightM));
    }
    ctx.lineTo(xToPx(maxDistKm), yToPx(minY));
    ctx.closePath();
    const terrGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    terrGrad.addColorStop(0, 'rgba(71, 85, 105, 0.78)');
    terrGrad.addColorStop(1, 'rgba(15, 23, 42, 0.94)');
    ctx.fillStyle = terrGrad;
    ctx.fill();

    ctx.beginPath();
    samples.forEach((s, idx) => {
      const px = xToPx(s.distanceKm);
      const py = yToPx(s.effectiveHeightM);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.6;
    ctx.stroke();

    // 4. Aircraft Constant-Altitude Curve: y_UAV(d) = H_UAV - d^2 / (2 * Re)
    ctx.beginPath();
    samples.forEach((s, idx) => {
      const dM = s.distanceKm * 1000.0;
      const yUav = p.hUavM - dM * dM * inv2Re;
      const px = xToPx(s.distanceKm);
      const py = yToPx(yUav);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.65)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 5. Direct LOS Tangent Ray from Station (0, H0) to (rMaxKm, yUavAtRMax)
    const rMaxM = p.rMaxKm * 1000.0;
    const yUavAtRMax = p.hUavM - rMaxM * rMaxM * inv2Re;
    const gcsX = xToPx(0);
    const gcsY = yToPx(p.H0);
    const endX = xToPx(p.rMaxKm);
    const endY = yToPx(yUavAtRMax);

    ctx.beginPath();
    ctx.moveTo(gcsX, gcsY);
    ctx.lineTo(endX, endY);
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2.4;
    ctx.stroke();

    // 6. Blocking Ridge Marker
    if (p.blockingDistKm > 0.05) {
      const bDistM = p.blockingDistKm * 1000.0;
      const bEffY = p.blockingHeightM - bDistM * bDistM * inv2Re;
      const bx = xToPx(p.blockingDistKm);
      const by = yToPx(bEffY);

      ctx.beginPath();
      ctx.arc(bx, by, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#f59e0b';
      ctx.fill();
      ctx.strokeStyle = '#0b1118';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // 7. Station & Max-Range UAV Endpoint Dots
    ctx.beginPath();
    ctx.arc(gcsX, gcsY, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#0ea5e9';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(endX, endY, 5.5, 0, Math.PI * 2);
    ctx.fillStyle = '#10b981';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // 8. Hover Crosshair Tooltip
    if (this.hoverIdx >= 0 && this.hoverIdx < samples.length) {
      const hs = samples[this.hoverIdx];
      const hx = xToPx(hs.distanceKm);
      ctx.beginPath();
      ctx.moveTo(hx, pad.top);
      ctx.lineTo(hx, pad.top + plotH);
      ctx.strokeStyle = 'rgba(226, 232, 240, 0.5)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      const boxW = 200;
      const boxH = 54;
      const bx = hx + 12 + boxW > w - pad.right ? hx - boxW - 12 : hx + 12;
      const by = pad.top + 4;
      ctx.fillStyle = 'rgba(11, 17, 24, 0.92)';
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
      ctx.beginPath();
      ctx.roundRect(bx, by, boxW, boxH, 6);
      ctx.fill();
      ctx.stroke();

      ctx.font = '11px "Heebo", sans-serif';
      ctx.textAlign = 'right';
      ctx.fillStyle = '#f8fafc';
      ctx.fillText(
        `מרחק: ${hs.distanceKm.toFixed(1)} ק"מ | גובה קרקע: ${hs.terrainAslM} מ'`,
        bx + boxW - 8,
        by + 8
      );
      ctx.fillStyle = '#38bdf8';
      ctx.fillText(
        `שקיעת עקמומיות: -${Math.round(hs.curvatureDropM)} מ' (${Math.round(
          hs.terrainAslM * M_TO_FT
        )} ft)`,
        bx + boxW - 8,
        by + 28
      );
    }

    ctx.restore();
  }
}
