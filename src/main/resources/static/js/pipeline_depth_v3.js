// pipeline_depth_v3.js
// 管线探测模块地下电缆埋深 V3：连续分段标定 + 因果 Q25 + 012置信度 + 沿电缆里程状态估计
(function (global) {
    'use strict';

    const CFG = {
        DEPTH_MIN_M: 0.05,
        DEPTH_MAX_M: 5.0,
        SIGMA_SIGNAL_ZERO: 0.25,
        SIGMA_CROSSING: 0.40,
        SIGMA_SIDE: 0.65,
        SIGMA_UNKNOWN: 0.80,
        SIGMA_Q25: 0.36,
        SLOPE_RANDOM_WALK_Q: 0.0025,
        INNOVATION_GATE_SIGMA: 3.0,
        MAX_R_INFLATION: 100.0,
        CROSSING_LOOKBACK_MS: 2500,
        SIGNAL_MATCH_MAX_GAP_MS: 500,
        PATH_TIME_MATCH_MAX_GAP_MS: 3000,
        RAW_HISTORY_SIZE: 4
    };

    const SINGLE_MODEL = {
        knotsCm: [200.0, 500.0],
        c: 0.64546983,
        s1: 0.02851825,
        s2: 1.19981675,
        s3: 0.04947191
    };

    const Q25_MODEL = {
        knotsCm: [160.0, 500.0],
        c: 0.59700438,
        s1: 0.09464048,
        s2: 1.03192798,
        s3: 0.29409459
    };

    function finiteNumber(v) {
        const n = Number(v);
        return Number.isFinite(n) ? n : null;
    }

    function clamp(v, lo, hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    function calibratePiecewiseLog(rawCm, model) {
        const r = finiteNumber(rawCm);
        if (r === null || r <= 0) return null;
        const k1 = Math.log(model.knotsCm[0]);
        const k2 = Math.log(model.knotsCm[1]);
        const x = Math.log(r);
        const z = model.c
            + model.s1 * Math.min(x - k1, 0)
            + model.s2 * Math.min(Math.max(x - k1, 0), k2 - k1)
            + model.s3 * Math.max(x - k2, 0);
        return clamp(z, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M);
    }

    function quantileLinear(values, q) {
        const a = values.map(Number).filter(Number.isFinite).sort((x, y) => x - y);
        if (!a.length) return null;
        if (a.length === 1) return a[0];
        const pos = (a.length - 1) * q;
        const lo = Math.floor(pos);
        const hi = Math.ceil(pos);
        if (lo === hi) return a[lo];
        const w = pos - lo;
        return a[lo] * (1 - w) + a[hi] * w;
    }

    function cumulativeDistance(points) {
        const s = new Array(points.length).fill(0);
        for (let i = 1; i < points.length; i++) {
            const dx = Number(points[i].x) - Number(points[i - 1].x);
            const dy = Number(points[i].y) - Number(points[i - 1].y);
            const ds = Math.hypot(dx, dy);
            s[i] = s[i - 1] + (Number.isFinite(ds) ? ds : 0);
        }
        return s;
    }

    function predictState(state, ds) {
        const q = CFG.SLOPE_RANDOM_WALK_Q;
        const d = state.d + state.g * ds;
        const g = state.g;
        const p00 = state.p00 + ds * (state.p01 + state.p10) + ds * ds * state.p11 + q * ds * ds * ds / 3.0;
        const p01 = state.p01 + ds * state.p11 + q * ds * ds / 2.0;
        const p10 = state.p10 + ds * state.p11 + q * ds * ds / 2.0;
        const p11 = state.p11 + q * Math.max(ds, 1e-9);
        return { d: clamp(d, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M), g, p00, p01, p10, p11 };
    }

    function robustUpdate(state, z, sigma) {
        let R = sigma * sigma;
        const innovation = z - state.d;
        let S = state.p00 + R;
        const innovationSigma = Math.abs(innovation) / Math.max(Math.sqrt(S), 1e-9);
        let inflation = 1.0;
        if (innovationSigma > CFG.INNOVATION_GATE_SIGMA) {
            inflation = Math.min(
                CFG.MAX_R_INFLATION,
                Math.pow(innovationSigma / CFG.INNOVATION_GATE_SIGMA, 2)
            );
            R *= inflation;
            S = state.p00 + R;
        }

        const k0 = state.p00 / S;
        const k1 = state.p10 / S;
        const d = clamp(state.d + k0 * innovation, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M);
        const g = state.g + k1 * innovation;

        const a = 1 - k0;
        const b = -k1;
        const m00 = a * state.p00;
        const m01 = a * state.p01;
        const m10 = b * state.p00 + state.p10;
        const m11 = b * state.p01 + state.p11;
        let p00 = m00 * a + R * k0 * k0;
        let p01 = m00 * b + m01 + R * k0 * k1;
        let p10 = m10 * a + R * k1 * k0;
        let p11 = m10 * b + m11 + R * k1 * k1;
        const cross = 0.5 * (p01 + p10);
        p01 = cross;
        p10 = cross;
        p00 = Math.max(p00, 1e-9);
        p11 = Math.max(p11, 1e-9);
        return { state: { d, g, p00, p01, p10, p11 }, innovationSigma, inflation };
    }

    function cloneState(state) {
        return state ? {
            d: state.d, g: state.g,
            p00: state.p00, p01: state.p01,
            p10: state.p10, p11: state.p11
        } : null;
    }

    function invertSymmetric2x2(p00, p01, p11) {
        let det = p00 * p11 - p01 * p01;
        if (!Number.isFinite(det) || Math.abs(det) < 1e-12) {
            const jitter = 1e-6;
            p00 += jitter;
            p11 += jitter;
            det = p00 * p11 - p01 * p01;
        }
        if (!Number.isFinite(det) || Math.abs(det) < 1e-15) return null;
        return {
            a00: p11 / det,
            a01: -p01 / det,
            a10: -p01 / det,
            a11: p00 / det
        };
    }

    function rtsSmoothStep(filtered, predictedNext, smoothedNext, ds) {
        if (!filtered || !predictedNext || !smoothedNext) return cloneState(filtered);
        const pfFt00 = filtered.p00 + ds * filtered.p01;
        const pfFt01 = filtered.p01;
        const pfFt10 = filtered.p10 + ds * filtered.p11;
        const pfFt11 = filtered.p11;
        const inv = invertSymmetric2x2(
            predictedNext.p00,
            0.5 * (predictedNext.p01 + predictedNext.p10),
            predictedNext.p11
        );
        if (!inv) return cloneState(filtered);
        const c00 = pfFt00 * inv.a00 + pfFt01 * inv.a10;
        const c01 = pfFt00 * inv.a01 + pfFt01 * inv.a11;
        const c10 = pfFt10 * inv.a00 + pfFt11 * inv.a10;
        const c11 = pfFt10 * inv.a01 + pfFt11 * inv.a11;

        const dd = smoothedNext.d - predictedNext.d;
        const dg = smoothedNext.g - predictedNext.g;
        const d = clamp(filtered.d + c00 * dd + c01 * dg, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M);
        const g = filtered.g + c10 * dd + c11 * dg;

        const a00 = smoothedNext.p00 - predictedNext.p00;
        const a01 = smoothedNext.p01 - predictedNext.p01;
        const a10 = smoothedNext.p10 - predictedNext.p10;
        const a11 = smoothedNext.p11 - predictedNext.p11;
        const ca00 = c00 * a00 + c01 * a10;
        const ca01 = c00 * a01 + c01 * a11;
        const ca10 = c10 * a00 + c11 * a10;
        const ca11 = c10 * a01 + c11 * a11;
        let p00 = filtered.p00 + ca00 * c00 + ca01 * c01;
        let p01 = filtered.p01 + ca00 * c10 + ca01 * c11;
        let p10 = filtered.p10 + ca10 * c00 + ca11 * c01;
        let p11 = filtered.p11 + ca10 * c10 + ca11 * c11;
        const cross = 0.5 * (p01 + p10);
        p01 = cross;
        p10 = cross;
        p00 = Math.max(Number.isFinite(p00) ? p00 : filtered.p00, 1e-9);
        p11 = Math.max(Number.isFinite(p11) ? p11 : filtered.p11, 1e-9);
        return { d, g, p00, p01, p10, p11 };
    }

    function confidenceLabel(sigma, source) {
        if (source === 'measured_center_0' && sigma <= 0.35) return 'HIGH';
        if (sigma <= 0.35) return 'MEDIUM_HIGH';
        if (sigma <= 0.60) return 'MEDIUM';
        return 'LOW';
    }

    class PipelineDepthEstimatorV3 {
        constructor() {
            this.reset();
        }

        reset() {
            this.signalHistory = [];
            this.poseHistory = [];
            this.rawHistory = [];
            this.observations = [];
            this.lastSummary = null;
            this.lastFinalSummary = null;
            this._lastFilterRun = null;
            this.isFinalized = false;
        }

        addSignal(signal, timeMs) {
            const sig = Number(signal);
            if (![0, 1, 2].includes(sig)) return;
            const t = finiteNumber(timeMs) ?? Date.now();
            this.signalHistory.push({ t, sig });
            const keepAfter = t - 60000;
            if (this.signalHistory.length > 2000) {
                this.signalHistory = this.signalHistory.filter(e => e.t >= keepAfter);
            }
        }

        addPose(x, y, timeMs) {
            const px = finiteNumber(x);
            const py = finiteNumber(y);
            if (px === null || py === null) return;
            const t = finiteNumber(timeMs) ?? Date.now();
            this.poseHistory.push({ t, x: px, y: py });
            const keepAfter = t - 120000;
            if (this.poseHistory.length > 6000) {
                this.poseHistory = this.poseHistory.filter(e => e.t >= keepAfter);
            }
        }

        _poseAt(t) {
            const a = this.poseHistory;
            if (!a.length) return null;
            let lo = 0, hi = a.length;
            while (lo < hi) {
                const mid = (lo + hi) >> 1;
                if (a[mid].t < t) lo = mid + 1;
                else hi = mid;
            }
            if (lo === 0) {
                return Math.abs(a[0].t - t) <= CFG.SIGNAL_MATCH_MAX_GAP_MS
                    ? { x: a[0].x, y: a[0].y } : null;
            }
            if (lo >= a.length) {
                const e = a[a.length - 1];
                return Math.abs(e.t - t) <= CFG.SIGNAL_MATCH_MAX_GAP_MS
                    ? { x: e.x, y: e.y } : null;
            }
            const p0 = a[lo - 1], p1 = a[lo];
            if (t - p0.t > CFG.SIGNAL_MATCH_MAX_GAP_MS || p1.t - t > CFG.SIGNAL_MATCH_MAX_GAP_MS) return null;
            const dt = p1.t - p0.t;
            if (dt <= 0) return { x: p0.x, y: p0.y };
            const u = clamp((t - p0.t) / dt, 0, 1);
            return { x: p0.x + u * (p1.x - p0.x), y: p0.y + u * (p1.y - p0.y) };
        }

        _nearestPathTimeIndex(cablePoints, t) {
            const timed = [];
            for (let i = 0; i < cablePoints.length; i++) {
                const pt = finiteNumber(cablePoints[i]?.userData?.timestampMs);
                if (pt !== null) timed.push({ i, t: pt });
            }
            if (!timed.length) return { index: -1, gap: Infinity };
            let lo = 0, hi = timed.length;
            while (lo < hi) {
                const mid = (lo + hi) >> 1;
                if (timed[mid].t < t) lo = mid + 1;
                else hi = mid;
            }
            const cands = [];
            if (lo < timed.length) cands.push(timed[lo]);
            if (lo > 0) cands.push(timed[lo - 1]);
            let best = cands[0];
            for (const e of cands) if (Math.abs(e.t - t) < Math.abs(best.t - t)) best = e;
            return { index: best.i, gap: Math.abs(best.t - t) };
        }

        _projectPoseToPathIndex(pose, cablePoints) {
            if (!pose || cablePoints.length < 2) return null;
            let best = null;
            for (let i = 0; i < cablePoints.length - 1; i++) {
                const a = cablePoints[i], b = cablePoints[i + 1];
                const ax = finiteNumber(a.x), ay = finiteNumber(a.y);
                const bx = finiteNumber(b.x), by = finiteNumber(b.y);
                if ([ax, ay, bx, by].some(v => v === null)) continue;
                const vx = bx - ax, vy = by - ay;
                const len2 = vx * vx + vy * vy;
                let u = len2 > 1e-12 ? ((pose.x - ax) * vx + (pose.y - ay) * vy) / len2 : 0;
                u = clamp(u, 0, 1);
                const qx = ax + u * vx, qy = ay + u * vy;
                const dist = Math.hypot(pose.x - qx, pose.y - qy);
                if (!best || dist < best.dist) {
                    best = { index: u < 0.5 ? i : i + 1, dist, projectedX: qx, projectedY: qy };
                }
            }
            return best;
        }

        _captureTime(payload) {
            const server = finiteNumber(payload.serverReceiveTimeMs) ?? Date.now();
            const capture = finiteNumber(payload.captureTimeMs);
            const backend = finiteNumber(payload.backendReceiveTimeMs);
            if (capture !== null && backend !== null) {
                const delay = backend - capture;
                if (Number.isFinite(delay) && delay >= 0 && delay <= 10000) {
                    return server - delay;
                }
            }
            return server;
        }

        _signalContext(t) {
            let nearest = null;
            let nearestGap = Infinity;
            for (const e of this.signalHistory) {
                const gap = Math.abs(e.t - t);
                if (gap < nearestGap) {
                    nearestGap = gap;
                    nearest = e;
                }
            }
            const signal = nearest && nearestGap <= CFG.SIGNAL_MATCH_MAX_GAP_MS ? nearest.sig : null;
            if (signal === 0) return { context: 'center_0', signal, sigma: CFG.SIGMA_SIGNAL_ZERO };

            const recent = this.signalHistory.filter(e => e.t >= t - CFG.CROSSING_LOOKBACK_MS && e.t <= t);
            const comp = [];
            for (const e of recent) {
                if (!comp.length || comp[comp.length - 1] !== e.sig) comp.push(e.sig);
            }
            const crossing = comp.includes(0) || (comp.includes(1) && comp.includes(2));
            if (crossing) return { context: 'crossing_recent', signal, sigma: CFG.SIGMA_CROSSING };
            if (signal === 1 || signal === 2) return { context: 'side', signal, sigma: CFG.SIGMA_SIDE };
            return { context: 'unknown', signal, sigma: CFG.SIGMA_UNKNOWN };
        }

        ingestDepth(payload, cablePoints) {
            const raw = finiteNumber(payload.rawDepthCm ?? payload.depth);
            if (raw === null || raw <= 0) return null;

            const captureTimeMs = this._captureTime(payload);
            const ctx = this._signalContext(captureTimeMs);
            this.rawHistory.push(raw);
            if (this.rawHistory.length > CFG.RAW_HISTORY_SIZE) this.rawHistory.shift();

            let calibrated = calibratePiecewiseLog(raw, SINGLE_MODEL);
            let sigma = ctx.sigma;
            let channel = 'single_fast_segmented';
            let featureCm = raw;

            if (ctx.context !== 'center_0' && this.rawHistory.length >= CFG.RAW_HISTORY_SIZE) {
                const q25 = quantileLinear(this.rawHistory, 0.25);
                if (q25 !== null) {
                    featureCm = q25;
                    calibrated = calibratePiecewiseLog(q25, Q25_MODEL);
                    sigma = CFG.SIGMA_Q25;
                    channel = 'rolling_q25_4_segmented';
                }
            } else if (ctx.context === 'center_0') {
                channel = 'single_center_0_segmented';
            }

            const obs = {
                id: this.observations.length + 1,
                receiptTimeMs: finiteNumber(payload.serverReceiveTimeMs) ?? Date.now(),
                captureTimeMs,
                rawDepthCm: raw,
                featureCm,
                calibratedDepthM: calibrated,
                sigmaM: sigma,
                signal: ctx.signal,
                signalContext: ctx.context,
                observationChannel: channel,
                targetIndex: null,
                targetTimeGapMs: null
            };
            this.observations.push(obs);
            if (this.observations.length > 500) this.observations.shift();
            this.recompute(cablePoints || []);

            console.log('[V3深度] raw=', raw.toFixed(1), 'cm, feature=', featureCm.toFixed(1),
                'cm, calibrated=', calibrated.toFixed(3), 'm, context=', ctx.context,
                'channel=', channel, 'sigma=', sigma.toFixed(2));
            return obs;
        }

        _mapObservationsToPath(cablePoints) {
            if (!Array.isArray(cablePoints) || cablePoints.length < 2) return [];
            const mapped = [];
            for (const obs of this.observations) {
                const pose = this._poseAt(obs.captureTimeMs);
                const projected = this._projectPoseToPathIndex(pose, cablePoints);
                if (projected) {
                    obs.targetIndex = projected.index;
                    obs.targetTimeGapMs = null;
                    obs.robotX = pose.x;
                    obs.robotY = pose.y;
                    obs.lateralDistanceM = projected.dist;
                    obs.mappingMode = 'pose_projection';
                    mapped.push(obs);
                    continue;
                }

                const fallback = this._nearestPathTimeIndex(cablePoints, obs.captureTimeMs);
                if (fallback.index < 0 || fallback.gap > CFG.PATH_TIME_MATCH_MAX_GAP_MS) continue;
                obs.targetIndex = fallback.index;
                obs.targetTimeGapMs = fallback.gap;
                obs.mappingMode = 'path_time_fallback';
                mapped.push(obs);
            }
            return mapped;
        }

        recompute(cablePoints) {
            if (!Array.isArray(cablePoints) || cablePoints.length === 0) return null;

            const s = cumulativeDistance(cablePoints);
            for (let i = 0; i < cablePoints.length; i++) {
                const ud = cablePoints[i].userData || (cablePoints[i].userData = {});
                ud.cableS_M = s[i];
                ud.estimatedDepthM = null;
                ud.depthSigmaM = null;
                ud.depthConfidence = 'UNKNOWN';
                ud.depthSource = 'uninitialized';
                ud.rawDepthCm = null;
                ud.depthObservationChannel = null;
                ud.depthSignalContext = null;
                ud.onlineDepthM = null;
                ud.onlineDepthSigmaM = null;
                ud.finalDepthM = null;
                ud.finalDepthSigmaM = null;
                ud.finalDepthConfidence = null;
                ud.finalDepthSource = null;
            }
            this.isFinalized = false;
            this.lastFinalSummary = null;

            const mapped = this._mapObservationsToPath(cablePoints);
            if (!mapped.length) return null;
            const byNode = new Map();
            for (const obs of mapped) {
                if (!byNode.has(obs.targetIndex)) byNode.set(obs.targetIndex, []);
                byNode.get(obs.targetIndex).push(obs);
            }
            for (const arr of byNode.values()) arr.sort((a, b) => a.captureTimeMs - b.captureTimeMs);

            const indices = Array.from(byNode.keys()).sort((a, b) => a - b);
            const firstIdx = indices[0];
            const firstObs = byNode.get(firstIdx)[0];
            let state = {
                d: clamp(firstObs.calibratedDepthM, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M),
                g: 0,
                p00: Math.max(firstObs.sigmaM * firstObs.sigmaM, 0.04),
                p01: 0,
                p10: 0,
                p11: 0.08 * 0.08
            };
            const filteredStates = new Array(cablePoints.length).fill(null);
            const predictedStates = new Array(cablePoints.length).fill(null);

            for (let i = 0; i < firstIdx; i++) {
                const ud = cablePoints[i].userData;
                ud.estimatedDepthM = state.d;
                ud.depthSigmaM = 1.0;
                ud.depthConfidence = 'LOW';
                ud.depthSource = 'backfilled_before_first_measurement';
                ud.onlineDepthM = state.d;
                ud.onlineDepthSigmaM = 1.0;
                ud.depth = state.d;
            }

            for (let i = firstIdx; i < cablePoints.length; i++) {
                if (i > firstIdx) {
                    const ds = Math.max(0, s[i] - s[i - 1]);
                    state = predictState(state, ds);
                    predictedStates[i] = cloneState(state);
                } else {
                    predictedStates[i] = cloneState(state);
                }

                let source = 'predicted';
                let lastObs = i === firstIdx ? firstObs : null;
                let nodeObs = byNode.get(i) || [];
                const contexts = i === firstIdx ? [firstObs.signalContext] : [];
                if (i === firstIdx) nodeObs = nodeObs.filter(obs => obs !== firstObs);
                for (const obs of nodeObs) {
                    const result = robustUpdate(state, obs.calibratedDepthM, obs.sigmaM);
                    state = result.state;
                    lastObs = obs;
                    contexts.push(obs.signalContext);
                }
                if (contexts.includes('center_0')) source = 'measured_center_0';
                else if (contexts.includes('crossing_recent')) source = 'measured_crossing';
                else if (nodeObs.length) source = 'measured_side';

                const sigma = Math.sqrt(Math.max(state.p00, 0));
                const ud = cablePoints[i].userData;
                ud.estimatedDepthM = state.d;
                ud.depthSigmaM = sigma;
                ud.depthConfidence = confidenceLabel(sigma, source);
                ud.depthSource = source;
                ud.onlineDepthM = state.d;
                ud.onlineDepthSigmaM = sigma;
                ud.depth = state.d;
                filteredStates[i] = cloneState(state);
                if (lastObs) {
                    ud.rawDepthCm = lastObs.rawDepthCm;
                    ud.depthObservationChannel = lastObs.observationChannel;
                    ud.depthSignalContext = lastObs.signalContext;
                    ud.depthUpdatedAtMs = lastObs.receiptTimeMs;
                }
            }

            this._lastFilterRun = {
                firstIdx,
                s: s.slice(),
                filteredStates,
                predictedStates,
                observationCount: mapped.length
            };

            const last = cablePoints[cablePoints.length - 1]?.userData;
            this.lastSummary = last ? {
                pathLengthM: s[s.length - 1],
                estimatedDepthM: last.estimatedDepthM,
                sigmaM: last.depthSigmaM,
                confidence: last.depthConfidence,
                observationCount: mapped.length,
                finalized: false
            } : null;
            return this.lastSummary;
        }

        finalizePath(cablePoints) {
            if (!Array.isArray(cablePoints) || cablePoints.length === 0) return null;
            this.recompute(cablePoints);
            const run = this._lastFilterRun;
            if (!run || !run.observationCount) return null;

            const { firstIdx, s, filteredStates, predictedStates } = run;
            const lastIdx = cablePoints.length - 1;
            const smoothed = new Array(cablePoints.length).fill(null);
            smoothed[lastIdx] = cloneState(filteredStates[lastIdx]);
            for (let i = lastIdx - 1; i >= firstIdx; i--) {
                const ds = Math.max(0, s[i + 1] - s[i]);
                smoothed[i] = rtsSmoothStep(
                    filteredStates[i],
                    predictedStates[i + 1],
                    smoothed[i + 1],
                    ds
                );
            }

            for (let i = 0; i < firstIdx; i++) {
                const ud = cablePoints[i].userData || (cablePoints[i].userData = {});
                ud.finalDepthM = ud.estimatedDepthM;
                ud.finalDepthSigmaM = Math.max(Number(ud.depthSigmaM) || 1.0, 1.0);
                ud.finalDepthConfidence = 'LOW';
                ud.finalDepthSource = 'final_backfilled_before_first_measurement';
            }

            for (let i = firstIdx; i <= lastIdx; i++) {
                const st = smoothed[i] || filteredStates[i];
                if (!st) continue;
                const sigma = Math.sqrt(Math.max(st.p00, 0));
                const confidence = confidenceLabel(sigma, 'final_rts_smoothed');
                const ud = cablePoints[i].userData || (cablePoints[i].userData = {});
                ud.onlineDepthM = Number.isFinite(Number(ud.onlineDepthM)) ? Number(ud.onlineDepthM) : ud.estimatedDepthM;
                ud.onlineDepthSigmaM = Number.isFinite(Number(ud.onlineDepthSigmaM)) ? Number(ud.onlineDepthSigmaM) : ud.depthSigmaM;
                ud.finalDepthM = clamp(st.d, CFG.DEPTH_MIN_M, CFG.DEPTH_MAX_M);
                ud.finalDepthSigmaM = sigma;
                ud.finalDepthConfidence = confidence;
                ud.finalDepthSource = 'final_rts_smoothed';
                ud.estimatedDepthM = ud.finalDepthM;
                ud.depthSigmaM = ud.finalDepthSigmaM;
                ud.depthConfidence = ud.finalDepthConfidence;
                ud.depthSource = ud.finalDepthSource;
                ud.depth = ud.finalDepthM;
            }

            this.isFinalized = true;
            const last = cablePoints[lastIdx]?.userData;
            this.lastFinalSummary = last ? {
                pathLengthM: s[lastIdx],
                estimatedDepthM: last.finalDepthM,
                sigmaM: last.finalDepthSigmaM,
                confidence: last.finalDepthConfidence,
                observationCount: run.observationCount,
                finalized: true
            } : null;
            console.log('[V3深度] RTS最终平滑完成：', this.lastFinalSummary);
            return this.lastFinalSummary;
        }

        updatePath(cablePoints) {
            return this.recompute(cablePoints || []);
        }
    }

    global.PipelineDepthEstimatorV3 = PipelineDepthEstimatorV3;
    global.PIPELINE_DEPTH_V3_CONFIG = CFG;
    global.PIPELINE_DEPTH_V3_MODELS = { single: SINGLE_MODEL, q25: Q25_MODEL };
})(window);
