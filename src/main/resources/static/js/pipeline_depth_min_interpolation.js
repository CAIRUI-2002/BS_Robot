(function (root, factory) {
    const api = factory();
    if (typeof module === 'object' && module.exports) module.exports = api;
    root.PipelineDepthMinInterpolation = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
    'use strict';

    const MIN_VALID_DEPTH_CM = 10;
    const MAX_VALID_DEPTH_CM = 500;
    const DEFAULT_TOLERANCE_CM = 5;

    function finiteNumber(value) {
        if (value === null || value === undefined || value === '') return null;
        const n = Number(value);
        return Number.isFinite(n) ? n : null;
    }

    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    function cumulativeDistances(points) {
        const distances = new Array(points.length).fill(0);
        for (let i = 1; i < points.length; i++) {
            const dx = finiteNumber(points[i]?.x) - finiteNumber(points[i - 1]?.x);
            const dy = finiteNumber(points[i]?.y) - finiteNumber(points[i - 1]?.y);
            const dz = (finiteNumber(points[i]?.z) ?? 0) - (finiteNumber(points[i - 1]?.z) ?? 0);
            const segmentLength = Number.isFinite(dx) && Number.isFinite(dy)
                ? Math.hypot(dx, dy, Number.isFinite(dz) ? dz : 0)
                : 0;
            distances[i] = distances[i - 1] + segmentLength;
        }
        return distances;
    }

    function setDepth(point, depthCm) {
        if (!point) return;
        point.userData = point.userData || {};
        point.userData.depth = depthCm;
    }

    function apply(points, samples, toleranceCm = DEFAULT_TOLERANCE_CM) {
        const path = Array.isArray(points) ? points : [];
        const validSamples = (Array.isArray(samples) ? samples : [])
            .map((sample, order) => ({
                depthCm: finiteNumber(sample?.depthCm),
                pathIndex: finiteNumber(sample?.pathIndex),
                order
            }))
            .filter(sample => sample.depthCm !== null
                && sample.depthCm >= MIN_VALID_DEPTH_CM
                && sample.depthCm <= MAX_VALID_DEPTH_CM);

        if (path.length === 0 || validSamples.length === 0) {
            return {
                applied: false,
                reason: path.length === 0 ? 'empty_path' : 'no_valid_depth',
                sampleCount: validSamples.length,
                pathPointCount: path.length
            };
        }

        const minDepthCm = validSamples.reduce(
            (minimum, sample) => Math.min(minimum, sample.depthCm),
            Number.POSITIVE_INFINITY
        );
        const tolerance = Math.max(0, finiteNumber(toleranceCm) ?? DEFAULT_TOLERANCE_CM);
        const lowerDepthCm = Math.max(MIN_VALID_DEPTH_CM, minDepthCm - tolerance);
        const upperDepthCm = Math.min(MAX_VALID_DEPTH_CM, minDepthCm + tolerance);
        const distances = cumulativeDistances(path);
        const lastIndex = path.length - 1;

        const anchorsByIndex = new Map();
        validSamples.forEach((sample, order) => {
            const fallbackIndex = validSamples.length === 1
                ? 0
                : Math.round(order * lastIndex / (validSamples.length - 1));
            const index = clamp(Math.round(sample.pathIndex ?? fallbackIndex), 0, lastIndex);
            const depthCm = clamp(sample.depthCm, lowerDepthCm, upperDepthCm);
            const existing = anchorsByIndex.get(index) || [];
            existing.push(depthCm);
            anchorsByIndex.set(index, existing);
        });

        const anchors = Array.from(anchorsByIndex.entries())
            .map(([index, depths]) => ({
                index,
                distance: distances[index],
                depthCm: depths.reduce((sum, value) => sum + value, 0) / depths.length
            }))
            .sort((a, b) => a.index - b.index);

        if (anchors.length < 2) {
            path.forEach(point => setDepth(point, minDepthCm));
        } else {
            if (anchors[0].index !== 0) {
                anchors.unshift({ index: 0, distance: distances[0], depthCm: anchors[0].depthCm });
            }
            if (anchors[anchors.length - 1].index !== lastIndex) {
                const last = anchors[anchors.length - 1];
                anchors.push({ index: lastIndex, distance: distances[lastIndex], depthCm: last.depthCm });
            }

            let right = 1;
            path.forEach((point, index) => {
                while (right < anchors.length - 1 && index > anchors[right].index) right++;
                const leftAnchor = anchors[right - 1];
                const rightAnchor = anchors[right];
                const span = rightAnchor.distance - leftAnchor.distance;
                const ratio = span > 1e-9
                    ? (distances[index] - leftAnchor.distance) / span
                    : (rightAnchor.index === leftAnchor.index ? 0 : (index - leftAnchor.index) / (rightAnchor.index - leftAnchor.index));
                setDepth(point, leftAnchor.depthCm + clamp(ratio, 0, 1) * (rightAnchor.depthCm - leftAnchor.depthCm));
            });
        }

        return {
            applied: true,
            sampleCount: validSamples.length,
            anchorCount: anchorsByIndex.size,
            pathPointCount: path.length,
            minDepthCm,
            lowerDepthCm,
            upperDepthCm
        };
    }

    return {
        MIN_VALID_DEPTH_CM,
        MAX_VALID_DEPTH_CM,
        DEFAULT_TOLERANCE_CM,
        apply
    };
});
