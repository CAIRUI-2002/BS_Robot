'use strict';

const assert = require('assert');
const interpolation = require('../src/main/resources/static/js/pipeline_depth_min_interpolation.js');

function path(xs) {
    return xs.map(x => ({ x, y: 0, z: 0, userData: {} }));
}

{
    const points = path([0, 1, 2, 3, 4]);
    const summary = interpolation.apply(points, [
        { depthCm: 100, pathIndex: 0 },
        { depthCm: 130, pathIndex: 2 },
        { depthCm: 103, pathIndex: 4 }
    ]);
    assert.strictEqual(summary.minDepthCm, 100);
    assert.strictEqual(points[0].userData.depth, 100);
    assert.strictEqual(points[2].userData.depth, 105);
    assert.strictEqual(points[4].userData.depth, 103);
    assert(points.every(point => point.userData.depth >= 95 && point.userData.depth <= 105));
}

{
    const points = path([0, 1, 2]);
    interpolation.apply(points, [{ depthCm: 88, pathIndex: 1 }]);
    assert.deepStrictEqual(points.map(point => point.userData.depth), [88, 88, 88]);
}

{
    const points = path([0, 1, 10]);
    interpolation.apply(points, [
        { depthCm: 100, pathIndex: 0 },
        { depthCm: 105, pathIndex: 2 },
        { depthCm: 9, pathIndex: 1 },
        { depthCm: 501, pathIndex: 1 }
    ]);
    assert.strictEqual(points[1].userData.depth, 100.5);
}

console.log('pipeline depth minimum interpolation tests passed');
