(function () {
    "use strict";

    const VERTEX_SHADER_SOURCE = `
        attribute vec2 a_position;
        varying vec2 v_screenUv;

        void main() {
            v_screenUv = a_position * 0.5 + 0.5;
            gl_Position = vec4(a_position, 0.0, 1.0);
        }
    `;

    const VERTEX_SHADER_SOURCE_WEBGL2 = `#version 300 es
        in vec2 a_position;
        out vec2 v_screenUv;

        void main() {
            v_screenUv = a_position * 0.5 + 0.5;
            gl_Position = vec4(a_position, 0.0, 1.0);
        }
    `;

    const FRAGMENT_SHADER_SOURCE = `
        precision highp float;

        uniform sampler2D u_video;
        uniform bool u_applyCorrection;
        uniform vec2 u_inputSize;
        uniform vec2 u_outputSize;
        uniform vec4 u_inputK;
        uniform vec4 u_outputK;
        uniform vec4 u_distortion;
        uniform float u_skew;

        varying vec2 v_screenUv;

        void main() {
            // HTMLVideoElement 的图像坐标以左上角为原点；WebGL 屏幕坐标以左下角为原点。
            vec2 outputPixel = vec2(
                v_screenUv.x * u_outputSize.x,
                (1.0 - v_screenUv.y) * u_outputSize.y
            );

            vec2 sourcePixel;
            if (u_applyCorrection) {
                float x = (outputPixel.x - u_outputK.z) / u_outputK.x;
                float y = (outputPixel.y - u_outputK.w) / u_outputK.y;
                float radius = length(vec2(x, y));

                float scale = 1.0;
                if (radius > 0.0000001) {
                    float theta = atan(radius);
                    float theta2 = theta * theta;
                    float theta4 = theta2 * theta2;
                    float theta6 = theta4 * theta2;
                    float theta8 = theta4 * theta4;
                    float thetaDistorted = theta * (
                        1.0
                        + u_distortion.x * theta2
                        + u_distortion.y * theta4
                        + u_distortion.z * theta6
                        + u_distortion.w * theta8
                    );
                    scale = thetaDistorted / radius;
                }

                float xd = x * scale;
                float yd = y * scale;
                sourcePixel = vec2(
                    u_inputK.x * (xd + u_skew * yd) + u_inputK.z,
                    u_inputK.y * yd + u_inputK.w
                );
            } else {
                sourcePixel = vec2(
                    outputPixel.x * u_inputSize.x / u_outputSize.x,
                    outputPixel.y * u_inputSize.y / u_outputSize.y
                );
            }

            vec2 sourceUv = sourcePixel / u_inputSize;
            if (
                sourceUv.x < 0.0 || sourceUv.x > 1.0 ||
                sourceUv.y < 0.0 || sourceUv.y > 1.0
            ) {
                gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
                return;
            }

            // 未设置 UNPACK_FLIP_Y_WEBGL，纹理纵坐标继续使用视频的左上角图像坐标。
            gl_FragColor = texture2D(u_video, sourceUv);
        }
    `;

    const FRAGMENT_SHADER_SOURCE_WEBGL2 = `#version 300 es
        precision highp float;

        uniform sampler2D u_video;
        uniform bool u_applyCorrection;
        uniform vec2 u_inputSize;
        uniform vec2 u_outputSize;
        uniform vec4 u_inputK;
        uniform vec4 u_outputK;
        uniform vec4 u_distortion;
        uniform float u_skew;

        in vec2 v_screenUv;
        out vec4 outputColor;

        void main() {
            vec2 outputPixel = vec2(
                v_screenUv.x * u_outputSize.x,
                (1.0 - v_screenUv.y) * u_outputSize.y
            );

            vec2 sourcePixel;
            if (u_applyCorrection) {
                float x = (outputPixel.x - u_outputK.z) / u_outputK.x;
                float y = (outputPixel.y - u_outputK.w) / u_outputK.y;
                float radius = length(vec2(x, y));

                float scale = 1.0;
                if (radius > 0.0000001) {
                    float theta = atan(radius);
                    float theta2 = theta * theta;
                    float theta4 = theta2 * theta2;
                    float theta6 = theta4 * theta2;
                    float theta8 = theta4 * theta4;
                    float thetaDistorted = theta * (
                        1.0
                        + u_distortion.x * theta2
                        + u_distortion.y * theta4
                        + u_distortion.z * theta6
                        + u_distortion.w * theta8
                    );
                    scale = thetaDistorted / radius;
                }

                float xd = x * scale;
                float yd = y * scale;
                sourcePixel = vec2(
                    u_inputK.x * (xd + u_skew * yd) + u_inputK.z,
                    u_inputK.y * yd + u_inputK.w
                );
            } else {
                sourcePixel = vec2(
                    outputPixel.x * u_inputSize.x / u_outputSize.x,
                    outputPixel.y * u_inputSize.y / u_outputSize.y
                );
            }

            vec2 sourceUv = sourcePixel / u_inputSize;
            if (
                sourceUv.x < 0.0 || sourceUv.x > 1.0 ||
                sourceUv.y < 0.0 || sourceUv.y > 1.0
            ) {
                outputColor = vec4(0.0, 0.0, 0.0, 1.0);
                return;
            }

            outputColor = texture(u_video, sourceUv);
        }
    `;

    function flattenCameraMatrix(value) {
        if (!Array.isArray(value)) return null;
        if (value.length === 9 && value.every(Number.isFinite)) {
            return value.slice();
        }
        if (
            value.length === 3 &&
            value.every(row => Array.isArray(row) && row.length === 3)
        ) {
            const flattened = value.flat();
            return flattened.every(Number.isFinite) ? flattened : null;
        }
        return null;
    }

    function normalizeDistortion(value) {
        if (!Array.isArray(value) || value.length < 4) return null;
        const distortion = value.slice(0, 4).map(Number);
        return distortion.every(Number.isFinite) ? distortion : null;
    }

    function compileShader(gl, type, source) {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            const message = gl.getShaderInfoLog(shader) || "未知着色器编译错误";
            gl.deleteShader(shader);
            throw new Error(message);
        }
        return shader;
    }

    function createProgram(gl, vertexSource, fragmentSource) {
        const vertexShader = compileShader(gl, gl.VERTEX_SHADER, vertexSource);
        const fragmentShader = compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
        const program = gl.createProgram();
        gl.attachShader(program, vertexShader);
        gl.attachShader(program, fragmentShader);
        gl.linkProgram(program);
        gl.deleteShader(vertexShader);
        gl.deleteShader(fragmentShader);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            const message = gl.getProgramInfoLog(program) || "未知着色器链接错误";
            gl.deleteProgram(program);
            throw new Error(message);
        }
        return program;
    }

    class FisheyeWebGLRenderer {
        constructor(canvas, calibration = null, options = {}) {
            if (!(canvas instanceof HTMLCanvasElement)) {
                throw new TypeError("FisheyeWebGLRenderer 需要 HTMLCanvasElement");
            }

            this.canvas = canvas;
            this.video = null;
            this.calibration = calibration || {};
            this.profile = options.profile || "inspection";
            this.correctionRequested = options.correctionEnabled !== false;
            this.contextLost = false;
            this.lastError = "";
            this.textureWidth = 0;
            this.textureHeight = 0;

            this._onContextLost = event => {
                event.preventDefault();
                this.contextLost = true;
                this.lastError = "WebGL 上下文已丢失";
            };
            this._onContextRestored = () => {
                this.contextLost = false;
                this.lastError = "";
                this._initializeResources();
            };

            this.canvas.addEventListener("webglcontextlost", this._onContextLost, false);
            this.canvas.addEventListener("webglcontextrestored", this._onContextRestored, false);
            this._initializeContext();
            this._initializeResources();
        }

        _initializeContext() {
            const attributes = {
                alpha: false,
                antialias: false,
                depth: false,
                stencil: false,
                preserveDrawingBuffer: false,
                powerPreference: "high-performance"
            };
            this.gl = this.canvas.getContext("webgl2", attributes)
                || this.canvas.getContext("webgl", attributes)
                || this.canvas.getContext("experimental-webgl", attributes);
            if (!this.gl) {
                throw new Error("当前浏览器或设备不支持 WebGL");
            }
            this.isWebGL2 = typeof WebGL2RenderingContext !== "undefined"
                && this.gl instanceof WebGL2RenderingContext;
        }

        _initializeResources() {
            const gl = this.gl;
            this.program = createProgram(
                gl,
                this.isWebGL2 ? VERTEX_SHADER_SOURCE_WEBGL2 : VERTEX_SHADER_SOURCE,
                this.isWebGL2 ? FRAGMENT_SHADER_SOURCE_WEBGL2 : FRAGMENT_SHADER_SOURCE
            );
            this.locations = {
                position: gl.getAttribLocation(this.program, "a_position"),
                video: gl.getUniformLocation(this.program, "u_video"),
                applyCorrection: gl.getUniformLocation(this.program, "u_applyCorrection"),
                inputSize: gl.getUniformLocation(this.program, "u_inputSize"),
                outputSize: gl.getUniformLocation(this.program, "u_outputSize"),
                inputK: gl.getUniformLocation(this.program, "u_inputK"),
                outputK: gl.getUniformLocation(this.program, "u_outputK"),
                distortion: gl.getUniformLocation(this.program, "u_distortion"),
                skew: gl.getUniformLocation(this.program, "u_skew")
            };

            this.positionBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
            gl.bufferData(
                gl.ARRAY_BUFFER,
                new Float32Array([-1, -1, 3, -1, -1, 3]),
                gl.STATIC_DRAW
            );

            this.videoTexture = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, this.videoTexture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
            this.textureWidth = 0;
            this.textureHeight = 0;
        }

        setVideo(video) {
            this.video = video || null;
        }

        setCalibration(calibration) {
            this.calibration = calibration || {};
            this.lastError = "";
        }

        setProfile(profile) {
            this.profile = profile || "inspection";
        }

        setCorrectionEnabled(enabled) {
            this.correctionRequested = !!enabled;
        }

        resize(width, height) {
            const nextWidth = Math.max(2, Math.round(width || 0));
            const nextHeight = Math.max(2, Math.round(height || 0));
            if (this.canvas.width !== nextWidth) this.canvas.width = nextWidth;
            if (this.canvas.height !== nextHeight) this.canvas.height = nextHeight;
        }

        _getCorrectionParameters(outputWidth, outputHeight) {
            if (!this.correctionRequested) {
                return { active: false, reason: "已选择原始画面" };
            }
            if (!this.calibration || this.calibration.enabled !== true) {
                return { active: false, reason: "相机尚未写入实机标定参数" };
            }

            const inputSize = this.calibration.inputSize;
            const inputK = flattenCameraMatrix(this.calibration.K);
            const distortion = normalizeDistortion(this.calibration.D);
            const profileConfig = this.calibration.profiles && this.calibration.profiles[this.profile];
            const outputK = flattenCameraMatrix(profileConfig && profileConfig.Knew);
            if (
                !Array.isArray(inputSize) || inputSize.length !== 2 ||
                !inputSize.every(value => Number.isFinite(value) && value > 0) ||
                !inputK || !distortion || !outputK
            ) {
                return { active: false, reason: "标定配置不完整" };
            }

            const videoWidth = this.video && this.video.videoWidth;
            const videoHeight = this.video && this.video.videoHeight;
            if (!videoWidth || !videoHeight) {
                return { active: false, reason: "视频尺寸尚未就绪" };
            }

            const configuredAspect = inputSize[0] / inputSize[1];
            const actualAspect = videoWidth / videoHeight;
            if (Math.abs(configuredAspect - actualAspect) / configuredAspect > 0.01) {
                return {
                    active: false,
                    reason: `视频比例 ${videoWidth}x${videoHeight} 与标定比例不一致`
                };
            }

            const inputScaleX = videoWidth / inputSize[0];
            const inputScaleY = videoHeight / inputSize[1];
            const outputScaleX = outputWidth / inputSize[0];
            const outputScaleY = outputHeight / inputSize[1];

            return {
                active: true,
                reason: "",
                inputK: [
                    inputK[0] * inputScaleX,
                    inputK[4] * inputScaleY,
                    inputK[2] * inputScaleX,
                    inputK[5] * inputScaleY
                ],
                outputK: [
                    outputK[0] * outputScaleX,
                    outputK[4] * outputScaleY,
                    outputK[2] * outputScaleX,
                    outputK[5] * outputScaleY
                ],
                distortion,
                skew: inputK[1] / Math.max(inputK[0], 0.000001)
            };
        }

        render() {
            if (this.contextLost || !this.gl || !this.video) return false;
            if (
                this.video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA ||
                !this.video.videoWidth || !this.video.videoHeight
            ) {
                return false;
            }

            const gl = this.gl;
            const outputWidth = this.canvas.width;
            const outputHeight = this.canvas.height;
            const parameters = this._getCorrectionParameters(outputWidth, outputHeight);
            this.lastError = parameters.active ? "" : parameters.reason;

            gl.viewport(0, 0, outputWidth, outputHeight);
            gl.clearColor(0, 0, 0, 1);
            gl.clear(gl.COLOR_BUFFER_BIT);
            gl.useProgram(this.program);

            gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
            gl.enableVertexAttribArray(this.locations.position);
            gl.vertexAttribPointer(this.locations.position, 2, gl.FLOAT, false, 0, 0);

            gl.activeTexture(gl.TEXTURE0);
            gl.bindTexture(gl.TEXTURE_2D, this.videoTexture);
            try {
                gl.texImage2D(
                    gl.TEXTURE_2D,
                    0,
                    gl.RGBA,
                    gl.RGBA,
                    gl.UNSIGNED_BYTE,
                    this.video
                );
            } catch (error) {
                this.lastError = error && error.message ? error.message : "视频帧上传GPU失败";
                return false;
            }

            this.textureWidth = this.video.videoWidth;
            this.textureHeight = this.video.videoHeight;
            gl.uniform1i(this.locations.video, 0);
            gl.uniform1i(this.locations.applyCorrection, parameters.active ? 1 : 0);
            gl.uniform2f(this.locations.inputSize, this.video.videoWidth, this.video.videoHeight);
            gl.uniform2f(this.locations.outputSize, outputWidth, outputHeight);

            if (parameters.active) {
                gl.uniform4fv(this.locations.inputK, parameters.inputK);
                gl.uniform4fv(this.locations.outputK, parameters.outputK);
                gl.uniform4fv(this.locations.distortion, parameters.distortion);
                gl.uniform1f(this.locations.skew, parameters.skew);
            } else {
                gl.uniform4f(this.locations.inputK, 1, 1, 0, 0);
                gl.uniform4f(this.locations.outputK, 1, 1, 0, 0);
                gl.uniform4f(this.locations.distortion, 0, 0, 0, 0);
                gl.uniform1f(this.locations.skew, 0);
            }

            gl.drawArrays(gl.TRIANGLES, 0, 3);
            return true;
        }

        isCorrectionActive() {
            const parameters = this._getCorrectionParameters(this.canvas.width, this.canvas.height);
            return parameters.active;
        }

        getCorrectionReason() {
            const parameters = this._getCorrectionParameters(this.canvas.width, this.canvas.height);
            return parameters.active ? "" : parameters.reason;
        }

        captureDataUrl(width, height, quality = 0.92) {
            const previousWidth = this.canvas.width;
            const previousHeight = this.canvas.height;
            this.resize(width, height);
            if (!this.render()) {
                this.resize(previousWidth, previousHeight);
                throw new Error(this.lastError || "摄像头画面尚未就绪");
            }

            const dataUrl = this.canvas.toDataURL("image/jpeg", quality);
            this.resize(previousWidth, previousHeight);
            this.render();
            return dataUrl;
        }

        destroy() {
            this.canvas.removeEventListener("webglcontextlost", this._onContextLost, false);
            this.canvas.removeEventListener("webglcontextrestored", this._onContextRestored, false);
            if (!this.gl) return;
            if (this.videoTexture) this.gl.deleteTexture(this.videoTexture);
            if (this.positionBuffer) this.gl.deleteBuffer(this.positionBuffer);
            if (this.program) this.gl.deleteProgram(this.program);
            this.videoTexture = null;
            this.positionBuffer = null;
            this.program = null;
        }
    }

    window.FisheyeWebGLRenderer = FisheyeWebGLRenderer;
})();

