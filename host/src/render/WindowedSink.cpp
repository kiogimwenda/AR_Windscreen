#include "ar_drive_assist/render/WindowedSink.h"

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <opencv2/imgproc.hpp>

namespace ar_drive_assist {
namespace {

// ---------------------------------------------------------------------------------------------
// Shaders (GLSL 3.30 core)
// ---------------------------------------------------------------------------------------------

const char* kVideoVs = R"glsl(#version 330 core
out vec2 vUv;
void main() {
    // One triangle covering the screen; uv (0,0) is the image's TOP-left.
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = vec2(p.x, 1.0 - p.y);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
})glsl";

const char* kVideoFs = R"glsl(#version 330 core
in vec2 vUv;
out vec4 o;
uniform sampler2D uVideo;
void main() { o = vec4(texture(uVideo, vUv).rgb, 1.0); })glsl";

// Road-space items: vehicle-frame metres projected with the calibrated camera model. The
// distortion formulas are those of common/Camera.h (OpenCV's model), term for term.
const char* kRoadVs = R"glsl(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in vec2 aUv;
uniform mat4 uCamFromVeh;
uniform vec4 uK;     // fx, fy, cx, cy
uniform vec4 uDist;  // k1, k2, p1, p2
uniform float uK3;
uniform vec2 uVideo;
out vec4 vColor;
out vec2 vUv;
out float vDepth;
void main() {
    vec4 pc = uCamFromVeh * vec4(aPos, 1.0);
    vec2 n = pc.xy / max(pc.z, 1e-3);
    float r2 = dot(n, n);
    float radial = 1.0 + uDist.x * r2 + uDist.y * r2 * r2 + uK3 * r2 * r2 * r2;
    vec2 d = vec2(n.x * radial + 2.0 * uDist.z * n.x * n.y + uDist.w * (r2 + 2.0 * n.x * n.x),
                  n.y * radial + uDist.z * (r2 + 2.0 * n.y * n.y) + 2.0 * uDist.w * n.x * n.y);
    vec2 px = vec2(uK.x * d.x + uK.z, uK.y * d.y + uK.w);
    // The division by depth was done above (distortion needs it), but the GPU must still know the
    // depth: multiplying back by w = z makes attribute interpolation PERSPECTIVE-correct. With
    // w = 1 the across-coordinate was interpolated linearly in screen space, each segment's two
    // triangles disagreed, and band edges came out as a sawtooth (seen in the first capture).
    float w = max(pc.z, 1e-3);
    gl_Position = vec4((px.x / uVideo.x * 2.0 - 1.0) * w, (1.0 - px.y / uVideo.y * 2.0) * w, 0.0, w);
    vColor = aColor;
    vUv = aUv;
    vDepth = pc.z;
})glsl";

const char* kRoadFs = R"glsl(#version 330 core
in vec4 vColor;
in vec2 vUv;       // x: metres along the item; y: band/ring -1..1 across, barrier 0..1 up
in float vDepth;   // camera depth of this fragment (m)
out vec4 o;
uniform int uPattern;      // 0 band/ring, 1 filled zone, 2 barrier
uniform float uCore;       // band: the solid core's share of the half-width
uniform float uOpacity, uTime, uPulseHz, uShimmerHz;
uniform sampler2D uOcc;    // nearest hazard depth per pixel (m), 0 = none (rule 4)
uniform vec2 uVideo;
void main() {
    float a;
    if (uPattern == 0) {
        float x = abs(vUv.y);
        float aa = fwidth(vUv.y) * 1.5;
        float core = (1.0 - smoothstep(uCore - aa, uCore + aa, x)) * 0.6;
        float glow = pow(1.0 - smoothstep(uCore, 1.0, x), 2.0) * 0.35;
        a = max(core, glow);
    } else if (uPattern == 1) {
        a = 0.45;
    } else {
        // Holographic barrier: bright top and bottom edges, fading upwards, moving diagonal bands.
        float fade = mix(0.75, 0.15, vUv.y);
        float bands = step(0.5, fract(vUv.x * 1.2 + vUv.y * 1.5 - uTime * 0.6));
        float edge = smoothstep(0.92, 1.0, vUv.y) + (1.0 - smoothstep(0.0, 0.06, vUv.y)) * 0.8;
        a = fade * (0.55 + 0.3 * bands) + edge;
    }
    float pulse = uPulseHz > 0.0 ? 0.7 + 0.3 * sin(6.2831853 * uPulseHz * uTime) : 1.0;
    float shim = uShimmerHz > 0.0 ? 0.85 + 0.3 * sin(6.2831853 * (vUv.x * 0.15 - uShimmerHz * uTime)) : 1.0;
    // Hidden only where this fragment lies BEHIND the hazard at this pixel (with a 0.3 m margin
    // and a soft 0.4 m transition), so graphics in front of a hazard stay visible.
    float hz = texture(uOcc, vec2(gl_FragCoord.x / uVideo.x, 1.0 - gl_FragCoord.y / uVideo.y)).r;
    float hidden = hz > 0.0 ? clamp((vDepth - hz - 0.3) / 0.4, 0.0, 1.0) : 0.0;
    o = vec4(vColor.rgb * pulse * shim, clamp(a * vColor.a * uOpacity * (1.0 - hidden), 0.0, 1.0));
})glsl";

// Image-space quads (mask glows and textured badges): positions in video pixels.
const char* kQuadVs = R"glsl(#version 330 core
layout(location = 0) in vec2 aPix;
layout(location = 1) in vec2 aTex;
uniform vec2 uVideo;
out vec2 vPix;
out vec2 vTex;
void main() {
    vPix = aPix;
    vTex = aTex;
    gl_Position = vec4(aPix.x / uVideo.x * 2.0 - 1.0, 1.0 - aPix.y / uVideo.y * 2.0, 0.0, 1.0);
})glsl";

// The hazard glow (overlay design §3.2): a rim along the outline, a halo just outside it, at most
// a 22% tint inside (rule 4: the object stays fully visible), and a slow diagonal light band
// (shimmer). The outline is found by sampling the mask on a circle around each pixel: the maximum
// (a dilation) gives the outside halo, the minimum on a half-size circle (an erosion) the inside
// rim.
const char* kMaskFs = R"glsl(#version 330 core
in vec2 vPix;
out vec4 o;
uniform sampler2D uMask;
uniform vec4 uRect;        // the mask's coverage in video pixels: x, y, w, h
uniform float uGlowPx, uTime, uShimmerHz, uOpacity;
uniform vec4 uColor;
float m(vec2 px) { return texture(uMask, (px - uRect.xy) / uRect.zw).r; }
void main() {
    float inside = smoothstep(0.35, 0.65, m(vPix));
    float dil = 0.0, ero = 1.0;
    for (int i = 0; i < 12; ++i) {
        float ang = 6.2831853 * float(i) / 12.0;
        vec2 off = vec2(cos(ang), sin(ang));
        dil = max(dil, m(vPix + off * uGlowPx));
        ero = min(ero, m(vPix + off * uGlowPx * 0.5));
    }
    float halo = smoothstep(0.1, 0.9, dil) * (1.0 - inside);
    float rim = inside * (1.0 - smoothstep(0.1, 0.9, ero));
    float glow = max(halo * 0.7, rim);
    float band = 0.5 + 0.5 * sin(6.2831853 * ((vPix.x + vPix.y) / 260.0 - uShimmerHz * uTime));
    float shim = uShimmerHz > 0.0 ? 0.55 + 0.6 * band : 1.0;
    o = vec4(uColor.rgb, clamp(glow * shim + inside * 0.22, 0.0, 1.0) * uOpacity * uColor.a);
})glsl";

const char* kImageFs = R"glsl(#version 330 core
in vec2 vTex;
out vec4 o;
uniform sampler2D uTex;
uniform float uOpacity;
void main() { vec4 c = texture(uTex, vTex); o = vec4(c.rgb, c.a * uOpacity); })glsl";

GLuint compile(GLenum type, const char* src, std::string& err) {
    const GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        err += log;
    }
    return s;
}

GLuint program(const char* vs, const char* fs, std::string& err) {
    const GLuint p = glCreateProgram();
    const GLuint v = compile(GL_VERTEX_SHADER, vs, err), f = compile(GL_FRAGMENT_SHADER, fs, err);
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof log, nullptr, log);
        err += log;
    }
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

struct RoadVertex {
    float x, y, z, r, g, b, a, u, v;
};

constexpr double kNearZ = 0.3;  // metres in front of the camera: geometry nearer is trimmed

double cameraZ(const Eigen::Isometry3d& T, const Eigen::Vector3d& p) {
    return (T * p).z();
}

void push(std::vector<RoadVertex>& out, const Eigen::Vector3d& p, const Rgba& c, float alpha,
          float u, float v) {
    out.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()),
                   c.r, c.g, c.b, c.a * alpha, u, v});
}

// A band along a polyline lying on the road: two triangles per segment, `across` -1..1 between
// the outer edges of the halo.
void tessellateBand(const OverlayItem& it, const Eigen::Isometry3d& T, float outerHalf,
                    std::vector<RoadVertex>& out) {
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < it.points.size(); ++i)
        if (cameraZ(T, it.points[i]) >= kNearZ) keep.push_back(i);
    if (keep.size() < 2) return;
    std::vector<Eigen::Vector3d> L, R;
    std::vector<float> along, alpha;
    std::vector<Rgba> col;
    float s = 0;
    for (std::size_t k = 0; k < keep.size(); ++k) {
        const std::size_t i = keep[k];
        const Eigen::Vector3d prev = it.points[keep[k == 0 ? 0 : k - 1]];
        const Eigen::Vector3d next = it.points[keep[k + 1 < keep.size() ? k + 1 : k]];
        Eigen::Vector2d t = (next - prev).head<2>();
        if (t.norm() < 1e-9) t = {1, 0};
        t.normalize();
        const Eigen::Vector3d n(-t.y(), t.x(), 0);
        L.push_back(it.points[i] + n * outerHalf);
        R.push_back(it.points[i] - n * outerHalf);
        if (k > 0) s += static_cast<float>((it.points[i] - it.points[keep[k - 1]]).norm());
        along.push_back(s);
        alpha.push_back(i < it.pointOpacity.size() ? it.pointOpacity[i] : 1.0f);
        col.push_back(i < it.pointColor.size() ? it.pointColor[i] : it.style.color);
    }
    for (std::size_t k = 0; k + 1 < L.size(); ++k) {
        push(out, L[k], col[k], alpha[k], along[k], 1);
        push(out, R[k], col[k], alpha[k], along[k], -1);
        push(out, L[k + 1], col[k + 1], alpha[k + 1], along[k + 1], 1);
        push(out, R[k], col[k], alpha[k], along[k], -1);
        push(out, R[k + 1], col[k + 1], alpha[k + 1], along[k + 1], -1);
        push(out, L[k + 1], col[k + 1], alpha[k + 1], along[k + 1], 1);
    }
}

cv::Mat rasteriseLabel(const OverlayItem& it) {
    const double scale = it.sizePx / 30.0;
    int base = 0;
    const cv::Size ts = cv::getTextSize(it.text, cv::FONT_HERSHEY_DUPLEX, scale, 2, &base);
    const int pad = static_cast<int>(it.sizePx * 0.35);
    cv::Mat img(ts.height + base + 2 * pad, ts.width + 2 * pad, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    const int r = pad;
    const cv::Scalar bg(20, 20, 20, 150);
    cv::rectangle(img, {r, 0}, {img.cols - r, img.rows}, bg, cv::FILLED);
    cv::rectangle(img, {0, r}, {img.cols, img.rows - r}, bg, cv::FILLED);
    for (auto c : {cv::Point(r, r), cv::Point(img.cols - r, r), cv::Point(r, img.rows - r),
                   cv::Point(img.cols - r, img.rows - r)})
        cv::circle(img, c, r, bg, cv::FILLED, cv::LINE_AA);
    const cv::Scalar fg(it.style.color.b * 255, it.style.color.g * 255, it.style.color.r * 255,
                        255);
    cv::putText(img, it.text, {pad, pad + ts.height}, cv::FONT_HERSHEY_DUPLEX, scale, fg, 2,
                cv::LINE_AA);
    return img;
}

cv::Mat rasteriseBadge(const OverlayItem& it) {
    // A speed-limit sign: white disc, red ring, black number.
    const int d = static_cast<int>(it.sizePx);
    cv::Mat img(d, d, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    const cv::Point c(d / 2, d / 2);
    cv::circle(img, c, d / 2 - 1, cv::Scalar(40, 40, 230, 255), cv::FILLED, cv::LINE_AA);
    cv::circle(img, c, static_cast<int>(d * 0.38), cv::Scalar(255, 255, 255, 255), cv::FILLED,
               cv::LINE_AA);
    const double scale = d / 70.0 * (it.text.size() > 2 ? 0.8 : 1.0);
    int base = 0;
    const cv::Size ts = cv::getTextSize(it.text, cv::FONT_HERSHEY_DUPLEX, scale, 2, &base);
    cv::putText(img, it.text, {c.x - ts.width / 2, c.y + ts.height / 2}, cv::FONT_HERSHEY_DUPLEX,
                scale, cv::Scalar(0, 0, 0, 255), 2, cv::LINE_AA);
    return img;
}

cv::Mat rasteriseChevron(const OverlayItem& it) {
    const int d = static_cast<int>(it.sizePx);
    cv::Mat img(d, d, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    const cv::Scalar col(it.style.color.b * 255, it.style.color.g * 255, it.style.color.r * 255,
                         230);
    std::vector<cv::Point> tri{
        {d / 5, d / 6}, {d * 4 / 5, d / 2}, {d / 5, d * 5 / 6}, {d * 2 / 5, d / 2}};
    cv::fillPoly(img, std::vector<std::vector<cv::Point>>{tri}, col, cv::LINE_AA);
    return img;
}

}  // namespace

struct WindowedSink::Gl {
    GLuint videoProg = 0, roadProg = 0, maskProg = 0, imageProg = 0;
    GLuint vao = 0, roadVbo = 0, quadVbo = 0;
    GLuint videoTex = 0, pbo[2] = {0, 0}, fbo = 0, fboTex = 0, occTex = 0, maskTex = 0;
    int videoW = 0, videoH = 0, frame = 0;
    GLuint queries[3][2] = {};
    int queryFrame = 0;
    struct Cached {
        GLuint tex;
        int w, h, lastUsed;
    };
    std::map<std::string, Cached> textCache;
};

WindowedSink::WindowedSink(WindowedSinkOptions options) : opt_(std::move(options)), gl_(new Gl) {}

WindowedSink::~WindowedSink() {
    if (glContext_) {
        SDL_GL_DeleteContext(static_cast<SDL_GLContext>(glContext_));
    }
    if (window_) SDL_DestroyWindow(window_);
    // The video subsystem and the GL library are deliberately NOT unloaded (see init()): Mesa's
    // D3D12 driver keeps worker threads, and unloading the library under them crashes the
    // process (found 2026-10-07: a segfault on every shutdown of the running system, the render
    // thread stopping while the others were still being joined). They go with the process.
}

bool WindowedSink::init(const FrameGeometry& geometry, const CameraModel& camera,
                        const Eigen::Isometry3d& cameraFromVehicle) {
    camera_ = camera;
    cameraFromVehicle_ = cameraFromVehicle;
    // Select Mesa's Direct3D 12 driver on the named adapter BEFORE the GL context exists (see the
    // header). Does not override values the environment already sets.
    ::setenv("GALLIUM_DRIVER", "d3d12", 0);
    ::setenv("MESA_D3D12_DEFAULT_ADAPTER_NAME", opt_.adapterHint.c_str(), 0);
    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
        error_ = std::string("SDL video: ") + SDL_GetError();
        return false;
    }
    // One reference to the GL library for the life of the process, so destroying the window never
    // unloads the driver while its threads run (see the destructor).
    static const bool glPinned = SDL_GL_LoadLibrary(nullptr) == 0;
    if (!glPinned) {
        error_ = std::string("SDL GL library: ") + SDL_GetError();
        return false;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI;
    flags |= opt_.hidden ? SDL_WINDOW_HIDDEN : SDL_WINDOW_SHOWN;
    int w = geometry.width, h = geometry.height;
    if (opt_.fullscreen) {
        SDL_DisplayMode dm;
        if (SDL_GetDesktopDisplayMode(0, &dm) == 0) {
            w = dm.w;
            h = dm.h;
        }
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    }
    window_ = SDL_CreateWindow(opt_.title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               w, h, flags);
    if (!window_) {
        error_ = std::string("SDL window: ") + SDL_GetError();
        return false;
    }
    glContext_ = SDL_GL_CreateContext(window_);
    if (!glContext_) {
        error_ = std::string("GL context: ") + SDL_GetError();
        return false;
    }
    glewExperimental = GL_TRUE;
    glewInit();  // under Wayland GLEW reports a spurious error; the entry points still load
    glGetError();
    SDL_GL_SetSwapInterval(opt_.vsync ? 1 : 0);
    renderer_ = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    software_ = renderer_.find("llvmpipe") != std::string::npos ||
                renderer_.find("softpipe") != std::string::npos;
    if (software_ && opt_.requireGpu) {
        error_ = "software renderer (" + renderer_ + "): refusing, requireGpu is set";
        return false;
    }
    SDL_GL_GetDrawableSize(window_, &w, &h);
    geometry_ = {w, h, static_cast<float>(w) / std::max(h, 1)};

    std::string err;
    gl_->videoProg = program(kVideoVs, kVideoFs, err);
    gl_->roadProg = program(kRoadVs, kRoadFs, err);
    gl_->maskProg = program(kQuadVs, kMaskFs, err);
    gl_->imageProg = program(kQuadVs, kImageFs, err);
    if (!err.empty()) {
        error_ = "shader: " + err;
        return false;
    }
    glGenVertexArrays(1, &gl_->vao);
    glBindVertexArray(gl_->vao);
    glGenBuffers(1, &gl_->roadVbo);
    glGenBuffers(1, &gl_->quadVbo);
    glGenBuffers(2, gl_->pbo);
    for (auto& q : gl_->queries) glGenQueries(2, q);
    auto tex = [](GLuint& t, GLint filter, GLint wrap) {
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    };
    tex(gl_->videoTex, GL_LINEAR, GL_CLAMP_TO_EDGE);
    tex(gl_->fboTex, GL_LINEAR, GL_CLAMP_TO_EDGE);
    tex(gl_->occTex, GL_LINEAR, GL_CLAMP_TO_EDGE);
    tex(gl_->maskTex, GL_LINEAR, GL_CLAMP_TO_BORDER);
    const float zero[4] = {0, 0, 0, 0};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, zero);
    glGenFramebuffers(1, &gl_->fbo);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    startSeconds_ = SDL_GetTicks() / 1000.0;
    return true;
}

bool WindowedSink::pollEvents() {
    SDL_Event e;
    bool open = true;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) open = false;
        if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) open = false;
    }
    return open;
}

void WindowedSink::present(const CompositedFrame& frame) {
    const auto cpu0 = std::chrono::steady_clock::now();
    Gl& g = *gl_;
    cv::Mat video = frame.video;
    if (video.empty()) return;
    if (!video.isContinuous()) video = video.clone();
    const int W = video.cols, H = video.rows;
    const std::size_t bytes = static_cast<std::size_t>(W) * H * 3;

    // (Re)allocate the video-sized resources when the frame size changes.
    if (W != g.videoW || H != g.videoH) {
        g.videoW = W;
        g.videoH = H;
        glBindTexture(GL_TEXTURE_2D, g.videoTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, W, H, 0, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, g.fboTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindFramebuffer(GL_FRAMEBUFFER, g.fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g.fboTex, 0);
        for (GLuint p : g.pbo) {
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, p);
            glBufferData(GL_PIXEL_UNPACK_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr,
                         GL_STREAM_DRAW);
        }
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }
    const float t = static_cast<float>(SDL_GetTicks() / 1000.0 - startSeconds_);
    GLuint* q = g.queries[g.queryFrame % 3];

    // 1. Video upload through a pixel buffer object, then drawn into the framebuffer.
    glBeginQuery(GL_TIME_ELAPSED, q[0]);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, g.pbo[g.frame & 1]);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr,
                 GL_STREAM_DRAW);  // orphan
    if (void* dst = glMapBufferRange(GL_PIXEL_UNPACK_BUFFER, 0, static_cast<GLsizeiptr>(bytes),
                                     GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT)) {
        std::memcpy(dst, video.data, bytes);
        glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
    }
    glBindTexture(GL_TEXTURE_2D, g.videoTex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glEndQuery(GL_TIME_ELAPSED);

    glBindFramebuffer(GL_FRAMEBUFFER, g.fbo);
    glViewport(0, 0, W, H);
    glDisable(GL_BLEND);
    glBindVertexArray(g.vao);
    glUseProgram(g.videoProg);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g.videoTex);
    glUniform1i(glGetUniformLocation(g.videoProg, "uVideo"), 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glBeginQuery(GL_TIME_ELAPSED, q[1]);
    // 2. Occluders: every hazard silhouette at quarter resolution, holding the NEAREST hazard's
    // camera depth at each pixel (0 = no hazard).
    const int ow = std::max(1, W / 4), oh = std::max(1, H / 4);
    cv::Mat occ(oh, ow, CV_32FC1, cv::Scalar(0));
    for (const HazardOccluder& h : frame.overlays.occluders) {
        const PixelMask& m = h.mask;
        const float d = static_cast<float>(std::max(h.depthM, 0.01));
        const double s = m.cellPx / 4.0;
        for (int cy = 0; cy < m.h; ++cy)
            for (int cx = 0; cx < m.w; ++cx) {
                if (!m.bits[static_cast<std::size_t>(cy) * m.w + cx]) continue;
                const cv::Rect r =
                    cv::Rect(static_cast<int>((m.x + cx) * s), static_cast<int>((m.y + cy) * s),
                             std::max(1, static_cast<int>(std::ceil(s))),
                             std::max(1, static_cast<int>(std::ceil(s)))) &
                    cv::Rect(0, 0, ow, oh);
                for (int y = r.y; y < r.y + r.height; ++y)
                    for (int x = r.x; x < r.x + r.width; ++x) {
                        float& cur = occ.at<float>(y, x);
                        if (cur == 0 || d < cur) cur = d;
                    }
            }
    }
    glBindTexture(GL_TEXTURE_2D, g.occTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);  // depths must not blend
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, ow, oh, 0, GL_RED, GL_FLOAT, occ.data);

    // 3. Items, back to front.
    std::vector<const OverlayItem*> items;
    for (const auto& it : frame.overlays.items) items.push_back(&it);
    std::stable_sort(items.begin(), items.end(), [](const OverlayItem* a, const OverlayItem* b) {
        return a->style.layer < b->style.layer;
    });
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Eigen::Matrix4f M = cameraFromVehicle_.matrix().cast<float>();
    auto setRoadUniforms = [&]() {
        glUseProgram(g.roadProg);
        glUniformMatrix4fv(glGetUniformLocation(g.roadProg, "uCamFromVeh"), 1, GL_FALSE, M.data());
        glUniform4f(glGetUniformLocation(g.roadProg, "uK"), static_cast<float>(camera_.fx),
                    static_cast<float>(camera_.fy), static_cast<float>(camera_.cx),
                    static_cast<float>(camera_.cy));
        glUniform4f(glGetUniformLocation(g.roadProg, "uDist"), static_cast<float>(camera_.k1),
                    static_cast<float>(camera_.k2), static_cast<float>(camera_.p1),
                    static_cast<float>(camera_.p2));
        glUniform1f(glGetUniformLocation(g.roadProg, "uK3"), static_cast<float>(camera_.k3));
        glUniform2f(glGetUniformLocation(g.roadProg, "uVideo"), static_cast<float>(W),
                    static_cast<float>(H));
        glUniform1f(glGetUniformLocation(g.roadProg, "uTime"), t);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, g.occTex);
        glUniform1i(glGetUniformLocation(g.roadProg, "uOcc"), 1);
    };
    auto drawRoad = [&](const std::vector<RoadVertex>& v, int pattern, float core,
                        const OverlayItem& it) {
        if (v.empty()) return;
        setRoadUniforms();
        glUniform1i(glGetUniformLocation(g.roadProg, "uPattern"), pattern);
        glUniform1f(glGetUniformLocation(g.roadProg, "uCore"), core);
        const float occl = it.occludedByHazards ? 1.0f : 0.0f;
        glUniform1f(glGetUniformLocation(g.roadProg, "uOpacity"), it.style.opacity);
        glUniform1f(glGetUniformLocation(g.roadProg, "uPulseHz"), std::min(it.style.pulseHz, 3.0f));
        glUniform1f(glGetUniformLocation(g.roadProg, "uShimmerHz"),
                    std::min(it.style.shimmerHz, 3.0f));
        if (occl == 0.0f) {  // not occluded: point the sampler at a blank unit
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glBindBuffer(GL_ARRAY_BUFFER, g.roadVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(RoadVertex)),
                     v.data(), GL_STREAM_DRAW);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RoadVertex), nullptr);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RoadVertex),
                              reinterpret_cast<void*>(offsetof(RoadVertex, r)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(RoadVertex),
                              reinterpret_cast<void*>(offsetof(RoadVertex, u)));
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(v.size()));
        glDisableVertexAttribArray(2);
    };
    auto drawQuad = [&](GLuint prog, float x0, float y0, float x1, float y1, float tx0 = 0,
                        float ty0 = 0, float tx1 = 1, float ty1 = 1) {
        const float v[] = {x0, y0, tx0, ty0, x1, y0, tx1, ty0, x1, y1, tx1, ty1,
                           x0, y0, tx0, ty0, x1, y1, tx1, ty1, x0, y1, tx0, ty1};
        glBindBuffer(GL_ARRAY_BUFFER, g.quadVbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof v, v, GL_STREAM_DRAW);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));
        glUseProgram(prog);
        glUniform2f(glGetUniformLocation(prog, "uVideo"), static_cast<float>(W),
                    static_cast<float>(H));
        glDrawArrays(GL_TRIANGLES, 0, 6);
    };
    auto drawMaskGlow = [&](const PixelMask& m, const OverlayItem& it) {
        if (m.w <= 0 || m.h <= 0) return;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, g.maskTex);
        std::vector<std::uint8_t> px(m.bits.size());
        for (std::size_t i = 0; i < px.size(); ++i) px[i] = m.bits[i] ? 255 : 0;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, m.w, m.h, 0, GL_RED, GL_UNSIGNED_BYTE, px.data());
        const float rx = static_cast<float>(m.x * m.cellPx),
                    ry = static_cast<float>(m.y * m.cellPx);
        const float rw = static_cast<float>(m.w * m.cellPx),
                    rh = static_cast<float>(m.h * m.cellPx);
        const float glow = it.style.glowPx > 0 ? it.style.glowPx : 10.0f;
        glUseProgram(g.maskProg);
        glUniform1i(glGetUniformLocation(g.maskProg, "uMask"), 0);
        glUniform4f(glGetUniformLocation(g.maskProg, "uRect"), rx, ry, rw, rh);
        glUniform1f(glGetUniformLocation(g.maskProg, "uGlowPx"), glow);
        glUniform1f(glGetUniformLocation(g.maskProg, "uTime"), t);
        glUniform1f(glGetUniformLocation(g.maskProg, "uShimmerHz"),
                    std::min(it.style.shimmerHz, 3.0f));
        glUniform1f(glGetUniformLocation(g.maskProg, "uOpacity"), it.style.opacity);
        glUniform4f(glGetUniformLocation(g.maskProg, "uColor"), it.style.color.r, it.style.color.g,
                    it.style.color.b, it.style.color.a);
        drawQuad(g.maskProg, rx - glow, ry - glow, rx + rw + glow, ry + rh + glow);
    };
    auto drawImage = [&](const std::string& key, const std::function<cv::Mat()>& make, float cx,
                         float cy, float opacity) {
        auto it = g.textCache.find(key);
        if (it == g.textCache.end()) {
            const cv::Mat img = make();
            GLuint tx;
            glGenTextures(1, &tx);
            glBindTexture(GL_TEXTURE_2D, tx);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.cols, img.rows, 0, GL_BGRA,
                         GL_UNSIGNED_BYTE, img.data);
            it = g.textCache.emplace(key, Gl::Cached{tx, img.cols, img.rows, g.frame}).first;
        }
        it->second.lastUsed = g.frame;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, it->second.tex);
        glUseProgram(g.imageProg);
        glUniform1i(glGetUniformLocation(g.imageProg, "uTex"), 0);
        glUniform1f(glGetUniformLocation(g.imageProg, "uOpacity"), opacity);
        const float hw = it->second.w / 2.0f, hh = it->second.h / 2.0f;
        drawQuad(g.imageProg, cx - hw, cy - hh, cx + hw, cy + hh);
    };
    auto colorKey = [](const Rgba& c) {
        return std::to_string(static_cast<int>(c.r * 255)) + "," +
               std::to_string(static_cast<int>(c.g * 255)) + "," +
               std::to_string(static_cast<int>(c.b * 255));
    };

    for (const OverlayItem* ip : items) {
        const OverlayItem& it = *ip;
        std::vector<RoadVertex> v;
        switch (it.kind) {
            case OverlayKind::ROAD_BAND: {
                const float halo =
                    it.style.glowPx > 0 ? std::max(0.12f, 0.25f * it.widthM) : 0.05f * it.widthM;
                const float outer = it.widthM / 2 + halo;
                tessellateBand(it, cameraFromVehicle_, outer, v);
                drawRoad(v, 0, (it.widthM / 2) / outer, it);
                break;
            }
            case OverlayKind::ROAD_RING: {
                if (it.points.empty() ||
                    cameraZ(cameraFromVehicle_, it.points[0]) < kNearZ + it.widthM)
                    break;
                OverlayItem band = it;
                band.points.clear();
                band.pointColor.clear();
                band.pointOpacity.clear();
                for (int k = 0; k <= 48; ++k) {
                    const double a = 2 * 3.14159265358979 * k / 48;
                    band.points.push_back(it.points[0] +
                                          it.widthM * Eigen::Vector3d(std::cos(a), std::sin(a), 0));
                }
                band.widthM = 0.18f;
                const float outer = 0.09f + 0.12f;
                tessellateBand(band, cameraFromVehicle_, outer, v);
                drawRoad(v, 0, 0.09f / outer, it);
                break;
            }
            case OverlayKind::ROAD_BARRIER: {
                if (it.points.size() < 2) break;
                const Eigen::Vector3d a = it.points[0], b = it.points[1], up(0, 0, it.heightM);
                if (cameraZ(cameraFromVehicle_, a) < kNearZ ||
                    cameraZ(cameraFromVehicle_, b) < kNearZ)
                    break;
                const float L = static_cast<float>((b - a).norm());
                push(v, a, it.style.color, 1, 0, 0);
                push(v, b, it.style.color, 1, L, 0);
                push(v, b + up, it.style.color, 1, L, 1);
                push(v, a, it.style.color, 1, 0, 0);
                push(v, b + up, it.style.color, 1, L, 1);
                push(v, a + up, it.style.color, 1, 0, 1);
                drawRoad(v, 2, 0, it);
                break;
            }
            case OverlayKind::ROAD_ZONE: {
                bool front = it.points.size() >= 3;
                for (const auto& p : it.points)
                    front = front && cameraZ(cameraFromVehicle_, p) >= kNearZ;
                if (!front) break;
                for (std::size_t k = 1; k + 1 < it.points.size(); ++k) {
                    push(v, it.points[0], it.style.color, 1, 0, 0);
                    push(v, it.points[k], it.style.color, 1, 0, 0);
                    push(v, it.points[k + 1], it.style.color, 1, 0, 0);
                }
                drawRoad(v, 1, 0, it);
                break;
            }
            case OverlayKind::ROAD_MARKER: {
                if (it.points.empty()) break;
                // Three chevrons across the lane, pointing towards the car.
                for (int c = 0; c < 3; ++c) {
                    OverlayItem arm = it;
                    const Eigen::Vector3d o = it.points[0] + Eigen::Vector3d(0.9 * c, 0, 0);
                    arm.points = {o + Eigen::Vector3d(0.6, it.widthM / 2, 0), o,
                                  o + Eigen::Vector3d(0.6, -it.widthM / 2, 0)};
                    arm.pointColor.clear();
                    arm.pointOpacity.clear();
                    arm.widthM = 0.25f;
                    std::vector<RoadVertex> av;
                    tessellateBand(arm, cameraFromVehicle_, 0.2f, av);
                    v.insert(v.end(), av.begin(), av.end());
                }
                drawRoad(v, 0, 0.125f / 0.2f, it);
                break;
            }
            case OverlayKind::MASK_GLOW:
                drawMaskGlow(it.mask, it);
                break;
            case OverlayKind::ELLIPSE_GLOW:
                drawMaskGlow(ellipseMaskFor(it.box), it);
                break;
            case OverlayKind::IMAGE_LABEL:
                drawImage(
                    "L|" + it.text + "|" + std::to_string(static_cast<int>(it.sizePx)) + "|" +
                        colorKey(it.style.color),
                    [&] { return rasteriseLabel(it); }, static_cast<float>(it.imagePos.x()),
                    static_cast<float>(it.imagePos.y()), it.style.opacity);
                break;
            case OverlayKind::IMAGE_BADGE:
                drawImage(
                    "B|" + it.text + "|" + std::to_string(static_cast<int>(it.sizePx)),
                    [&] { return rasteriseBadge(it); }, static_cast<float>(it.imagePos.x()),
                    static_cast<float>(it.imagePos.y()), it.style.opacity);
                break;
            case OverlayKind::IMAGE_CHEVRON: {
                // At the screen edge, towards the off-screen hazard.
                const float m = it.sizePx;
                const float x = std::clamp(static_cast<float>(it.imagePos.x()), m, W - m);
                const float y = std::clamp(static_cast<float>(it.imagePos.y()), m, H - m);
                drawImage(
                    "C|" + std::to_string(static_cast<int>(it.sizePx)) + "|" +
                        colorKey(it.style.color),
                    [&] { return rasteriseChevron(it); }, x, y, it.style.opacity);
                break;
            }
        }
    }
    glEndQuery(GL_TIME_ELAPSED);

    // Scale the framebuffer into the window, letterboxed.
    glBindFramebuffer(GL_READ_FRAMEBUFFER, g.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    int ww = 0, wh = 0;
    SDL_GL_GetDrawableSize(window_, &ww, &wh);
    glViewport(0, 0, ww, wh);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    const double s = std::min(static_cast<double>(ww) / W, static_cast<double>(wh) / H);
    const int dw = static_cast<int>(W * s), dh = static_cast<int>(H * s);
    const int dx = (ww - dw) / 2, dy = (wh - dh) / 2;
    glBlitFramebuffer(0, 0, W, H, dx, dy, dx + dw, dy + dh, GL_COLOR_BUFFER_BIT,
                      (dw == W && dh == H) ? GL_NEAREST : GL_LINEAR);
    SDL_GL_SwapWindow(window_);

    // Timings from the frame two presents ago (their queries are finished by now).
    ++g.queryFrame;
    if (g.queryFrame >= 3) {
        GLuint* old = g.queries[g.queryFrame % 3];
        GLuint64 up = 0, ov = 0;
        glGetQueryObjectui64v(old[0], GL_QUERY_RESULT, &up);
        glGetQueryObjectui64v(old[1], GL_QUERY_RESULT, &ov);
        timings_.uploadMs = up / 1e6;
        timings_.overlayMs = ov / 1e6;
    }
    // Forget cached label textures not used for 120 frames.
    for (auto it = g.textCache.begin(); it != g.textCache.end();) {
        if (g.frame - it->second.lastUsed > 120) {
            glDeleteTextures(1, &it->second.tex);
            it = g.textCache.erase(it);
        } else {
            ++it;
        }
    }
    ++g.frame;
    timings_.presentCpuMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpu0).count();
}

cv::Mat WindowedSink::capture() {
    Gl& g = *gl_;
    if (!g.videoW) return {};
    cv::Mat img(g.videoH, g.videoW, CV_8UC3);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, g.fbo);
    glReadPixels(0, 0, g.videoW, g.videoH, GL_BGR, GL_UNSIGNED_BYTE, img.data);
    cv::flip(img, img, 0);  // GL rows run bottom-up
    return img;
}

}  // namespace ar_drive_assist
