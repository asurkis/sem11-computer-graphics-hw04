#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <string_view>
#include <tuple>
#include <vector>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

////////////////////////////////////////////////////////////////

using RNG = std::default_random_engine;

using Real = float;
using Vec2 = glm::vec<2, Real>;
using Vec3 = glm::vec<3, Real>;
using Vec4 = glm::vec<4, Real>;
using Vec2i = glm::vec<2, int>;
using Vec3i = glm::vec<3, int>;
using Vec4i = glm::vec<4, int>;

constexpr Real ZERO = Real(0.);
constexpr Real ONE = Real(1.);
constexpr Real TWO = Real(2.);
constexpr Real HALF = Real(.5);

////////////////////////////////////////////////////////////////

constexpr size_t PIC_WIDTH = 1920;
constexpr size_t PIC_HEIGHT = 1080;
constexpr int PIXEL_MAX = 255;

constexpr int MAX_BOUNCES = 32;
constexpr int AA_SAMPLES = 32;

const Vec3 CAM_ORIGIN = {-1., 3., 3.};
const Vec3 CAM_LOOK_AT = {0., 1., 0.};
const Vec3 WORLD_UP = {0., 1., 0.};

const Vec3 SUN_DIR = glm::normalize(Vec3(.5, .1, -1.));
const Vec3 SUNLIGHT_COLOR = {2.5, 2., 1.5};
constexpr Real FUZZINESS = .005;

constexpr Real GAMMA = 2.;
constexpr Real EPSILON = 1e-4;

////////////////////////////////////////////////////////////////

const Vec3 CAM_FORWARD = glm::normalize(CAM_LOOK_AT - CAM_ORIGIN);
const Vec3 CAM_RIGHT = glm::normalize(glm::cross(CAM_FORWARD, WORLD_UP));
const Vec3 CAM_UP = glm::normalize(glm::cross(CAM_RIGHT, CAM_FORWARD));

////////////////////////////////////////////////////////////////

struct HitPoint {
    Vec3 Position;
    Vec3 Normal;
    Vec2 TexCoord;
    Real Distance;
};

struct HitMaterial {
    Vec3 BaseColor;
    Vec3 Emission;
    Real ProbReflect;
};

struct HitFull {
    HitPoint Point;
    HitMaterial Material;
    bool Found;
};

////////////////////////////////////////////////////////////////

struct Sphere {
    Vec3 Center;
    Real Radius;
    HitMaterial Material;
};

constexpr Sphere SPHERES[] = {
    {{2., 1., 2.}, 1., {Vec3(.25), {}, 0.}},
};

////////////////////////////////////////////////////////////////

constexpr HitMaterial MODEL_MATERIAL = {Vec3(1., .75, .5), Vec3(.01), 0.};

struct Model {
    std::vector<Vec3> Vertices;
    std::vector<Vec2> TexCoords;
    std::vector<std::tuple<Vec2i, Vec2i, Vec2i>> Faces;
};

Model model;

////////////////////////////////////////////////////////////////

Vec3 RandomNormal3(RNG &rng) noexcept {
    // std::uniform_real_distribution<Real> uniform(-1.);
    // for (;;) {
    //     Vec3 vec;
    //     vec.x = uniform(rng);
    //     vec.y = uniform(rng);
    //     vec.z = uniform(rng);
    //     if (glm::dot(vec, vec) <= ONE) return glm::normalize(vec);
    // }
    std::normal_distribution<Real> distrib;
    Vec3 vec;
    vec.x = distrib(rng);
    vec.y = distrib(rng);
    vec.z = distrib(rng);
    return glm::normalize(vec);
}

Vec3 DebugNormal(Vec3 norm) noexcept { return norm * HALF + HALF; }

////////////////////////////////////////////////////////////////

// Всегда отсекаем внутренние поверхности,
// они мешают, когда отражаем лучи.

bool ScanSphereO(HitPoint &out, Vec3 dir, Vec3 oa, Real radius) noexcept {
    // O --- центр координат
    // A --- центр сферы
    // H --- проекция центра сферы на луч
    // X --- точка пересечения луча со сферой
    // X лежит на OH
    // OHA --- прямой угол
    Vec3 oh = dir * glm::dot(dir, oa);
    Vec3 ah = oh - oa;
    Real ah2 = glm::dot(ah, ah);
    Real r2 = radius * radius;
    Real hx2 = r2 - ah2;
    if (hx2 < ZERO) return false;

    Real oxLen = glm::length(oh) - glm::sqrt(hx2);
    if (oxLen < ZERO) return false;

    Vec3 ox = dir * oxLen;
    Vec3 ax = ox - oa;
    if (glm::dot(ax, dir) >= ZERO) return false;

    Vec3 norm = glm::normalize(ax);

    out.Position = ox;
    out.Normal = norm;
    out.Distance = oxLen;

    return true;
}

bool ScanSphere(HitPoint &out, Vec3 origin, Vec3 dir, Vec3 center, Real radius) noexcept {
    bool hitFound = ScanSphereO(out, dir, center - origin, radius);
    if (hitFound) out.Position += origin;
    return hitFound;
}

bool ScanPlaneO(
    HitPoint &out, Vec3 dir, Vec3 oa, Vec3 planeX, Vec3 planeY, Vec3 planeZ, bool denormalized = false) noexcept {
    // O --- центр координат
    // A --- точка на плоскости
    // X --- точка пересечения луча с плоскостью
    Real dirDotNorm = glm::dot(dir, planeY);
    if (dirDotNorm >= ZERO) return false;

    Real distToPlane = glm::dot(oa, planeY);
    if (distToPlane >= ZERO) return false;

    out.Distance = distToPlane / dirDotNorm;
    out.Position = dir * out.Distance;
    out.Normal = planeY;

    Vec3 ax = out.Position - oa;
    if (denormalized) {
        // TODO: nepravda, fix this
        Real x2 = glm::dot(planeX, planeX);
        Real z2 = glm::dot(planeZ, planeZ);
        Real xz = glm::dot(planeX, planeZ);
        Vec3 tauZ = xz / x2 * planeX;
        Vec3 orthoZ = planeZ - tauZ;
        out.TexCoord.y = glm::dot(ax, orthoZ) / glm::dot(orthoZ, orthoZ);
        // ax = planeX * x + planeZ * y
        // planeX * x = ax - planeZ * y
        // planeX^2 * x = dot(ax - planeZ * y, planeX)
        out.TexCoord.x = glm::dot(ax - planeZ * out.TexCoord.y, planeX) / x2;
    } else {
        out.TexCoord.x = glm::dot(ax, planeX);
        out.TexCoord.y = glm::dot(ax, planeZ);
        return true;
    }

    return true;
}

bool ScanPlane(HitPoint &out,
               Vec3 origin,
               Vec3 dir,
               Vec3 planeOrigin,
               Vec3 planeX,
               Vec3 planeY,
               Vec3 planeZ,
               bool denormalized = false) noexcept {
    bool hitFound = ScanPlaneO(out, dir, planeOrigin - origin, planeX, planeY, planeZ, denormalized);
    if (hitFound) out.Position += origin;
    return hitFound;
}

bool ScanTriangle(HitPoint &out, Vec3 origin, Vec3 dir, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 norm = glm::cross(ab, ac);
    if (glm::dot(norm, norm) < EPSILON) return false;
    norm = glm::normalize(norm);
    bool found = ScanPlane(out, origin, dir, a, ab, norm, ac, true);
    if (!found) return false;
    if (out.TexCoord.x < ZERO) return false;
    if (out.TexCoord.y < ZERO) return false;
    if (out.TexCoord.x + out.TexCoord.y > ONE) return false;
    return true;
}

////////////////////////////////////////////////////////////////

void HitSelect(HitFull &lhsHit, const HitFull &rhsHit) noexcept {
    if (!rhsHit.Found) return;
    if (!lhsHit.Found || rhsHit.Point.Distance < lhsHit.Point.Distance) { lhsHit = rhsHit; }
}

void ScanScene(HitFull &bestHit, Vec3 origin, Vec3 dir) noexcept {
    for (auto &sphere : SPHERES) {
        HitFull hit;
        hit.Found = ScanSphere(hit.Point, origin, dir, sphere.Center, sphere.Radius);
        hit.Material = sphere.Material;
        HitSelect(bestHit, hit);
    }

    for (auto &face : model.Faces) {
        HitFull hit;
        auto [ai, bi, ci] = face;
        Vec3 a = model.Vertices[ai.x - 1];
        Vec3 b = model.Vertices[bi.x - 1];
        Vec3 c = model.Vertices[ci.x - 1];
        hit.Found = ScanTriangle(hit.Point, origin, dir, a, b, c);
        hit.Material = MODEL_MATERIAL;
        HitSelect(bestHit, hit);
    }

    HitFull hitFloor;
    hitFloor.Found = ScanPlane(hitFloor.Point, origin, dir, {0., -1., 0.}, {1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.});
    if (hitFloor.Found) {
        Vec2i ij = glm::floor(hitFloor.Point.TexCoord);
        if ((ij.x + ij.y) % 2 == 0) {
            hitFloor.Material.BaseColor = Vec3(.8);
        } else {
            hitFloor.Material.BaseColor = Vec3(.2);
        }
        hitFloor.Material.Emission = {};
        hitFloor.Material.ProbReflect = 0.;
    }
    HitSelect(bestHit, hitFloor);
}

////////////////////////////////////////////////////////////////

Vec3 Sky(Vec3 dir) noexcept {
    constexpr Vec3 SKY_COLORS[] = {
        Vec3(.05, .05, .25),
        Vec3(.75, .25, .5),
        Vec3(.9, .9, .6),
        Vec3(1., 1., .99),
        Vec3(5., 5., 3.),
    };
    constexpr Real SKY_BORDER[] = {
        -1.,
        0.,
        .9,
        .99,
        2., // > 1 на случай, если косинус строго равен 1
    };
    constexpr size_t COLOR_COUNT = sizeof(SKY_COLORS) / sizeof(SKY_COLORS[0]);
    static_assert(sizeof(SKY_BORDER) / sizeof(SKY_BORDER[0]) == COLOR_COUNT);

    Real cosTh = glm::dot(SUN_DIR, dir);
    for (size_t i = 0; i < COLOR_COUNT; ++i) {
        Real thresh1 = SKY_BORDER[i + 1];
        if (cosTh <= thresh1) {
            Vec3 col0 = SKY_COLORS[i];
            Vec3 col1 = SKY_COLORS[i + 1];
            Real thresh0 = SKY_BORDER[i];
            Real delta = thresh1 - thresh0;
            return glm::mix(col0, col1, (cosTh - thresh0) / delta);
        }
    }
    return SKY_COLORS[COLOR_COUNT - 1];
}

Vec3 CastRay(RNG &rng, Vec3 origin, Vec3 dir) noexcept {
    std::uniform_real_distribution<Real> uniform;

    Vec3 accPixelColor = {};
    Vec3 accMaterialBaseColor(ONE);
    for (int i = 0; i < MAX_BOUNCES; ++i) {
        HitFull hit = {};
        ScanScene(hit, origin, dir);
        if (!hit.Found) break;

        Vec3 base = hit.Material.BaseColor;
        accPixelColor += accMaterialBaseColor * hit.Material.Emission;
        accMaterialBaseColor *= hit.Material.BaseColor;

        origin = hit.Point.Position;
        if (uniform(rng) < hit.Material.ProbReflect) {
            dir = glm::reflect(dir, hit.Point.Normal);
        } else {
            // Проверим, освещены ли мы солнцем
            HitFull hitSun = {};
            Vec3 fuzzySunDir = glm::normalize(SUN_DIR + FUZZINESS * RandomNormal3(rng));
            ScanScene(hitSun, origin, fuzzySunDir);
            if (!hitSun.Found) {
                // Ничто не загораживает солнце
                Real coef = glm::dot(hit.Point.Normal, fuzzySunDir);
                coef = glm::max(coef, ZERO);
                accPixelColor += coef * accMaterialBaseColor * SUNLIGHT_COLOR;
            }

            dir = hit.Point.Normal + RandomNormal3(rng);
            dir = glm::normalize(dir);
        }
    }
    accPixelColor += accMaterialBaseColor * Sky(dir);
    return accPixelColor;
}

////////////////////////////////////////////////////////////////

void ReadModel() {
    model.Vertices.clear();
    model.TexCoords.clear();
    model.Faces.clear();

    // Плюсовые стримы не настолько удобны,
    // если разделитель не пробел.
    // Как-нибудь переживём отсутствие RAII в одном месте.
    FILE *f = fopen("model.obj", "r");
    while (!feof(f)) {
        char type[4];
        if (fscanf(f, "%3s", type) != 1) break;
        if (std::string_view(type) == "v") {
            glm::vec3 vertex;
            if (fscanf(f, "%f %f %f", &vertex.x, &vertex.y, &vertex.z) != 3) break;
            model.Vertices.push_back(vertex);
        } else if (std::string_view(type) == "vt") {
            glm::vec2 textureCoord;
            if (fscanf(f, "%f %f", &textureCoord.x, &textureCoord.y) != 2) break;
            model.TexCoords.push_back(textureCoord);
        } else if (std::string_view(type) == "f") {
            std::tuple<Vec2i, Vec2i, Vec2i> face;
            if (fscanf(f,
                       "%d/%d %d/%d %d/%d",
                       &std::get<0>(face).x,
                       &std::get<0>(face).y,
                       &std::get<1>(face).x,
                       &std::get<1>(face).y,
                       &std::get<2>(face).x,
                       &std::get<2>(face).y)
                != 6)
                break;
            model.Faces.push_back(face);
        }
    }
    fclose(f);

    std::cout << "Read model with:\n- " << model.Vertices.size() << " vertices\n- " << model.TexCoords.size()
              << " texture coords\n- " << model.Faces.size() << " faces\n";
}

////////////////////////////////////////////////////////////////

Vec3 outPicture[PIC_HEIGHT][PIC_WIDTH];

void GeneratePicture() noexcept {
    constexpr Real ASPECT_RATIO = Real(PIC_WIDTH) / Real(PIC_HEIGHT);
    constexpr Real SAMPLE_WEIGHT = ONE / AA_SAMPLES;

    std::atomic<size_t> rowsCompleted = 0;

#pragma omp parallel for
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        RNG rng(row);
        std::uniform_real_distribution<Real> uniform;

        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            Vec3 result = {};
            for (int sample = 0; sample < AA_SAMPLES; ++sample) {
                Vec2 xy = Vec2(col, row);
                xy.x += uniform(rng);
                xy.y += uniform(rng);
                // xy += HALF;
                xy /= Vec2(PIC_WIDTH, PIC_HEIGHT);
                xy = xy * TWO - ONE;
                xy *= Vec2(ASPECT_RATIO, -ONE);

                Vec3 viewDir = xy.x * CAM_RIGHT + xy.y * CAM_UP + CAM_FORWARD;
                viewDir = glm::normalize(viewDir);
                result += SAMPLE_WEIGHT * CastRay(rng, CAM_ORIGIN, viewDir);
            }
            outPicture[row][col] = result;
        }
        size_t completed = rowsCompleted.fetch_add(1);
        if (++completed % 10 == 0) { std::cout << completed << " rows completed\n"; }
    }
}

void WritePicture() {
    constexpr Vec3 GAMMA_INV_V = Vec3(1. / GAMMA);
    std::ofstream fout("out.ppm");
    fout << "P3 " << PIC_WIDTH << ' ' << PIC_HEIGHT << ' ' << PIXEL_MAX << '\n';
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            Vec3 pixel = outPicture[row][col];
            pixel = glm::pow(pixel, GAMMA_INV_V);
            for (size_t chan = 0; chan < 3; ++chan) {
                int asInt = int(pixel[chan] * (PIXEL_MAX + 1));
                asInt = std::min(std::max(asInt, 0), PIXEL_MAX);
                fout << asInt << ' ';
            }
        }
        fout << '\n';
    }
}

int main() {
    ReadModel();
    GeneratePicture();
    WritePicture();
}
