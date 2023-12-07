#include <cmath>
#include <cstddef>
#include <fstream>
#include <random>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

////////////////////////////////////////////////////////////////

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

constexpr size_t PIC_WIDTH = 1280;
constexpr size_t PIC_HEIGHT = 720;
constexpr int PIXEL_MAX = 255;

constexpr int MAX_REFLECTIONS = 4;
constexpr int AA_SAMPLES = 4;

const Vec3 CAM_ORIGIN = {0., 1., 5.};
const Vec3 CAM_LOOK_AT = {0., 2., 0.};
const Vec3 WORLD_UP = {0., 1., 0.};

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
    Real Reflectivity;
};

struct HitFull {
    HitPoint Point;
    HitMaterial Material;
    bool Found;
};

////////////////////////////////////////////////////////////////

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
    Vec3 ox = dir * oxLen;
    Vec3 ax = ox - oa;
    Vec3 norm = glm::normalize(ax);

    if (oxLen < ZERO) return false;
    if (glm::dot(norm, dir) >= ZERO) return false;

    out.Position = ox;
    out.Normal = norm;
    out.Distance = oxLen;

    return true;
}

bool ScanSphere(HitPoint &out, Vec3 origin, Vec3 dir, Vec3 center,
                Real radius) noexcept {
    bool hitFound = ScanSphereO(out, dir, center - origin, radius);
    if (hitFound) out.Position += origin;
    return hitFound;
}

bool ScanPlaneO(HitPoint &out, Vec3 dir, Vec3 oa, Vec3 planeX, Vec3 planeY,
                Vec3 planeZ) noexcept {
    // O --- центр координат
    // A --- точка на плоскости
    // X --- точка пересечения луча с плоскостью
    Real dirDotNorm = glm::dot(dir, planeY);
    if (dirDotNorm >= ZERO) return false;

    out.Distance = -ONE / dirDotNorm;
    out.Position = dir * out.Distance;
    out.Normal = planeY;

    Vec3 ax = out.Position - oa;
    out.TexCoord.x = glm::dot(ax, planeX);
    out.TexCoord.y = glm::dot(ax, planeZ);

    return true;
}

bool ScanPlane(HitPoint &out, Vec3 origin, Vec3 dir, Vec3 planeOrigin,
               Vec3 planeX, Vec3 planeY, Vec3 planeZ) noexcept {
    bool hitFound
        = ScanPlaneO(out, dir, planeOrigin - origin, planeX, planeY, planeZ);
    if (hitFound) out.Position += origin;
    return hitFound;
}

////////////////////////////////////////////////////////////////

void HitJoin(HitFull &lhsHit, const HitFull &rhsHit) noexcept {
    if (!rhsHit.Found) return;
    if (!lhsHit.Found || rhsHit.Point.Distance < lhsHit.Point.Distance) {
        lhsHit = rhsHit;
    }
}

void ScanScene(HitFull &bestHit, Vec3 origin, Vec3 dir) noexcept {
    for (int i = -2; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = -2; k < 2; ++k) {
                Vec3 pos = Real(3.) * (Vec3(i, j, k) + HALF);
                HitFull hit;
                hit.Found = ScanSphere(hit.Point, origin, dir, pos, ONE);
                hit.Material.BaseColor = Vec3(.25);
                hit.Material.Reflectivity = .75;
                HitJoin(bestHit, hit);
            }
        }
    }

    HitFull floorHit;
    floorHit.Found = ScanPlane(floorHit.Point, origin, dir, {}, {1., 0., 0.},
                               {0., 1., 0.}, {0., 0., 1.});
    if (floorHit.Found) {
        Vec2i ij = glm::floor(floorHit.Point.TexCoord);
        if ((ij.x + ij.y) % 2 == 0) {
            floorHit.Material.BaseColor = Vec3(.8);
        } else {
            floorHit.Material.BaseColor = Vec3(.2);
        }
        floorHit.Material.Reflectivity = .25;
    }
    HitJoin(bestHit, floorHit);
}

////////////////////////////////////////////////////////////////

Vec3 Sky(Vec3 dir) noexcept {
    // Для отладки покажем нормаль
    // Real max0 = std::max(ZERO, dir.y);
    // Real gray = max0 * max0;
    // return {gray, gray, gray};
    return DebugNormal(dir);
}

Vec3 CastRay(Vec3 origin, Vec3 dir) noexcept {
    Vec3 acc = {};
    Real weight = ONE;
    for (int i = 0; i < MAX_REFLECTIONS; ++i) {
        HitFull hit = {};
        ScanScene(hit, origin, dir);
        if (!hit.Found) break;

        Vec3 base = hit.Material.BaseColor;
        Real reflectivity = hit.Material.Reflectivity;
        acc += weight * (ONE - reflectivity) * base;
        weight *= reflectivity;

        origin = hit.Point.Position;
        dir = glm::reflect(dir, hit.Point.Normal);
    }
    acc += weight * Sky(dir);
    return acc;
}

////////////////////////////////////////////////////////////////

Vec3 picture[PIC_HEIGHT][PIC_WIDTH];

void GeneratePicture() noexcept {
    constexpr Real ASPECT_RATIO = Real(PIC_WIDTH) / Real(PIC_HEIGHT);
    constexpr Real SAMPLE_WEIGHT = ONE / AA_SAMPLES;

#pragma omp parallel for
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        std::default_random_engine rng;
        std::uniform_real_distribution<Real> uniform;

        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            Vec3 result = {};
            for (int i = 0; i < AA_SAMPLES; ++i) {
                Vec2 xy = Vec2(col, row);
                xy.x += uniform(rng);
                xy.y += uniform(rng);
                // xy += HALF;
                xy /= Vec2(PIC_WIDTH, PIC_HEIGHT);
                xy = xy * TWO - ONE;
                xy *= Vec2(ASPECT_RATIO, -ONE);

                Vec3 viewDir = xy.x * CAM_RIGHT + xy.y * CAM_UP + CAM_FORWARD;
                viewDir = glm::normalize(viewDir);
                result += SAMPLE_WEIGHT * CastRay(CAM_ORIGIN, viewDir);
            }
            picture[row][col] = result;
        }
    }
}

void WritePicture() {
    std::ofstream fout("out.ppm");
    fout << "P3 " << PIC_WIDTH << ' ' << PIC_HEIGHT << ' ' << PIXEL_MAX << '\n';
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            for (size_t chan = 0; chan < 3; ++chan) {
                int asInt = int(picture[row][col][chan] * (PIXEL_MAX + 1));
                asInt = std::min(std::max(asInt, 0), PIXEL_MAX);
                fout << asInt << ' ';
            }
        }
        fout << '\n';
    }
}

int main() {
    GeneratePicture();
    WritePicture();
}
