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

constexpr Real ZERO = Real(0.);
constexpr Real ONE = Real(1.);
constexpr Real TWO = Real(2.);
constexpr Real HALF = Real(.5);

////////////////////////////////////////////////////////////////

constexpr size_t PIC_WIDTH = 1280;
constexpr size_t PIC_HEIGHT = 720;
constexpr int PIXEL_MAX = 255;

constexpr int MAX_REFLECTIONS = 16;
constexpr int AA_SAMPLES = 16;

const Vec3 CAM_ORIGIN = {0., 0., 5.};
const Vec3 CAM_LOOK_AT = {0., 0., 0.};
const Vec3 WORLD_UP = {0., 1., 0.};

////////////////////////////////////////////////////////////////

const Vec3 CAM_FORWARD = glm::normalize(CAM_LOOK_AT - CAM_ORIGIN);
const Vec3 CAM_RIGHT = glm::normalize(glm::cross(CAM_FORWARD, WORLD_UP));
const Vec3 CAM_UP = glm::normalize(glm::cross(CAM_RIGHT, CAM_FORWARD));

////////////////////////////////////////////////////////////////

struct Hit {
    Vec3 point;
    Vec3 normal;
    Real distance;
};

////////////////////////////////////////////////////////////////

Vec3 DebugNormal(Vec3 norm) { return norm * HALF + HALF; }

////////////////////////////////////////////////////////////////

bool ScanSphereO(Hit &outHit, Vec3 dir, Vec3 oa, Real radius) {
    // O --- центр координат
    // A --- центр сферы
    // H --- проекция центра сферы на луч
    // X --- точка пересечения луча со сферой
    // dot(OH, AH) = 0
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
    if (glm::dot(norm, dir) > ZERO) return false;

    outHit.point = ox;
    outHit.normal = norm;
    outHit.distance = oxLen;
    return true;
}

bool ScanSphere(Hit &outHit, Vec3 origin, Vec3 dir, Vec3 center, Real radius) {
    bool hitFound = ScanSphereO(outHit, dir, center - origin, radius);
    outHit.point += origin;
    return hitFound;
}

////////////////////////////////////////////////////////////////

bool ScanScene(Hit &bestHit, Vec3 origin, Vec3 dir) {
    bool bestHitFound = false;
    for (int i = -2; i < 2; ++i) {
        for (int j = -2; j < 2; ++j) {
            for (int k = -2; k < 2; ++k) {
                Vec3 pos = Real(3.) * (Vec3(i, j, k) + HALF);
                Hit hit;
                bool hitFound = ScanSphere(hit, origin, dir, pos, ONE);
                if (!hitFound) continue;
                if (!bestHitFound || bestHit.distance > hit.distance) {
                    bestHit = hit;
                    bestHitFound = true;
                }
            }
        }
    }
    return bestHitFound;
}

////////////////////////////////////////////////////////////////

Vec3 Sky(Vec3 dir) {
    // Для отладки покажем нормаль
    // Real max0 = std::max(ZERO, dir.y);
    // Real gray = max0 * max0;
    // return {gray, gray, gray};
    return DebugNormal(dir);
}

Vec3 CastRay(Vec3 origin, Vec3 dir) {
    Vec3 acc = {};
    Real weight = ONE;
    for (int i = 0; i < MAX_REFLECTIONS; ++i) {
        Hit hit;
        bool hitFound = ScanScene(hit, origin, dir);
        if (!hitFound) break;

        Real reflectivity = .75;
        acc += weight * (1. - reflectivity) * .25;
        weight *= reflectivity;

        origin = hit.point;
        dir = glm::reflect(dir, hit.normal);
    }
    acc += weight * Sky(dir);
    return acc;
}

////////////////////////////////////////////////////////////////

Vec3 picture[PIC_HEIGHT][PIC_WIDTH];

void GeneratePicture() {
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
