#include <cmath>
#include <cstddef>
#include <fstream>
#include <optional>
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

struct Hit {
    Vec3 point;
    Vec3 normal;
    Real distance;
};

////////////////////////////////////////////////////////////////

Vec3 DebugNormal(Vec3 norm) { return norm * HALF + HALF; }

////////////////////////////////////////////////////////////////

std::optional<Hit> ScanSphereO(Vec3 dir, Vec3 oa, Real radius) {
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
    if (hx2 < ZERO) return {};

    Real oxLen = glm::length(oh) - glm::sqrt(hx2);
    Vec3 ox = dir * oxLen;
    Vec3 ax = ox - oa;
    Vec3 norm = glm::normalize(ax);

    if (oxLen < ZERO) return {};

    return {{ox, norm, oxLen}};
}

std::optional<Hit> ScanSphere(Vec3 origin, Vec3 dir, Vec3 center, Real radius) {
    std::optional<Hit> hit = ScanSphereO(dir, center - origin, radius);
    if (hit.has_value()) { hit->point += origin; }
    return hit;
}

////////////////////////////////////////////////////////////////

const Vec3 CAM_ORIGIN = {4., 4., 4.};
const Vec3 CAM_LOOK_AT = {0., 0., 0.};
const Vec3 WORLD_UP = {0., 1., 0.};

const Vec3 CAM_FORWARD = glm::normalize(CAM_LOOK_AT - CAM_ORIGIN);
const Vec3 CAM_RIGHT = glm::normalize(glm::cross(CAM_FORWARD, WORLD_UP));
const Vec3 CAM_UP = glm::normalize(glm::cross(CAM_RIGHT, CAM_FORWARD));

std::optional<Hit> ScanScene(Vec3 origin, Vec3 dir) {
    std::optional<Hit> bestHit;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                std::optional<Hit> hit = ScanSphere(origin, dir, TWO * Vec3(i, j, k), 1);
                if (!hit) continue;
                if (!bestHit || bestHit->distance > hit->distance) {
                    bestHit = hit;
                }
            }
        }
    }
    return bestHit;
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
    std::optional<Hit> hit = ScanScene(origin, dir);
    if (!hit) return Sky(dir);
    return DebugNormal(hit->normal);
}

////////////////////////////////////////////////////////////////

constexpr size_t PIC_WIDTH = 640;
constexpr size_t PIC_HEIGHT = 480;

Vec3 picture[PIC_HEIGHT][PIC_WIDTH];

void GeneratePicture() {
    constexpr Real ASPECT_RATIO = Real(PIC_WIDTH) / Real(PIC_HEIGHT);
    constexpr int SAMPLES = 1;
    constexpr Real SAMPLE_WEIGHT = ONE / SAMPLES;

#pragma omp parallel for
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        std::default_random_engine rng;
        std::uniform_real_distribution<Real> uniform;

        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            Vec3 result = {};
            for (int i = 0; i < SAMPLES; ++i) {
                Vec2 xy = Vec2(col, row);
                // xy.x += uniform(rng);
                // xy.y += uniform(rng);
                xy += HALF;
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
    fout << "P3 " << PIC_WIDTH << ' ' << PIC_HEIGHT << " 255\n";
    for (size_t row = 0; row < PIC_HEIGHT; ++row) {
        for (size_t col = 0; col < PIC_WIDTH; ++col) {
            for (size_t chan = 0; chan < 3; ++chan) {
                int asInt = int(picture[row][col][chan] * 256);
                asInt = std::min(std::max(asInt, 0), 255);
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
