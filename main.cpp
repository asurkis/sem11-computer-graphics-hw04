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

std::optional<Hit> ScanSphere(Vec3 dir, Vec3 center, Real r) {
    // O --- центр координат
    // A --- центр сферы
    // H --- проекция центра сферы на луч
    // X --- точка пересечения луча со сферой
    // dot(OH, AH) = 0
    Vec3 oh = dir * glm::dot(dir, center);
    Vec3 ah = oh - center;
    Real ah2 = glm::dot(ah, ah);
    Real r2 = r * r;
    Real hx2 = r2 - ah2;
    if (hx2 < ZERO) return {};

    Real oxLen = glm::length(oh) - glm::sqrt(hx2);
    Vec3 ox = dir * oxLen;
    Vec3 ax = ox - center;
    Vec3 norm = glm::normalize(ax);

    return {{ox, norm, oxLen}};
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
    std::optional<Hit> hit = ScanSphere(dir, Vec3() - origin, ONE);
    if (!hit.has_value()) return Sky(dir);
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

                Vec3 origin = {ZERO, ZERO, Real(3.)};
                Vec3 viewDir = glm::normalize(Vec3(xy, -ONE));
                result += SAMPLE_WEIGHT * CastRay(origin, viewDir);
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
