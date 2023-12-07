#include <cstddef>
#include <fstream>

using Float = float;

constexpr size_t PIC_WIDTH = 640;
constexpr size_t PIC_HEIGHT = 480;
constexpr size_t PIC_CHANNELS = 3;

Float picture[PIC_HEIGHT][PIC_WIDTH][PIC_CHANNELS];

void GeneratePicture() {
  for (size_t row = 0; row < PIC_HEIGHT; ++row) {
    for (size_t col = 0; col < PIC_WIDTH; ++col) {
      Float x = 2.0 * col / PIC_WIDTH - 1.0;
      Float y = 2.0 * row / PIC_HEIGHT - 1.0;
      x = x * PIC_WIDTH / PIC_HEIGHT;

      Float r2 = x * x + y * y;
      for (size_t chan = 0; chan < PIC_CHANNELS; ++chan) {
        picture[row][col][chan] = r2 < 1.0 ? 1.0 : 0.0;
      }
    }
  }
}

void WritePicture() {
  std::ofstream fout("out.ppm");
  fout << "P3 " << PIC_WIDTH << ' ' << PIC_HEIGHT << " 1\n";
  for (size_t row = 0; row < PIC_HEIGHT; ++row) {
    for (size_t col = 0; col < PIC_WIDTH; ++col) {
      for (size_t chan = 0; chan < PIC_CHANNELS; ++chan) {
        if (picture[row][col][chan] < 0.5) {
          fout << "0 ";
        } else {
          fout << "1 ";
        }
      }
    }
    fout << '\n';
  }
}

int main() {
  GeneratePicture();
  WritePicture();
}
