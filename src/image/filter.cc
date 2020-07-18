#include "filter.h"

void Filter::Gaussian(GrayImage& image, GrayImage& filtered_image,
                       std::size_t size, float sigma) {
  filtered_image.Reset(image.width(), image.height());

  std::vector<float> filter_buffer = CreateGaussianFilterBuffer(size, sigma);
  int filter_center = int(roundf(float(size - 1) / 2.0f));

  int image_width = image.width();
  int image_height = image.height();

  auto y_end = image_height - int(size);
  auto x_end = image_width - int(size);

  auto image_ptr = image.buffer();
  auto filtered_ptr =
      filtered_image.buffer() + (image_width * filter_center + filter_center);

  for (auto y = 0; y <= y_end; ++y) {
    for (auto x = 0; x <= x_end; ++x) {
      float value = 0.0f;

      for (auto fy = 0; fy < size; ++fy) {
        std::size_t image_offset = image_width * (y + fy) + x;
        std::size_t filter_offset = size * fy;
        for (auto fx = 0; fx < size; ++fx) {
          value +=
              image_ptr[image_offset + fx] * filter_buffer[filter_offset + fx];
        }
      }

      filtered_ptr[image_width * y + x] =
          unsigned char(std::min(std::max(int(value), 0), 255));
    }
  }

  auto margin_y = image_height - filter_center;
  auto margin_x = image_width - filter_center;

  filtered_ptr = filtered_image.buffer();

  for (auto y = 0; y < image_height; ++y) {
    for (auto x = 0; x < image_width; ++x) {
      if (filter_center <= y && y < margin_y && filter_center <= x &&
          x < margin_x) {
        continue;
      }

      float value = 0.0f;
      for (int fy = -filter_center, fyy = 0; fy < -filter_center + int(size);
           ++fy, ++fyy) {
        for (int fx = -filter_center, fxx = 0; fx < -filter_center + int(size);
             ++fx, ++fxx) {
          int image_x = std::min(std::max(x + fx, 0), int(image_width - 1));
          int image_y = std::min(std::max(y + fy, 0), int(image_height - 1));

          value += image_ptr[(image_width * (image_y)) + image_x] *
                   filter_buffer[(size * fyy) + fxx];
        }
      }

      filtered_ptr[image_width * y + x] =
          unsigned char(std::min(std::max(int(value), 0), 255));
    }
  }
}

void Filter::Sobel(GrayImage& image, IntImage& gx, IntImage& gy,
                    FloatImage& magnitude) {
  int image_width = image.width();
  int image_height = image.height();

  gx.Reset(image_width, image_height);
  gy.Reset(image_width, image_height);
  magnitude.Reset(image_width, image_height);

  int y_end = image_height - 1;
  int x_end = image_width - 1;

  auto gx_ptr = gx.buffer();
  auto gy_ptr = gy.buffer();
  auto mag_ptr = magnitude.buffer();
  auto image_ptr = image.buffer();

  for (int y = 1; y < y_end; y++) {
    int y_offset = image_width * y;
    for (int x = 1; x < x_end; x++) {
      int cur_offset = y_offset + x;

      auto current_image_ptr = image_ptr + cur_offset;

      int p00 = current_image_ptr[-image_width - 1];
      int p01 = current_image_ptr[-image_width];
      int p02 = current_image_ptr[-image_width + 1];
      int p10 = current_image_ptr[-1];
      int p12 = current_image_ptr[1];
      int p20 = current_image_ptr[image_width - 1];
      int p21 = current_image_ptr[image_width];
      int p22 = current_image_ptr[image_width + 1];

      int gx = -p00 + p02 + -2 * p10 + 2 * p12 + -p20 + p22;
      int gy = -p00 - 2 * p01 - p02 + p20 + 2 * p21 + p22;

      float magnitude = sqrt(float(gx * gx + gy * gy));

      gx_ptr[cur_offset] = gx;
      gy_ptr[cur_offset] = gy;
      mag_ptr[cur_offset] = magnitude;
    }
  }
}

void Filter::Prewitt(GrayImage& image, IntImage& gx, IntImage& gy,
                      FloatImage& magnitude) {
  int image_width = image.width();
  int image_height = image.height();

  gx.Reset(image_width, image_height);
  gy.Reset(image_width, image_height);
  magnitude.Reset(image_width, image_height);

  int y_end = image_height - 1;
  int x_end = image_width - 1;

  auto gx_ptr = gx.buffer();
  auto gy_ptr = gy.buffer();
  auto mag_ptr = magnitude.buffer();
  auto image_ptr = image.buffer();

  for (int y = 1; y < y_end; y++) {
    int y_offset = image_width * y;
    for (int x = 1; x < x_end; x++) {
      int cur_offset = y_offset + x;

      auto current_image_ptr = image_ptr + cur_offset;

      int p00 = current_image_ptr[-image_width - 1];
      int p01 = current_image_ptr[-image_width];
      int p02 = current_image_ptr[-image_width + 1];
      int p10 = current_image_ptr[-1];
      int p12 = current_image_ptr[1];
      int p20 = current_image_ptr[image_width - 1];
      int p21 = current_image_ptr[image_width];
      int p22 = current_image_ptr[image_width + 1];

      int gx = -p00 + p02 + -p10 + p12 + -p20 + p22;
      int gy = -p00 - p01 - p02 + p20 + p21 + p22;

      float magnitude = sqrt(float(gx * gx + gy * gy));

      gx_ptr[cur_offset] = gx;
      gy_ptr[cur_offset] = gy;
      mag_ptr[cur_offset] = magnitude;
    }
  }
}

std::vector<float> Filter::CreateGaussianFilterBuffer(std::size_t size,
                                                       float sigma) {
  std::vector<float> buffer(size * size);

  int center = int(roundf(float(size - 1) / 2.0f));

  auto buffer_ptr = buffer.data();

  float sigma_denominator = 2.0f * sigma * sigma;
  float filter_sum = 0.0f;

  for (auto y = 0; y < size; ++y) {
    auto y_ptr = buffer_ptr + (size * y);

    for (auto x = 0; x < size; ++x) {
      float x_term = float((center - x) * (center - x));
      float y_term = float((center - y) * (center - y));

      y_ptr[x] = exp(-1.0f * float(x_term + y_term) / sigma_denominator);
      filter_sum += y_ptr[x];
    }
  }

  for (auto& f : buffer) {
    f /= filter_sum;
  }

  return buffer;
}
