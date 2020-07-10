#include "edge_drawing.h"

#include "image/filters.h"

EdgeDrawing::EdgeDrawing(GrayImage& image, float magnitude_threshold,
                         float anchor_threshold, int anchor_extraction_interval)
    : magnitude_threshold_(magnitude_threshold),
      anchor_threshold_(anchor_threshold),
      anchor_extraction_interval_(anchor_extraction_interval) {
  width_ = image.width();
  height_ = image.height();

  PrepareEdgeMap(image);
  ExtractAnchor();
}

void EdgeDrawing::PrepareEdgeMap(GrayImage& image) {
  Filter prewitt_x = FilterFactory::PrewittXFilter();
  Filter prewitt_y = FilterFactory::PrewittYFilter();

  FloatImage x_gradient = image.MakeFloatFilteredImage(prewitt_x);
  FloatImage y_gradient = image.MakeFloatFilteredImage(prewitt_y);

  std::size_t width = image.width();
  std::size_t height = image.height();

  x_gradient_ =
      std::make_shared<FloatImage>(width, height, x_gradient.buffer());
  y_gradient_ =
      std::make_shared<FloatImage>(width, height, y_gradient.buffer());

  std::vector<float> magnitude(width * height);
  std::vector<unsigned char> direction(width * height);

  for (auto y = 0; y < height; ++y) {
    auto x_ptr = x_gradient_->buffer() + (width * y);
    auto y_ptr = x_gradient_->buffer() + (width * y);

    auto magnitude_ptr = magnitude.data() + (width * y);
    auto direction_ptr = direction.data() + (width * y);

    for (auto x = 0; x < width; ++x) {
      float magnitude = sqrt((x_ptr[x] * x_ptr[x]) + (y_ptr[x] * y_ptr[x]));
      if (magnitude >= magnitude_threshold_) {
        magnitude_ptr[x] = magnitude;
      } else {
        magnitude_ptr[x] = 0.0f;
      }

      if (abs(x_ptr[x]) >= abs(y_ptr[x])) {
        direction_ptr[x] = (unsigned char)EdgeDirection::VerticalEdge;
      } else {
        direction_ptr[x] = (unsigned char)EdgeDirection::HorizontalEdge;
      }
    }
  }

  magnitude_ = std::make_shared<FloatImage>(width, height, magnitude.data());
  direction_map_ =
      std::make_shared<Image<unsigned char>>(width, height, direction.data());
}

void EdgeDrawing::ExtractAnchor() {
  anchors_.clear();

  int x_start = std::max(1, anchor_extraction_interval_ / 2);
  int y_start = std::max(1, anchor_extraction_interval_ / 2);

  for (auto y = y_start; y < height_ - 1; y += anchor_extraction_interval_) {
    auto direction_map_ptr = direction_map_->buffer() + (y * width_);
    auto magnitude_ptr = magnitude_->buffer() + (y * width_);

    for (auto x = x_start; x < width_ - 1; x += anchor_extraction_interval_) {
      float magnitude = magnitude_ptr[x];
      float neighbor0 = 0.0f;
      float neighbor1 = 0.0f;

      if (direction_map_ptr[x] == (unsigned char)EdgeDirection::HorizontalEdge) {
        neighbor0 = magnitude_ptr[x - 1];
        neighbor1 = magnitude_ptr[x + 1];
      } else {
        neighbor0 = magnitude_ptr[x - width_];
        neighbor1 = magnitude_ptr[x + width_];
      }

      if (magnitude - neighbor0 > anchor_threshold_ &&
          magnitude - neighbor1 > anchor_threshold_) {
        anchors_.push_back(Position(x, y));
      }
    }
  }
}

