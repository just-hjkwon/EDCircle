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
  ConnectingAnchors();
}

void EdgeDrawing::PrepareEdgeMap(GrayImage& image) {
  Filter edge_x_filter = FilterFactory::SobelXFilter();
  Filter edge_y_filter = FilterFactory::SobelYFilter();

  FloatImage x_gradient = image.MakeFloatFilteredImage(edge_x_filter);
  FloatImage y_gradient = image.MakeFloatFilteredImage(edge_y_filter);

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
    auto y_ptr = y_gradient_->buffer() + (width * y);

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

      if (direction_map_ptr[x] ==
          (unsigned char)EdgeDirection::HorizontalEdge) {
        neighbor0 = magnitude_ptr[x - width_];
        neighbor1 = magnitude_ptr[x + width_];
      } else {
        neighbor0 = magnitude_ptr[x - 1];
        neighbor1 = magnitude_ptr[x + 1];
      }

      if (magnitude - neighbor0 > anchor_threshold_ &&
          magnitude - neighbor1 > anchor_threshold_) {
        anchors_.push_back(Position(x, y));
      }
    }
  }
}

void EdgeDrawing::ConnectingAnchors() {
  std::vector<unsigned char> edge_map_buffer(width_ * height_);

  edge_map_ = std::make_shared<Image<unsigned char>>(width_, height_,
                                                     edge_map_buffer.data());

  auto direction_map_ptr = direction_map_->buffer();
  auto magnitude_ptr = magnitude_->buffer();

  for (auto& anchor : anchors_) {
    if ((EdgeDirection)direction_map_ptr[anchor.y * width_ + anchor.x] ==
        EdgeDirection::HorizontalEdge) {
      DoSmartRouteToLeft(anchor);
      DoSmartRouteToRight(anchor);
    } else {
      DoSmartRouteToUp(anchor);
      DoSmartRouteToDown(anchor);
    }
  }
}

void EdgeDrawing::DoSmartRouteToLeft(Position start_position) {
  auto direction_map_ptr = direction_map_->buffer();
  auto magnitude_ptr = magnitude_->buffer();
  auto edge_map_ptr = edge_map_->buffer();

  Position current_position = start_position;

  int offset = width_ * current_position.y + current_position.x;

  float magnitude = magnitude_ptr[offset];
  unsigned char is_edge = edge_map_ptr[offset];
  EdgeDirection direction = (EdgeDirection)direction_map_ptr[offset];

  while (magnitude > 0.0f && is_edge == 0 &&
         direction == EdgeDirection::HorizontalEdge) {
    edge_map_ptr[offset] = 1;

    float neighbor0 = 0.0f;
    float neighbor1 = 0.0f;
    float neighbor2 = 0.0f;

    if (current_position.x >= 1 && current_position.y >= 1) {
      neighbor0 = magnitude_ptr[offset - width_ - 1];
    }
    if (current_position.x >= 1) {
      neighbor1 = magnitude_ptr[offset - 1];
    }
    if (current_position.x >= 1 && current_position.y <= int(height_ - 1)) {
      neighbor2 = magnitude_ptr[offset + width_ - 1];
    }

    if (neighbor0 > neighbor1 && neighbor0 > neighbor2) {
      current_position.x -= 1;
      current_position.y -= 1;
    } else if (neighbor2 > neighbor0 && neighbor2 > neighbor1) {
      current_position.x -= 1;
      current_position.y += 1;
    } else {
      current_position.x -= 1;
    }

    offset = width_ * current_position.y + current_position.x;

    magnitude = magnitude_ptr[offset];
    is_edge = edge_map_ptr[offset];
    direction = (EdgeDirection)direction_map_ptr[offset];
  }

  if (direction == EdgeDirection::VerticalEdge) {
    DoSmartRouteToUp(current_position);
    DoSmartRouteToDown(current_position);
  }
}

void EdgeDrawing::DoSmartRouteToRight(Position start_position) {
  auto direction_map_ptr = direction_map_->buffer();
  auto magnitude_ptr = magnitude_->buffer();
  auto edge_map_ptr = edge_map_->buffer();

  Position current_position = start_position;

  int offset = width_ * current_position.y + current_position.x;

  float magnitude = magnitude_ptr[offset];
  unsigned char is_edge = edge_map_ptr[offset];
  EdgeDirection direction = (EdgeDirection)direction_map_ptr[offset];

  while (magnitude > 0.0f && is_edge == 0 &&
         direction == EdgeDirection::HorizontalEdge) {
    edge_map_ptr[offset] = 1;

    float neighbor0 = 0.0f;
    float neighbor1 = 0.0f;
    float neighbor2 = 0.0f;

    if (current_position.x < int(width_ - 1) && current_position.y >= 1) {
      neighbor0 = magnitude_ptr[offset - width_ + 1];
    }
    if (current_position.x < int(width_ - 1)) {
      neighbor1 = magnitude_ptr[offset + 1];
    }
    if (current_position.x < int(width_ - 1) &&
        current_position.y <= int(height_ - 1)) {
      neighbor2 = magnitude_ptr[offset + width_ + 1];
    }

    if (neighbor0 > neighbor1 && neighbor0 > neighbor2) {
      current_position.x += 1;
      current_position.y -= 1;
    } else if (neighbor2 > neighbor0 && neighbor2 > neighbor1) {
      current_position.x += 1;
      current_position.y += 1;
    } else {
      current_position.x += 1;
    }

    offset = width_ * current_position.y + current_position.x;

    magnitude = magnitude_ptr[offset];
    is_edge = edge_map_ptr[offset];
    direction = (EdgeDirection)direction_map_ptr[offset];
  }

  if (direction == EdgeDirection::VerticalEdge) {
    DoSmartRouteToUp(current_position);
    DoSmartRouteToDown(current_position);
  }
}

void EdgeDrawing::DoSmartRouteToUp(Position start_position) {
  auto direction_map_ptr = direction_map_->buffer();
  auto magnitude_ptr = magnitude_->buffer();
  auto edge_map_ptr = edge_map_->buffer();

  Position current_position = start_position;

  int offset = width_ * current_position.y + current_position.x;

  float magnitude = magnitude_ptr[offset];
  unsigned char is_edge = edge_map_ptr[offset];
  EdgeDirection direction = (EdgeDirection)direction_map_ptr[offset];

  while (magnitude > 0.0f && is_edge == 0 &&
         direction == EdgeDirection::VerticalEdge) {
    edge_map_ptr[offset] = 1;

    float neighbor0 = 0.0f;
    float neighbor1 = 0.0f;
    float neighbor2 = 0.0f;

    if (current_position.x >= 1 && current_position.y >= 1) {
      neighbor0 = magnitude_ptr[offset - width_ - 1];
    }
    if (current_position.y >= 1) {
      neighbor1 = magnitude_ptr[offset - width_];
    }
    if (current_position.x < int(width_ - 1) && current_position.y >= 1) {
      neighbor2 = magnitude_ptr[offset - width_ + 1];
    }

    if (neighbor0 > neighbor1 && neighbor0 > neighbor2) {
      current_position.x -= 1;
      current_position.y -= 1;
    } else if (neighbor2 > neighbor0 && neighbor2 > neighbor1) {
      current_position.x += 1;
      current_position.y -= 1;
    } else {
      current_position.y -= 1;
    }

    if (current_position.x < 0 || current_position.x >= width_ ||
        current_position.y < 0 || current_position.y >= height_) {
      break;
    }

    offset = width_ * current_position.y + current_position.x;

    magnitude = magnitude_ptr[offset];
    is_edge = edge_map_ptr[offset];
    direction = (EdgeDirection)direction_map_ptr[offset];
  }

  if (direction == EdgeDirection::HorizontalEdge) {
    DoSmartRouteToLeft(current_position);
    DoSmartRouteToRight(current_position);
  }
}

void EdgeDrawing::DoSmartRouteToDown(Position start_position) {
  auto direction_map_ptr = direction_map_->buffer();
  auto magnitude_ptr = magnitude_->buffer();
  auto edge_map_ptr = edge_map_->buffer();

  Position current_position = start_position;

  int offset = width_ * current_position.y + current_position.x;

  float magnitude = magnitude_ptr[offset];
  unsigned char is_edge = edge_map_ptr[offset];
  EdgeDirection direction = (EdgeDirection)direction_map_ptr[offset];

  while (magnitude > 0.0f && is_edge == 0 &&
         direction == EdgeDirection::VerticalEdge) {
    edge_map_ptr[offset] = 1;

    float neighbor0 = 0.0f;
    float neighbor1 = 0.0f;
    float neighbor2 = 0.0f;

    if (current_position.x >= 1 && current_position.y < int(height_ - 1)) {
      neighbor0 = magnitude_ptr[offset + width_ - 1];
    }
    if (current_position.y < int(height_ - 1)) {
      neighbor1 = magnitude_ptr[offset + width_];
    }
    if (current_position.x < int(width_ - 1) &&
        current_position.y < int(height_ - 1)) {
      neighbor2 = magnitude_ptr[offset + width_ + 1];
    }

    if (neighbor0 > neighbor1 && neighbor0 > neighbor2) {
      current_position.x -= 1;
      current_position.y += 1;
    } else if (neighbor2 > neighbor0 && neighbor2 > neighbor1) {
      current_position.x += 1;
      current_position.y += 1;
    } else {
      current_position.y += 1;
    }

    offset = width_ * current_position.y + current_position.x;

    magnitude = magnitude_ptr[offset];
    is_edge = edge_map_ptr[offset];
    direction = (EdgeDirection)direction_map_ptr[offset];
  }

  if (direction == EdgeDirection::HorizontalEdge) {
    DoSmartRouteToLeft(current_position);
    DoSmartRouteToRight(current_position);
  }
}