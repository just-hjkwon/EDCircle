#include "edge_drawing.h"

#include "image/filters.h"

EdgeDrawing::EdgeDrawing(float magnitude_threshold, float anchor_threshold,
                         int anchor_extraction_interval)
    : magnitude_threshold_(magnitude_threshold),
      anchor_threshold_(anchor_threshold),
      anchor_extraction_interval_(anchor_extraction_interval) {

}

void EdgeDrawing::DetectEdge(GrayImage& image) {
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

  x_gradient_ =
      std::make_shared<FloatImage>(width_, height_, x_gradient.buffer());
  y_gradient_ =
      std::make_shared<FloatImage>(width_, height_, y_gradient.buffer());

  std::size_t width = image.width();
  std::size_t height = image.height();

  std::vector<float> magnitude(width * height);
  std::vector<unsigned char> direction(width * height);

  for (auto y = 0; y < height; ++y) {
    std::size_t offset = get_offset(Position(0, y));

    auto x_ptr = x_gradient.buffer() + offset;
    auto y_ptr = y_gradient.buffer() + offset;

    auto magnitude_ptr = magnitude.data() + offset;
    auto direction_ptr = direction.data() + offset;

    for (auto x = 0; x < width; ++x) {
      float magnitude = sqrt((x_ptr[x] * x_ptr[x]) + (y_ptr[x] * y_ptr[x]));
      if (magnitude > magnitude_threshold_) {
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
  anchors_.reserve(width_ * height_);

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

      if (magnitude - neighbor0 >= anchor_threshold_ &&
          magnitude - neighbor1 >= anchor_threshold_) {
        anchors_.push_back(Position(x, y));
      }
    }
  }

  anchors_.shrink_to_fit();
}

void EdgeDrawing::ConnectingAnchors() {
  std::vector<unsigned char> edge_map_buffer(width_ * height_);

  edge_map_ = std::make_shared<Image<unsigned char>>(width_, height_,
                                                     edge_map_buffer.data());

  auto direction_map_ptr = direction_map_->buffer();

  edge_segments_.clear();
  edge_segments_.reserve(anchors_.size());

  for (auto& anchor : anchors_) {
    EdgeSegment edge_segment;

    if (is_edge(anchor) == true) {
      continue;
    }

    set_edge(anchor, true);
    edge_segment.push_back(std::make_pair(anchor, magnitudeAt(anchor)));
    bool push_back = true;

    EdgeDirection direction = directionAt(anchor);

    ConnectingAim aims[2];

    if (direction == EdgeDirection::HorizontalEdge) {
      aims[0] = ConnectingAim::Left;
      aims[1] = ConnectingAim::Right;
    } else {
      aims[0] = ConnectingAim::Up;
      aims[1] = ConnectingAim::Down;
    }

    for (const auto aim : aims) {
      if (aim == aims[0]) {
        push_back = true;
      } else {
        push_back = false;
      }

      Position current_position(anchor.x, anchor.y);
      ConnectingAim currenct_aim = aim;
      EdgeDirection current_direction = direction;

      Position next_position =
          FindNextConnectingPosition(current_position, currenct_aim);

      while (true) {
        if (isValidPosition(next_position) == false) {
          break;
        }

        float magnitude = magnitudeAt(next_position);
        bool edge = is_edge(next_position);

        if (magnitude == 0.0f || edge == true) {
          break;
        }

        set_edge(next_position, true);

        if (push_back == true) {
          edge_segment.push_back(std::make_pair(next_position, magnitude));
        } else {
          edge_segment.push_front(std::make_pair(next_position, magnitude));
        }

        EdgeDirection next_direction = directionAt(next_position);

        if (current_direction != next_direction) {
          if (next_direction == EdgeDirection::VerticalEdge) {
            if (next_position.y > current_position.y) {
              currenct_aim = ConnectingAim::Down;
            } else {
              currenct_aim = ConnectingAim::Up;
            }
          } else if (next_direction == EdgeDirection::HorizontalEdge) {
            if (next_position.x > current_position.x) {
              currenct_aim = ConnectingAim::Right;
            } else {
              currenct_aim = ConnectingAim::Left;
            }
          } else {
            throw std::runtime_error("Something wrong..");
          }
        }

        current_position = next_position;
        current_direction = next_direction;
        next_position =
            FindNextConnectingPosition(current_position, currenct_aim);
      }
    }

    edge_segments_.push_back(edge_segment);
  }

  edge_segments_.shrink_to_fit();
}

Position EdgeDrawing::FindNextConnectingPosition(Position pos,
                                                 ConnectingAim direction) {
  float neighbor_magnitudes[3] = {0.0f, 0.0f, 0.0f};
  Position neighbor_positions[3] = {pos, pos, pos};

  switch (direction) {
    case ConnectingAim::Left:
      neighbor_positions[0].x -= 1;
      neighbor_positions[0].y -= 1;
      neighbor_positions[1].x -= 1;
      neighbor_positions[2].x -= 1;
      neighbor_positions[2].y += 1;
      break;
    case ConnectingAim::Right:
      neighbor_positions[0].x += 1;
      neighbor_positions[0].y -= 1;
      neighbor_positions[1].x += 1;
      neighbor_positions[2].x += 1;
      neighbor_positions[2].y += 1;
      break;
    case ConnectingAim::Up:
      neighbor_positions[0].x -= 1;
      neighbor_positions[0].y -= 1;
      neighbor_positions[1].y -= 1;
      neighbor_positions[2].x += 1;
      neighbor_positions[2].y -= 1;
      break;
    case ConnectingAim::Down:
      neighbor_positions[0].x -= 1;
      neighbor_positions[0].y += 1;
      neighbor_positions[1].y += 1;
      neighbor_positions[2].x += 1;
      neighbor_positions[2].y += 1;
      break;
  }

  for (int i = 0; i < 3; ++i) {
    if (isValidPosition(neighbor_positions[i]) == true) {
      neighbor_magnitudes[i] = magnitudeAt(neighbor_positions[i]);
    }
  }

  if (neighbor_magnitudes[0] > neighbor_magnitudes[1] &&
      neighbor_magnitudes[0] > neighbor_magnitudes[2]) {
    return neighbor_positions[0];
  } else if (neighbor_magnitudes[2] > neighbor_magnitudes[0] &&
             neighbor_magnitudes[2] > neighbor_magnitudes[1]) {
    return neighbor_positions[2];
  } else {
    return neighbor_positions[1];
  }
}

float EdgeDrawing::magnitudeAt(Position pos) {
  std::size_t offset = get_offset(pos);
  return magnitude_->buffer()[offset];
}

EdgeDirection EdgeDrawing::directionAt(Position pos) {
  std::size_t offset = get_offset(pos);
  return (EdgeDirection)(direction_map_->buffer()[offset]);
}

void EdgeDrawing::set_direction(Position pos, EdgeDirection direction) {
  std::size_t offset = get_offset(pos);
  direction_map_->buffer()[offset] = (unsigned char)direction;
}

void EdgeDrawing::set_edge(Position pos, bool value) {
  std::size_t offset = get_offset(pos);
  if (value == true) {
    edge_map_->buffer()[offset] = 1;
  } else {
    edge_map_->buffer()[offset] = 0;
  }
}

inline bool EdgeDrawing::is_edge(Position pos) {
  std::size_t offset = get_offset(pos);
  if (edge_map_->buffer()[offset] == 1) {
    return true;
  } else {
    return false;
  }
}

std::size_t EdgeDrawing::get_offset(Position pos) {
  return width_ * pos.y + pos.x;
}

bool EdgeDrawing::isValidPosition(Position pos) {
  if (pos.x < 0 || pos.x >= width_ || pos.y < 0 || pos.y >= height_) {
    return false;
  } else {
    return true;
  }
}
