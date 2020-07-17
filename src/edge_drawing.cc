#include "edge_drawing.h"

#include "image/filter.h"

EdgeDrawing::EdgeDrawing(float magnitude_threshold, float anchor_threshold,
                         int anchor_extraction_interval)
    : magnitude_threshold_(magnitude_threshold),
      anchor_threshold_(anchor_threshold),
      anchor_extraction_interval_(anchor_extraction_interval),
      gx_(0, 0),
      gy_(0, 0),
      magnitude_(0, 0),
      direction_map_(0, 0),
      edge_map_(0, 0) {}

void EdgeDrawing::DetectEdge(GrayImage& image) {
  width_ = image.width();
  height_ = image.height();

  PrepareEdgeMap(image);
  ExtractAnchor();
  ConnectingAnchors();
}

std::list<EdgeSegment> EdgeDrawing::edge_segments() {
  return edge_segments_;
}

void EdgeDrawing::PrepareEdgeMap(GrayImage& image) {
  Filter::Sobel(image, gx_, gy_, magnitude_);

  int image_width = image.width();
  int image_height = image.height();

  direction_map_.Reset(image_width, image_height);

  for (auto y = 1; y < image_height; ++y) {
    std::size_t offset = get_offset(Position(0, y));

    auto x_ptr = gx_.buffer() + offset;
    auto y_ptr = gy_.buffer() + offset;

    auto direction_ptr = direction_map_.buffer() + offset;

    for (auto x = 1; x < image_width; ++x) {
      if (abs(x_ptr[x]) >= abs(y_ptr[x])) {
        direction_ptr[x] = (unsigned char)EdgeDirection::VerticalEdge;
      } else {
        direction_ptr[x] = (unsigned char)EdgeDirection::HorizontalEdge;
      }
    }
  }
}

void EdgeDrawing::ExtractAnchor() {
  anchors_.clear();

  int x_start = std::max(1, anchor_extraction_interval_ / 2);
  int y_start = std::max(1, anchor_extraction_interval_ / 2);

  for (auto y = y_start; y < height_ - 1; y += anchor_extraction_interval_) {
    auto direction_map_ptr = direction_map_.buffer() + (y * width_);
    auto magnitude_ptr = magnitude_.buffer() + (y * width_);

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
        Edgel anchor{Position(x, y), magnitude};
        anchors_.push_back(anchor);
      }
    }
  }
}

void EdgeDrawing::ConnectingAnchors() {
  edge_map_.Reset(width_, height_);

  auto direction_map_ptr = direction_map_.buffer();

  edge_segments_.clear();

  for (auto& anchor : anchors_) {
    EdgeSegment edge_segment;

    if (is_edge(anchor.position) == true) {
      continue;
    }

    set_edge(anchor.position, true);
    edge_segment.push_back(anchor);
    bool push_back = true;

    EdgeDirection direction = directionAt(anchor.position);

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

      Position current_position = anchor.position;
      ConnectingAim currenct_aim = aim;
      EdgeDirection current_direction = direction;

      Position next_position =
          FindNextConnectingPosition(current_position, currenct_aim);

      while (true) {
        if (isValidPosition(next_position) == false) {
          break;
        }

        float next_magnitude = magnitudeAt(next_position);
        bool edge = is_edge(next_position);

        if (next_magnitude == 0.0f || edge == true) {
          break;
        }

        set_edge(next_position, true);

        Edgel next_edge{next_position, next_magnitude};
        if (push_back == true) {
          edge_segment.push_back(next_edge);
        } else {
          edge_segment.push_front(next_edge);
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
}

Position EdgeDrawing::FindNextConnectingPosition(Position position,
                                                 ConnectingAim direction) {
  float neighbor_magnitudes[3] = {0.0f, 0.0f, 0.0f};
  Position neighbor_positions[3] = {position, position, position};

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

float EdgeDrawing::magnitudeAt(Position position) {
  std::size_t offset = get_offset(position);
  return magnitude_.buffer()[offset];
}

EdgeDirection EdgeDrawing::directionAt(Position position) {
  std::size_t offset = get_offset(position);
  return (EdgeDirection)(direction_map_.buffer()[offset]);
}

void EdgeDrawing::set_direction(Position position, EdgeDirection direction) {
  std::size_t offset = get_offset(position);
  direction_map_.buffer()[offset] = (unsigned char)direction;
}

void EdgeDrawing::set_edge(Position position, bool value) {
  std::size_t offset = get_offset(position);
  if (value == true) {
    edge_map_.buffer()[offset] = 1;
  } else {
    edge_map_.buffer()[offset] = 0;
  }
}

inline bool EdgeDrawing::is_edge(Position position) {
  std::size_t offset = get_offset(position);
  if (edge_map_.buffer()[offset] == 1) {
    return true;
  } else {
    return false;
  }
}

std::size_t EdgeDrawing::get_offset(Position position) {
  return width_ * position.y + position.x;
}

bool EdgeDrawing::isValidPosition(Position position) {
  if (position.x < 0 || position.x >= width_ || position.y < 0 ||
      position.y >= height_) {
    return false;
  } else {
    return true;
  }
}
