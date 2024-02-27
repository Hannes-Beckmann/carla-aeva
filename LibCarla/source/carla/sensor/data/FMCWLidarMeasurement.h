// Copyright Aeva 2024

#pragma once

#include "carla/Debug.h"
#include "carla/rpc/Location.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/FMCWLidarSerializer.h"

namespace carla {
namespace sensor {
namespace data {

/// Measurement produced by a FMCW LiDAR. Consists of an array of 3D points plus
/// some extra meta-information about the LiDAR (See FMCWLidarData.h to see what fields
/// are contained in the header)
class FMCWLidarMeasurement : public Array<data::FMCWLidarDetection> {
  static_assert(sizeof(data::FMCWLidarDetection) ==
                    6u * sizeof(float) + 3u * sizeof(uint32_t) + 3u * sizeof(uint8_t),
                "FMCWLidarDetection size mismatch");
  using Super = Array<data::FMCWLidarDetection>;

 protected:
  using Serializer = s11n::FMCWLidarSerializer;

  friend Serializer;

  explicit FMCWLidarMeasurement(RawData &&data)
      : Super(std::move(data), [](const RawData &d) { return Serializer::GetHeaderOffset(d); }) {}

 private:
  auto GetHeader() const { return Serializer::DeserializeHeader(Super::GetRawData()); }

 public:
  /// Horizontal angle of the LiDAR at the time of the measurement.
  auto GetHorizontalAngle() const { return GetHeader().GetHorizontalAngle(); }

  /// Number of channels of the LiDAR.
  auto GetChannelCount() const { return GetHeader().GetChannelCount(); }
  /// Number of beams in the LiDAR.
  auto GetBeamCount() const { return GetHeader().GetBeamCount(); }

  /// Find the number of points raycast by each thread, regardless of dropoff.
  auto GetPointsPerChannel() const {
    return GetHeader().GetPointsPerChannel();
  }
  /// Find the number of points raycast by each beam, regardless of dropoff.
  auto GetPointsPerBeam() const { return GetHeader().GetPointsPerBeam(); }
};

}  // namespace data
}  // namespace sensor
}  // namespace carla
