// Copyright Aeva 2024

#include "carla/sensor/data/FMCWLidarMeasurement.h"
#include "carla/sensor/s11n/FMCWLidarSerializer.h"

namespace carla {
namespace sensor {
namespace s11n {

SharedPtr<SensorData> FMCWLidarSerializer::Deserialize(RawData &&data) {
  return SharedPtr<data::FMCWLidarMeasurement>(
      new data::FMCWLidarMeasurement{std::move(data)});
}

}  // namespace s11n
}  // namespace sensor
}  // namespace carla
