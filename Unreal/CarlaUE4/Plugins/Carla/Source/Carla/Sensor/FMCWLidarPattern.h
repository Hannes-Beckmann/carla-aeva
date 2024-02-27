// Copyright Aeva 2024

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <yaml.h>

#include "Carla/Sensor/LidarDescription.h"
#include "carla/geom/Math.h"

template <typename T>
inline constexpr int FindClosestMultiple(const T Num, const int Base) {
  // Uses rint instead of round to match Python's Banker's rounding implementation.
  return static_cast<int>(Base * rint(static_cast<float>(Num) / Base));
}

template <typename T>
inline std::vector<T> Linspace(const double Start, const double End, const size_t Size) {
  // Adapted from https://gist.github.com/lorenzoriano/5414671
  std::vector<T> linspaced;
  linspaced.reserve(Size);

  if (Size == 0) {
    return linspaced;
  }
  if (Size == 1) {
    linspaced.emplace_back(static_cast<T>(Start));
    return linspaced;
  }

  const double Delta = (End - Start) / (Size - 1);
  for (size_t i = 0; i < (Size - 1); ++i) {
    linspaced.emplace_back(static_cast<T>(Start + Delta * i));
  }

  // Ensure that start and end are exactly the same as the input.
  linspaced.emplace_back(static_cast<T>(End));
  return linspaced;
}

// Real rotating lidars do not capture the entire frame instantaneously and instead accumulate points 
// as the laser(s) traverse the frame. This enum represents the type of raycast at every simulation timestep.
// Raycasting fewer points will model the rolling shutter effect to increasingly higher accuracy. 
enum class RaycastMode : uint8_t {
  // The entire scan pattern is captured each timestep. Timestep should equal frame period.
  RAYCAST_PER_FRAME,
  // One line is captured each timestep. Timestep should equal frame period / number of lines.
  RAYCAST_PER_LINE,
  // One point is captured each timestep. Timestep should equal frame period / number of points.*
  // Since the number of points captured each timestep is a multiple of the number of threads,
  // timestep should instead equal frame period / (number of points / number of threads).
  RAYCAST_PER_POINT
};

// Class representing a set of vectors arranged in beams (lasers) and lines to be raycast by the simulator.
class FMCWLidarPattern {
public:
  FMCWLidarPattern() = default;
  FMCWLidarPattern(const FLidarDescription &LidarDescription) {
    // Load scan pattern YAML
    UE_LOG(LogCarla, Log, TEXT("Using scan pattern YAML: %s"), *LidarDescription.PatternFilePath);
    const auto YamlPath = std::string(TCHAR_TO_UTF8(*LidarDescription.PatternFilePath));
    verifyf(!YamlPath.empty(), TEXT("Missing scan pattern YAML"));

    const auto YamlNode = YAML::LoadFile(YamlPath);
    verifyf([&YamlNode](){
      return !YamlNode.IsNull() && YamlNode["num_beams"] && YamlNode["beam_elevation_offsets_deg"] && YamlNode["patterns"];
    }(), TEXT("Could not load pattern file"));

    const auto PatternNode =
        YamlNode["patterns"]
            [std::string(TCHAR_TO_UTF8(*LidarDescription.PatternName))];
    verifyf([&PatternNode](){
      return PatternNode.IsMap() && PatternNode["horizontal_fov_deg"] && PatternNode["points_per_line"] &&
      PatternNode["elevation_steps_deg"];
    }(), TEXT("Could not load specified scan pattern"));

    const auto ElevationSteps = PatternNode["elevation_steps_deg"].as<std::vector<double>>();
    const auto MinHorizontalFovDeg = PatternNode["horizontal_fov_deg"].as<std::vector<double>>()[0];
    const auto MaxHorizontalFovDeg = PatternNode["horizontal_fov_deg"].as<std::vector<double>>()[1];

    NumBeams = YamlNode["num_beams"].as<std::size_t>();
    NumRaycastThreads = LidarDescription.Channels;
    // Adjust points per line so it is divisible by the number of threads.
    PointsPerLine = FindClosestMultiple(PatternNode["points_per_line"].as<std::size_t>(), NumRaycastThreads);

    const auto LinesPerBeam = ElevationSteps.size();
    PointsPerBeam = PointsPerLine * LinesPerBeam;

    Mode = static_cast<RaycastMode>(LidarDescription.RaycastMode);

    const auto AzimuthSteps = Linspace<double>(MinHorizontalFovDeg, MaxHorizontalFovDeg, PointsPerLine);
    const auto BeamOffsets =
        YamlNode["beam_elevation_offsets_deg"].as<std::vector<double>>();
    verifyf(NumBeams == BeamOffsets.size(), TEXT("Beam offsets do not match number of beams"));

    // Populate the individual azimuth and elevation pairs.
    AzimuthsDeg.assign(NumBeams, std::vector<float>());
    ElevationsDeg.assign(NumBeams, std::vector<float>());
    for (size_t beam_idx = 0; beam_idx < NumBeams; ++beam_idx) {
      AzimuthsDeg[beam_idx].reserve(PointsPerBeam);
      ElevationsDeg[beam_idx].reserve(PointsPerBeam);
      for (const auto &elevation : ElevationSteps) {
        for (const auto &azimuth : AzimuthSteps) {
          AzimuthsDeg[beam_idx].emplace_back(azimuth);
          ElevationsDeg[beam_idx].emplace_back(elevation + BeamOffsets[beam_idx] + LidarDescription.ElevationOffset);
        }
      }
    }

    verifyf(PointsPerBeam == AzimuthsDeg[0].size(), TEXT("Points per beam mismatch"));

    UE_LOG(LogCarla, Log, TEXT("Scan pattern set to %s"), *LidarDescription.PatternName);
  }

  // Total number of pre-dropoff points that each thread should attempt to draw.
  inline std::size_t PointsPerThread() const {
    return PointsPerBeamPerThread() * NumBeams;
  }

  // Total number of pre-dropoff points per beam each thread should attempt to draw.
  inline std::size_t PointsPerBeamPerThread() const {
    if (Mode == RaycastMode::RAYCAST_PER_FRAME) {
      return PointsPerBeam / NumRaycastThreads;
    } else if (Mode == RaycastMode::RAYCAST_PER_LINE) {
      return PointsPerLine / NumRaycastThreads;
    } else if (Mode == RaycastMode::RAYCAST_PER_POINT) {
      return 1;
    }
    return 0;
  }

  // Advance the index of the point in the laser by one simulation timestep according to the raycast mode.
  inline void TickRaycastIdxOffset() const {
    if (Mode == RaycastMode::RAYCAST_PER_FRAME) {
      RaycastIndexOffset = 0;
    } else if (Mode == RaycastMode::RAYCAST_PER_LINE) {
      RaycastIndexOffset =
          (RaycastIndexOffset + PointsPerLine) % PointsPerBeam;
    } else if (Mode == RaycastMode::RAYCAST_PER_POINT) {
      RaycastIndexOffset =
          (RaycastIndexOffset + NumRaycastThreads) % PointsPerBeam;
    }
  }
  
  // Set of azimuth elevation pairs to be raycast per laser.
  std::vector<std::vector<float>> AzimuthsDeg;
  std::vector<std::vector<float>> ElevationsDeg;

  // Number of lasers.
  std::size_t NumBeams = 4;

  // Number of simultanous raycasts per laser. 
  std::size_t NumRaycastThreads = 32;

  std::size_t PointsPerBeam = 0;
  std::size_t PointsPerLine = 0;

  RaycastMode Mode = RaycastMode::RAYCAST_PER_FRAME;

  // Current index of the point in the beam to be raycast.
  mutable std::size_t RaycastIndexOffset = 0;
};
