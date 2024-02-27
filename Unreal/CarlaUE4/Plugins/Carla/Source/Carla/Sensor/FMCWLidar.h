// Copyright Aeva 2024

#pragma once

#include <utility>

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/FMCWLidarPattern.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/FMCWLidarData.h>
#include <compiler/enable-ue4-macros.h>

#include "FMCWLidar.generated.h"

/// A ray-cast based FMCW LiDAR sensor.
UCLASS()
class CARLA_API AFMCWLidar : public ASensor {
  GENERATED_BODY()

 protected:
  using FFMCWLidarData = carla::sensor::data::FMCWLidarData;
  using FFMCWDetection = carla::sensor::data::FMCWLidarDetection;

 public:
  static FActorDefinition GetSensorDefinition();

  AFMCWLidar(const FObjectInitializer& ObjectInitializer);

  virtual void Set(const FActorDescription& ActorDescription) override;
  virtual void Set(const FLidarDescription& LidarDescription);

 private:
  void BeginPlay() override;
  virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime) override;

  /// Updates FMCWLidarMeasurement with the points read in delta_time.
  void SimulateLidarScanPattern(const float DeltaTime);

  /// Shoot a laser ray-trace, return whether the laser hit something.
  bool ShootLaser(const float AzimuthAngle, const float ElevationAngle, FHitResult& HitResult,
                  FCollisionQueryParams& TraceParams) const;

  /// Method that allows to pre-process the ray before shooting it.
  virtual bool PreprocessRay() const;
  bool PostprocessDetection(FFMCWDetection& Detection) const;

  /// Compute all raw detection information.
  void ComputeRawDetection(const FHitResult& HitInfo, const FVector& HitVelocity,
                           const FTransform& SensorTransform,
                           const std::pair<uint32_t, uint8_t>& IndexInfo,
                           const std::pair<float, float>& AzElInfo,
                           FFMCWDetection& Detection) const;

  /// Compute velocities for static and dynamic objects in the scene.
  void ComputeCurrentSensorVelocity(const float DeltaTime);
  float ComputeDopplerVelocity(const FVector& TargetPosition,
                               const FVector& TargetVelocity) const;

  /// Saving the hits the ray-cast returns per channel.
  void WritePointAsync(uint32_t channel, uint32_t IndexInBeam, uint8_t BeamIndex,
                       const FHitResult& HitInfo, float AzimuthAngle, float ElevationAngle);

  /// Clear the recorded data structure.
  void ResetRecordedData(uint32_t Channels, uint32_t PointsPerChannel);

  /// This method uses all the saved FHitResults, compute the
  /// RawDetections and then send it to the LidarData structure.
  virtual void ComputeAndSaveDetections(const FTransform& SensorTransform);

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;
  FFMCWLidarData FMCWLidarData;

  // Active scan pattern information
  FMCWLidarPattern LidarPattern;

  /// Use nested vectors to track multiple channels. Each inner vector corresponds to one channel.
  std::vector<std::vector<FHitResult>> RecordedHits;
  std::vector<std::vector<FVector>> RecordedVelocities;
  /// List of indices of recorded points + the beams that they belong to, per channel.
  /// Number of points written to each channel (after preprocessing/intersection tests).
  std::vector<std::vector<std::pair<uint32_t, uint8_t>>> RecordedIndices;
  std::vector<std::vector<std::pair<float, float>>> RecordedAzEl;
  std::vector<uint32_t> RecordedPointsPerChannel;

  // State variables used to calculate doppler velocities and apply motion correction.
  FVector CurrentSensorPosition;
  float CurrentSensorTime = 0;
  FVector CurrentSensorVelocity;
  FVector PrevSensorPosition;
  float PrevSensorTime = 0;

  /// Enable/Disable general dropoff of lidar points
  bool DropOffGenActive = false;
  bool NoiseActive = false;

  /// Slope for the intensity dropoff of lidar points, it is calculated
  /// throught the dropoff limit and the dropoff at zero intensity
  /// The points is kept with a probality alpha*Intensity + beta where
  /// alpha = (1 - dropoff_zero_intensity) / droppoff_limit
  /// beta = (1 - dropoff_zero_intensity)
  float DropOffAlpha = 0.0f;
  float DropOffBeta = 0.0f;
};
