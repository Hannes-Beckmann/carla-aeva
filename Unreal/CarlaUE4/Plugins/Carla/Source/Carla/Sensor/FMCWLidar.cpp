// Copyright Aeva 2024

#include <PxScene.h>
#include <cmath>
#include <sstream>

#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/FMCWLidar.h"
#include "Carla/Sensor/FMCWLidarPattern.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

namespace crp = carla::rpc;

FActorDefinition AFMCWLidar::GetSensorDefinition() {
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("fmcw"));
}

AFMCWLidar::AFMCWLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {
  PrimaryActorTick.bCanEverTick = true;

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
}

void AFMCWLidar::Set(const FActorDescription& ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void AFMCWLidar::Set(const FLidarDescription& LidarDescription)
{
  Description = LidarDescription;
  LidarPattern = FMCWLidarPattern(Description);
  FMCWLidarData = FFMCWLidarData(
    Description.Channels, LidarPattern.NumBeams, LidarPattern.PointsPerThread(), LidarPattern.PointsPerBeam);
  RecordedPointsPerChannel.resize(Description.Channels, 0);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
  NoiseActive = Description.NoiseStdDev > std::numeric_limits<float>::epsilon();
}

void AFMCWLidar::BeginPlay()
{
  Super::BeginPlay();
  PrevSensorPosition = GetActorLocation();
}

void AFMCWLidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AFMCWLidar::PostPhysTick);

  ComputeCurrentSensorVelocity(DeltaTime);
  SimulateLidarScanPattern(DeltaTime);
  
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.SerializeAndSend(*this, FMCWLidarData, DataStream.PopBufferFromPool());
  }
}

void AFMCWLidar::ResetRecordedData(uint32_t Channels, uint32_t PointsPerChannel)
{
  RecordedHits.resize(Channels);
  RecordedVelocities.resize(Channels);
  RecordedIndices.resize(Channels);
  RecordedAzEl.resize(Channels);
  for (size_t i = 0; i < Channels; ++i) {
    RecordedHits[i].clear();
    RecordedVelocities[i].clear();
    RecordedIndices[i].clear();
    RecordedAzEl[i].clear();
    RecordedHits[i].reserve(PointsPerChannel);
    RecordedVelocities[i].reserve(PointsPerChannel);
    RecordedIndices[i].reserve(PointsPerChannel);
    RecordedAzEl[i].reserve(PointsPerChannel);
  }
}

void AFMCWLidar::ComputeCurrentSensorVelocity(const float DeltaTime)
{
  CurrentSensorPosition = GetActorLocation();
  CurrentSensorTime = GetWorld()->TimeSeconds;
  CurrentSensorVelocity = (CurrentSensorPosition - PrevSensorPosition) / (CurrentSensorTime - PrevSensorTime);
  PrevSensorPosition = CurrentSensorPosition;
  PrevSensorTime = CurrentSensorTime;
}

float AFMCWLidar::ComputeDopplerVelocity(const FVector& TargetPosition,
                                            const FVector& TargetVelocity) const
{
  const FVector Direction = (TargetPosition - CurrentSensorPosition).GetSafeNormal();
  const FVector RelativeVelocity = (TargetVelocity - CurrentSensorVelocity);
  // Calculate the measured velocity relative to the world or the sensor.
  const float DopplerVelocity = Description.MotionCompensate
                                 ? FVector::DotProduct(TargetVelocity, Direction)
                                 : FVector::DotProduct(RelativeVelocity, Direction);
  return DopplerVelocity * 1e-2;  // Convert cm/s to m/s
}

bool AFMCWLidar::PreprocessRay() const
{
  if (DropOffGenActive){
    return RandomEngine->GetUniformFloat() > Description.DropOffGenRate;
  }
  return true;
}

void AFMCWLidar::SimulateLidarScanPattern(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AFMCWLidar::SimulateLidarScanPattern);

  ResetRecordedData(LidarPattern.NumRaycastThreads, LidarPattern.PointsPerThread());
  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();

  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(LidarPattern.NumRaycastThreads, [&](int32 thread_idx) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

      FCollisionQueryParams TraceParams =
          FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnPhysicalMaterial = false;

      for (size_t beam_idx = 0; beam_idx < LidarPattern.NumBeams; ++beam_idx) {
        size_t thread_start_idx  = LidarPattern.RaycastIndexOffset +  thread_idx      * LidarPattern.PointsPerBeamPerThread();
        size_t thread_end_idx    = LidarPattern.RaycastIndexOffset + (thread_idx + 1) * LidarPattern.PointsPerBeamPerThread();
        for (size_t point_idx = thread_start_idx; point_idx < thread_end_idx; ++point_idx) {
          // Ray-cast and store the hit result.
          FHitResult HitResult;
          const auto &Azimuth = LidarPattern.AzimuthsDeg[beam_idx][point_idx];
          const auto &Elevation = LidarPattern.ElevationsDeg[beam_idx][point_idx];
          if (PreprocessRay()) {
            ShootLaser(Azimuth, Elevation, HitResult, TraceParams);
          }
          // Record the point even if it is pre-dropped or is not blocking. 
          WritePointAsync(thread_idx, point_idx, beam_idx, HitResult, Azimuth, Elevation);
        }
      }
    });
  }

  LidarPattern.TickRaycastIdxOffset();

  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransform = GetTransform();
  ComputeAndSaveDetections(ActorTransform);
}

bool AFMCWLidar::ShootLaser(const float AzimuthAngle, const float ElevationAngle,
                               FHitResult& HitResult, FCollisionQueryParams& TraceParams) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  const FTransform ActorTransform = GetTransform();
  const FVector LidarBodyLoc = ActorTransform.GetLocation();
  const FRotator LidarBodyRot = ActorTransform.Rotator();

  const FRotator LaserRot(ElevationAngle, AzimuthAngle, 0);  // float InPitch, float InYaw, float InRoll
  const FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot, LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  FHitResult HitInfo(ForceInit);
  GetWorld()->ParallelLineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  HitResult = HitInfo;
  return true;
}

void AFMCWLidar::WritePointAsync(uint32_t channel, uint32_t IndexInBeam, uint8_t BeamIndex,
                                    const FHitResult& HitInfo, float AzimuthAngle, float ElevationAngle)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  DEBUG_ASSERT(GetChannelCount() > channel);
  RecordedHits[channel].emplace_back(HitInfo);

  // Store the instantaneous actor velocity.
  FVector velocity = FVector(0.0f, 0.0f, 0.0f);
  if (HitInfo.IsValidBlockingHit()){
    const AActor* Actor = HitInfo.GetActor();
    if (Actor != nullptr) {
      velocity = Actor->GetVelocity();
    }
  }
  RecordedVelocities[channel].emplace_back(velocity);
  RecordedIndices[channel].emplace_back(std::make_pair(IndexInBeam, BeamIndex));
  RecordedAzEl[channel].emplace_back(std::make_pair(AzimuthAngle, ElevationAngle));
}

void AFMCWLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  // Count number of points in each channel/thread that were actually drawn (after preprocessing).
  for (auto idx_channel = 0u; idx_channel < Description.Channels; ++idx_channel)
    RecordedPointsPerChannel[idx_channel] = RecordedHits[idx_channel].size();

  FMCWLidarData.ResetMemory();

  for (size_t idx_channel = 0u; idx_channel < Description.Channels; ++idx_channel) {
    for (size_t i = 0; i < RecordedHits[idx_channel].size(); ++i) {
      FFMCWDetection detection;
      ComputeRawDetection(RecordedHits[idx_channel][i], RecordedVelocities[idx_channel][i],
                          SensorTransform, RecordedIndices[idx_channel][i], RecordedAzEl[idx_channel][i], detection);
      PostprocessDetection(detection);
      FMCWLidarData.WritePointSync(detection);
    }
  }
}

void AFMCWLidar::ComputeRawDetection(const FHitResult& HitInfo, const FVector& HitVelocity,
                                        const FTransform& SensorTransform,
                                        const std::pair<uint32_t, uint8_t>& IndexInfo,
                                        const std::pair<float, float>& AzElInfo,
                                        FFMCWDetection& Detection) const
{
  
  // Set the azimuth and elevation of the ray.
  Detection.azimuth = AzElInfo.first;
  Detection.elevation = AzElInfo.second;

  // Set the index of the point, and the index of its beam.
  Detection.point_idx = IndexInfo.first;
  Detection.beam_idx = IndexInfo.second;

  if (HitInfo.bBlockingHit) {
    const FVector HitPoint = HitInfo.ImpactPoint;
    const auto HitPointInSensorFrame = SensorTransform.Inverse().TransformPosition(HitPoint);
    Detection.range = HitPointInSensorFrame.Size() * 1e-2;
    Detection.intensity = exp(-Description.AtmospAttenRate * Detection.range);
    Detection.velocity = ComputeDopplerVelocity(HitPoint, HitVelocity);

    const FVector normal = -(HitPoint - SensorTransform.GetLocation()).GetSafeNormal();
    Detection.cos_inc_angle = FVector::DotProduct(normal, HitInfo.ImpactNormal);

    Detection.object_idx = 0;
    Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);

    const FActorRegistry& registry = GetEpisode().GetActorRegistry();
    const AActor* Actor = HitInfo.Actor.Get();
    if (Actor != nullptr) {
      Detection.dynamic = HitVelocity.Size() > 0.5;
      const FCarlaActor* view = registry.FindCarlaActor(Actor);
      if (view) {
        Detection.object_idx = view->GetActorId();
      }
    } else {
      UE_LOG(LogCarla, Warning, TEXT("Actor not valid %p!!!!"), Actor);
    }
    Detection.valid = true;
  } else {
    Detection.valid = false;
  }
}

bool AFMCWLidar::PostprocessDetection(FFMCWDetection& Detection) const
{
  if (!Detection.valid) {
    return false;
  }

  if (NoiseActive) {
    Detection.range += RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
  }

  const float Intensity = Detection.intensity;
  if(Intensity < Description.DropOffIntensityLimit){
    Detection.valid &= RandomEngine->GetUniformFloat() < (DropOffAlpha * Intensity + DropOffBeta);
  }

  return true;
}