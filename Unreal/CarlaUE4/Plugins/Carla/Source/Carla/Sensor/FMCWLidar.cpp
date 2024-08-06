// Copyright Aeva 2024

#include "Carla/Sensor/FMCWLidar.h"

#include <PxScene.h>
#include <compiler/disable-ue4-macros.h>
#include <compiler/enable-ue4-macros.h>

#include <cmath>
#include <sstream>

#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/FMCWLidarPattern.h"
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"

namespace crp = carla::rpc;

FActorDefinition AFMCWLidar::GetSensorDefinition()
{
    return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("fmcw"));
}

AFMCWLidar::AFMCWLidar(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
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

    // check if no bone transforms are stored
    // std::map<uint32_t, std::map<FName, FTransform>> LastBoneTransforms;
    if (LastBoneTransforms.empty()) {
        StoreBoneTransforms();
    }

    ComputeCurrentSensorVelocity(DeltaTime);
    SimulateLidarScanPattern(DeltaTime);
    StoreBoneTransforms();

    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
        auto DataStream = GetDataStream(*this);
        DataStream.SerializeAndSend(*this, FMCWLidarData, DataStream.PopBufferFromPool());
    }
}

void AFMCWLidar::StoreBoneTransforms()
{
    // get all carla actors
    const FActorRegistry& registry = GetEpisode().GetActorRegistry();

    for (auto It = registry.begin(); It != registry.end(); ++It) {
        // Get the Actor ID and the shared pointer to the actor
        // auto ActorId = It->Key;
        auto actor = It->Value;

        // get skeletal mesh component
        USkeletalMeshComponent* SkeletalMeshComp = actor->GetActor()->FindComponentByClass<USkeletalMeshComponent>();
        if (SkeletalMeshComp) {
            // get all bones
            const auto bones = SkeletalMeshComp->GetAllSocketNames();
            for (auto bone : bones) {
                // get body instance for bone
                FBodyInstance* BodyInstance = SkeletalMeshComp->GetBodyInstance(bone);
                if (BodyInstance) {
                    auto bone_transform = BodyInstance->GetUnrealWorldTransform();
                    LastBoneTransforms[actor->GetActorId()][bone] = bone_transform;
                }
            }
        }
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
    // CurrentSensorVelocity = GetActorVelocity();
    CurrentSensorTime = GetWorld()->TimeSeconds;
    CurrentSensorVelocity = (CurrentSensorPosition - PrevSensorPosition) / (CurrentSensorTime - PrevSensorTime);
    PrevSensorPosition = CurrentSensorPosition;
    PrevSensorTime = CurrentSensorTime;
}

float AFMCWLidar::ComputeDopplerVelocity(const FVector& TargetPosition, const FVector& TargetVelocity) const
{
    const FVector Direction = (TargetPosition - CurrentSensorPosition).GetSafeNormal();
    const FVector RelativeVelocity = (TargetVelocity - CurrentSensorVelocity);
    // Calculate the measured velocity relative to the world or the sensor.
    const float DopplerVelocity = Description.MotionCompensate ? FVector::DotProduct(TargetVelocity, Direction)
                                                               : FVector::DotProduct(RelativeVelocity, Direction);
    return DopplerVelocity * 1e-2; // Convert cm/s to m/s
}

bool AFMCWLidar::PreprocessRay() const
{
    if (DropOffGenActive) {
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

            FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
            TraceParams.bTraceComplex = true;
            TraceParams.bReturnPhysicalMaterial = false;

            for (size_t beam_idx = 0; beam_idx < LidarPattern.NumBeams; ++beam_idx) {
                size_t thread_start_idx
                    = LidarPattern.RaycastIndexOffset + thread_idx * LidarPattern.PointsPerBeamPerThread();
                size_t thread_end_idx
                    = LidarPattern.RaycastIndexOffset + (thread_idx + 1) * LidarPattern.PointsPerBeamPerThread();
                for (size_t point_idx = thread_start_idx; point_idx < thread_end_idx; ++point_idx) {
                    // Ray-cast and store the hit result.
                    FHitResult HitResult;
                    const auto& Azimuth = LidarPattern.AzimuthsDeg[beam_idx][point_idx];
                    const auto& Elevation = LidarPattern.ElevationsDeg[beam_idx][point_idx];
                    if (PreprocessRay()) {
                        ShootLaser(Azimuth, Elevation, HitResult, TraceParams);
                    }
                    // Record the point even if it is pre-dropped or is not blocking.
                    WritePointAsync(thread_idx, point_idx, beam_idx, HitResult, Azimuth, Elevation, DeltaTime);
                }
            }
        });
    }

    LidarPattern.TickRaycastIdxOffset();

    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

    FTransform ActorTransform = GetTransform();
    ComputeAndSaveDetections(ActorTransform);
}

bool AFMCWLidar::ShootLaser(const float AzimuthAngle, const float ElevationAngle, FHitResult& HitResult,
    FCollisionQueryParams& TraceParams) const
{
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

    const FTransform ActorTransform = GetTransform();
    const FVector LidarBodyLoc = ActorTransform.GetLocation();
    const FRotator LidarBodyRot = ActorTransform.Rotator();

    const FRotator LaserRot(ElevationAngle, AzimuthAngle,
        0); // float InPitch, float InYaw, float InRoll
    const FRotator ResultRot = UKismetMathLibrary::ComposeRotators(LaserRot, LidarBodyRot);

    const auto Range = Description.Range;
    FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

    FHitResult HitInfo(ForceInit);
    GetWorld()->ParallelLineTraceSingleByChannel(HitInfo, LidarBodyLoc, EndTrace, ECC_GameTraceChannel2, TraceParams,
        FCollisionResponseParams::DefaultResponseParam);

    HitResult = HitInfo;
    return true;
}

void AFMCWLidar::WritePointAsync(uint32_t channel, uint32_t IndexInBeam, uint8_t BeamIndex, const FHitResult& HitInfo,
    float AzimuthAngle, float ElevationAngle, float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    DEBUG_ASSERT(GetChannelCount() > channel);
    RecordedHits[channel].emplace_back(HitInfo);

    // Store the instantaneous actor velocity.
    FVector velocity = FVector(0.0f, 0.0f, 0.0f);
    if (HitInfo.IsValidBlockingHit()) {
        const AActor* Actor = HitInfo.GetActor();
        if (Actor != nullptr) {
            velocity = Actor->GetVelocity();
            const FActorRegistry& registry = GetEpisode().GetActorRegistry();
            const FCarlaActor* carla_actor = registry.FindCarlaActor(Actor);
            bool velocity_set = false;
            if (carla_actor) {
                // UE_LOG(LogCarla, Warning, TEXT("Hit is Actor: %s"),
                // *Actor->GetName()); auto component = HitInfo.GetComponent(); auto
                // name = component->GetReadableName(); UE_LOG(LogCarla, Warning,
                // TEXT("Hit Component name is %s"), *name);
                //   auto components = Actor->GetComponents();
                //   for (auto component : components)
                //   {
                //     auto name = component->GetReadableName();
                //     UE_LOG(LogCarla, Warning, TEXT("Component name is %s"), *name);
                //   }

                USkeletalMeshComponent* SkeletalMeshComp = Actor->FindComponentByClass<USkeletalMeshComponent>();
                if (SkeletalMeshComp) {
                    auto bones = SkeletalMeshComp->GetAllSocketNames();
                    if (bones.Num() > 0) {
                        bool is_vehicle = false;
                        for (auto bone : bones) {
                            if (bone.ToString().Contains("Wheel")) {
                                is_vehicle = true;
                                break;
                            }
                        }
                        if (is_vehicle) {
                            auto actor_name = Actor->GetName();
                            size_t i = 0;
                            for (i = 0; i < vehicleNames.size(); i++) {
                                if (actor_name.Contains(vehicleNames[i])) {
                                    break;
                                }
                            }
                            bool vehicle_is_known = i < vehicleNames.size();
                            if (!vehicle_is_known) {
                                UE_LOG(LogCarla, Warning, TEXT("Vehicle name not found: %s"), *actor_name);
                            } else {
                                float wheelRadius = wheelDiameters[i] / 2.0f * 1.03;
                                float wheelWidth = wheelWidths[i] * 1.2;

                                for (auto bone : bones) {
                                    if (bone.ToString().Contains("Wheel")) {
                                        auto bone_transform_socket = SkeletalMeshComp->GetSocketTransform(bone);
                                        // this causes Problems with riders:
                                        // SkeletalMeshComp->GetBodyInstance(bone)->GetUnrealWorldTransform()
                                        // auto bone_transform =
                                        // SkeletalMeshComp->GetBodyInstance(bone)->GetUnrealWorldTransform();
                                        // auto bone_location = bone_transform.GetLocation();
                                        // auto bone_rotation = bone_transform.GetRotation();
                                        auto bone_rotation_socket = bone_transform_socket.GetRotation();
                                        auto bone_location_socket = bone_transform_socket.GetLocation();
                                        auto actor_location = Actor->GetActorLocation();
                                        auto actor_rotation = Actor->GetActorRotation();
                                        auto actor_velocity = Actor->GetVelocity();
                                        // get wheel location
                                        FVector wheelLocation = bone_transform_socket.GetLocation();
                                        // get hit location
                                        FVector hitLocation = HitInfo.ImpactPoint;

                                        // check if hit point is in wheel
                                        auto difference_vector = hitLocation - wheelLocation;
                                        // UE_LOG(LogCarla, Warning, TEXT("Difference Vector is %f,
                                        // %f, %f"), difference_vector.X, difference_vector.Y,
                                        // difference_vector.Z);
                                        auto wheel_rotation = bone_transform_socket.GetRotation();
                                        // rotate difference vector by inverse of wheel rotation
                                        // auto rotated_difference_vector =
                                        // wheel_rotation.Inverse().RotateVector(difference_vector);
                                        auto rotated_difference_vector
                                            = actor_rotation.UnrotateVector(difference_vector);
                                        // UE_LOG(LogCarla, Warning, TEXT("Rotated Difference Vector
                                        // is %f, %f, %f"), rotated_difference_vector.X,
                                        // rotated_difference_vector.Y,
                                        // rotated_difference_vector.Z);
                                        //   check if hit point is in wheel
                                        //   we cant know if is right wheel or left wheel, so the
                                        //   whole width in both directions is considered
                                        if (rotated_difference_vector.Y > -wheelWidth
                                            && rotated_difference_vector.Y < wheelWidth) {
                                            auto radial_distance
                                                = sqrt(rotated_difference_vector.X * rotated_difference_vector.X
                                                    + rotated_difference_vector.Z * rotated_difference_vector.Z);
                                            if (radial_distance < wheelRadius) {
                                                // UE_LOG(LogCarla, Warning, TEXT("Hit point is in
                                                // wheel")); UE_LOG(LogCarla, Warning, TEXT("Car is
                                                // %s"), *vehicleNames[i]); UE_LOG(LogCarla, Warning,
                                                // TEXT("Wheel radius is %f"), wheelRadius);
                                                // UE_LOG(LogCarla, Warning, TEXT("Wheel width is %f"),
                                                // wheelWidth); UE_LOG(LogCarla, Warning, TEXT("Wheel
                                                // location is %f, %f, %f"), wheelLocation.X,
                                                // wheelLocation.Y, wheelLocation.Z); UE_LOG(LogCarla,
                                                // Warning, TEXT("Hit location is %f, %f, %f"),
                                                // hitLocation.X, hitLocation.Y, hitLocation.Z);
                                                // UE_LOG(LogCarla, Warning, TEXT("Wheel rotation is %f,
                                                // %f, %f, %f"), wheel_rotation.X, wheel_rotation.Y,
                                                // wheel_rotation.Z, wheel_rotation.W);
                                                auto actor_rotation = Actor->GetActorRotation();
                                                // UE_LOG(LogCarla, Warning, TEXT("Actor rotation is %f,
                                                // %f, %f"), actor_rotation.Pitch, actor_rotation.Yaw,
                                                // actor_rotation.Roll);
                                                //   hit point is in wheel
                                                UPrimitiveComponent* Primitive
                                                    = Cast<UPrimitiveComponent>(Actor->GetRootComponent());
                                                if (Primitive) {
                                                    auto velocity_wo_wheel = Primitive->GetPhysicsLinearVelocityAtPoint(
                                                        HitInfo.ImpactPoint, HitInfo.BoneName);
                                                    // UE_LOG(LogCarla, Warning, TEXT("Vehicle at point is
                                                    // %f, %f, %f"), velocity_wo_wheel.X,
                                                    // velocity_wo_wheel.Y, velocity_wo_wheel.Z);
                                                    auto actor_velocity = Actor->GetVelocity();
                                                    actor_velocity.Z = 0.0f;
                                                    // UE_LOG(LogCarla, Warning, TEXT("Actor velocity is
                                                    // %f, %f, %f"), actor_velocity.X, actor_velocity.Y,
                                                    // actor_velocity.Z); assume the velocity is always in
                                                    // the direction of the vehicle
                                                    auto wheel_angular_velocity = actor_velocity.Size() / wheelRadius;
                                                    // UE_LOG(LogCarla, Warning, TEXT("Wheel angular
                                                    // velocity is %f"), wheel_angular_velocity);
                                                    //   asume wheel is rotated at z the same as actor...
                                                    //   this is wrong...
                                                    auto wheel_angular_velocity_vector
                                                        = FVector(0.0f, wheel_angular_velocity, 0.0f);
                                                    auto wheel_velocity = FVector::CrossProduct(
                                                        wheel_angular_velocity_vector, rotated_difference_vector);
                                                    wheel_velocity = actor_rotation.RotateVector(wheel_velocity);
                                                    // UE_LOG(LogCarla, Warning, TEXT("Point velocity on
                                                    // wheel is %f, %f, %f"), wheel_velocity.X,
                                                    // wheel_velocity.Y, wheel_velocity.Z);
                                                    velocity = velocity_wo_wheel + wheel_velocity;
                                                    // UE_LOG(LogCarla, Warning, TEXT("Final velocity is
                                                    // %f, %f, %f"), velocity.X, velocity.Y, velocity.Z);
                                                    velocity_set = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        } else {
                            // has bones but is not a vehicle -> pedestrian
                            if (!HitInfo.BoneName.IsNone()) {
                                // UE_LOG(LogCarla, Warning, TEXT("Hit has Bone %s"),
                                // *HitInfo.BoneName.ToString());
                                auto lastTransform = LastBoneTransforms[carla_actor->GetActorId()][HitInfo.BoneName];
                                FBodyInstance* BodyInstance = SkeletalMeshComp->GetBodyInstance(HitInfo.BoneName);

                                auto body = SkeletalMeshComp->GetBodyInstance(HitInfo.BoneName);
                                if (body) {
                                    // auto owner = body->OwnerComponent;

                                    // UE_LOG(LogCarla, Warning, TEXT("Owner name is %s"),
                                    // *(owner->GetReadableName()));

                                    auto bone_transform = body->GetUnrealWorldTransform();
                                    auto differential_translation
                                        = bone_transform.GetLocation() - lastTransform.GetLocation();
                                    auto differential_velocity = differential_translation / DeltaTime;
                                    auto lever_arm = HitInfo.ImpactPoint - bone_transform.GetLocation();
                                    auto differential_rotation
                                        = bone_transform.GetRotation() * lastTransform.GetRotation().Inverse();
                                    auto diff_angle = differential_rotation.GetAngle();
                                    if (diff_angle > PI) {
                                        diff_angle -= 2 * PI;
                                    }
                                    auto diff_axis = differential_rotation.GetRotationAxis();
                                    auto differential_angular_velocity = diff_angle * diff_axis / DeltaTime;
                                    auto velocity_angular_component
                                        = FVector::CrossProduct(differential_angular_velocity, lever_arm);
                                    // if (differential_angular_velocity.Size() > 5.0)
                                    // {
                                    //   UE_LOG(LogCarla, Warning,
                                    //   TEXT("==================================================="));
                                    //   UE_LOG(LogCarla, Warning, TEXT("Hit is Actor: %s"),
                                    //   *Actor->GetName()); UE_LOG(LogCarla, Warning, TEXT("Bone
                                    //   name is %s"), *HitInfo.BoneName.ToString());
                                    //   UE_LOG(LogCarla, Warning, TEXT("Last Rot is %f, %f, %f,
                                    //   %f"), lastTransform.GetRotation().X,
                                    //   lastTransform.GetRotation().Y,
                                    //   lastTransform.GetRotation().Z,
                                    //   lastTransform.GetRotation().W); UE_LOG(LogCarla, Warning,
                                    //   TEXT("This Rot is %f, %f, %f, %f"),
                                    //   bone_transform.GetRotation().X,
                                    //   bone_transform.GetRotation().Y,
                                    //   bone_transform.GetRotation().Z,
                                    //   bone_transform.GetRotation().W); UE_LOG(LogCarla,
                                    //   Warning, TEXT("Diff Rot is %f, %f, %f"),
                                    //   differential_rotation.X, differential_rotation.Y,
                                    //   differential_rotation.Z, differential_rotation.W);
                                    //   UE_LOG(LogCarla, Warning, TEXT("Diff Rot Ax Ang is %f,
                                    //   %f, %f, %f"), diff_axis.X, diff_axis.Y, diff_axis.Z,
                                    //   diff_angle); UE_LOG(LogCarla, Warning, TEXT("Angular
                                    //   Velocity is %f, %f, %f"),
                                    //   differential_angular_velocity.X,
                                    //   differential_angular_velocity.Y,
                                    //   differential_angular_velocity.Z);
                                    // }

                                    velocity_set = true;
                                    velocity = differential_velocity + velocity_angular_component;
                                } else {
                                    UE_LOG(LogCarla, Error, TEXT("Failed to get body instance for actor %s"),
                                        *Actor->GetName());
                                    UE_LOG(LogCarla, Error, TEXT("Failed to get body instance for bone %s"),
                                        *HitInfo.BoneName.ToString());
                                }
                            }
                        }
                    }
                }
                // else
                // {
                //   UE_LOG(LogCarla, Error, TEXT("Failed to find skeletal mesh
                //   component for actor %s"), *Actor->GetName());
                // }
            }

            if (!velocity_set) {
                UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Actor->GetRootComponent());
                if (Primitive) {
                    // if (carla_actor)
                    // {
                    //   UE_LOG(LogCarla, Warning, TEXT("Hit has primitive component:
                    //   %s"), *Primitive->GetName());
                    // }
                    velocity = Primitive->GetPhysicsLinearVelocityAtPoint(HitInfo.ImpactPoint);
                } else {
                    velocity = Actor->GetVelocity();
                }
            }
        } else {
            UE_LOG(LogCarla, Error, TEXT("HitInfo actor is nullptr"));
        }
    }
    // TODO: Check frame of TurnRate
    // UPrimitiveComponent *Primitive =
    //     Cast<UPrimitiveComponent>(Actor->GetRootComponent());
    // if (Primitive)
    // {
    //   FVector velocity_at_impact =
    //   Primitive->GetPhysicsLinearVelocityAtPoint(HitInfo.ImpactPoint,
    //   HitInfo.BoneName); velocity = velocity_at_impact;
    // }
    // FVector angular_velocity = Primitive->GetPhysicsAngularVelocityInRadians();
    // angular_velocity =
    // Actor->GetActorRotation().RotateVector(angular_velocity); FVector
    // actor_location = Actor->GetActorLocation(); FVector hit_location =
    // HitInfo.ImpactPoint; FVector point_location_on_actor = hit_location -
    // actor_location; velocity = velocity +
    // FVector::CrossProduct(angular_velocity, point_location_on_actor);
    //   }
    //   if (!HitInfo.BoneName.IsNone())
    //   {
    //     // const FActorRegistry &registry = GetEpisode().GetActorRegistry();
    //     // const FCarlaActor *carla_actor = registry.FindCarlaActor(Actor);
    //     USkeletalMeshComponent *SkeletalMeshComp =
    //     Actor->FindComponentByClass<USkeletalMeshComponent>(); if
    //     (!SkeletalMeshComp)

    //       // Check if the bone exists
    //       if (!SkeletalMeshComp->DoesSocketExist(HitInfo.BoneName))
    //       {
    //         UE_LOG(LogTemp, Warning, TEXT("Bone %s does not exist in the
    //         skeletal mesh"), *HitInfo.BoneName.ToString());
    //       }

    //     // Find the body instance for the bone
    //     FBodyInstance *BodyInstance =
    //     SkeletalMeshComp->GetBodyInstance(HitInfo.BoneName); if (!BodyInstance)
    //     {
    //       UE_LOG(LogTemp, Warning, TEXT("No body instance found for bone %s"),
    //       *HitInfo.BoneName.ToString());
    //     }

    //     // Ensure the body instance is simulated
    //     if (!BodyInstance->IsInstanceSimulatingPhysics())
    //     {
    //       UE_LOG(LogTemp, Warning, TEXT("Body instance for bone %s is not
    //       simulating physics"), *HitInfo.BoneName.ToString());
    //       // BodyInstance->SetInstanceSimulatePhysics(true);
    //     }
    //     // SkeletalMeshComp->SetPhysicsBlendWeight(0.001f); // Blend weight
    //     of 1.0 means fully kinematic
    //     // SkeletalMeshComp->SetAllBodiesBelowPhysicsBlendWeight(NAME_None,
    //     0.001f, true);
    //     // SkeletalMeshComp->SetAllBodiesSimulatePhysics(true);
    //     SkeletalMeshComp->bSkipKinematicUpdateWhenInterpolating = false;
    //     SkeletalMeshComp->bUpdateJointsFromAnimation = true;
    //     auto hit_component = HitInfo.GetComponent();
    //     auto body = SkeletalMeshComp->GetBodyInstance(HitInfo.BoneName);
    //     auto point_velolcity =
    //     body->GetUnrealWorldVelocityAtPoint(HitInfo.ImpactPoint); auto
    //     bone_angular_velocity = body->GetUnrealWorldAngularVelocityInRadians();
    //     auto bone_velocity = body->GetUnrealWorldVelocity();
    //     auto bone_transform = body->GetUnrealWorldTransform();
    //     auto bone_location = bone_transform.GetLocation();
    //     auto owner = body->OwnerComponent;
    //     // UE_LOG(LogCarla, Warning, TEXT("Owner name is %s"),
    //     *(owner->GetReadableName())); UE_LOG(LogCarla, Warning, TEXT("Point
    //     Velocity is: %f, %f, %f"), point_velolcity.X, point_velolcity.Y,
    //     point_velolcity.Z);
    //     // UE_LOG(LogCarla, Warning, TEXT("Hit Component name is %s"),
    //     *(hit_component->GetReadableName()));
    //     // UE_LOG(LogCarla, Warning, TEXT("Bone name is %s"),
    //     *(HitInfo.BoneName.ToString())); UE_LOG(LogCarla, Warning, TEXT("Bone
    //     angular velocity is %f, %f, %f"), bone_angular_velocity.X,
    //     bone_angular_velocity.Y, bone_angular_velocity.Z); UE_LOG(LogCarla,
    //     Warning, TEXT("Bone velocity is %f, %f, %f"), bone_velocity.X,
    //     bone_velocity.Y, bone_velocity.Z); UE_LOG(LogCarla, Warning, TEXT("Bone
    //     location is %f, %f, %f"), bone_location.X, bone_location.Y,
    //     bone_location.Z); auto components = Actor->GetComponents(); for (auto
    //     component : components)
    //     {
    //       auto name = component->GetReadableName();
    //       UE_LOG(LogCarla, Warning, TEXT("Component name is %s"), *name);
    //     }
    //   }
    // }
    // ---------------------------------------
    // UPrimitiveComponent *HitComponent = HitInfo.GetComponent();
    // if (HitComponent != nullptr)
    // {
    //   const FVector HitComponentVelocity =
    //   HitComponent->GetPhysicsLinearVelocityAtPoint(HitInfo.ImpactPoint,
    //   HitInfo.BoneName); const FVector component_velocity =
    //   HitComponent->GetComponentVelocity(); const FVector physics_velocity =
    //   HitComponent->GetPhysicsLinearVelocity(); velocity =
    //   HitComponentVelocity; if (!HitInfo.BoneName.IsNone())
    //   {
    //     UE_LOG(LogCarla, Warning, TEXT("Bonename is %s"),
    //     *HitInfo.BoneName.ToString()); UE_LOG(LogCarla, Warning, TEXT("velocity
    //     is %f, %f, %f"), velocity.X, velocity.Y, velocity.Z); UE_LOG(LogCarla,
    //     Warning, TEXT("component velocity is %f, %f, %f"),
    //     component_velocity.X, component_velocity.Y, component_velocity.Z);
    //   }
    // }
    // else
    // {
    //   UE_LOG(LogCarla, Warning, TEXT("HitComponent not valid !!!!"));
    // }
    // else if (HitInfo.GetActor() != nullptr)
    // {
    //   velocity = HitInfo.GetActor()->GetVelocity();
    // }

    RecordedVelocities[channel].emplace_back(velocity);
    RecordedIndices[channel].emplace_back(std::make_pair(IndexInBeam, BeamIndex));
    RecordedAzEl[channel].emplace_back(std::make_pair(AzimuthAngle, ElevationAngle));
}

void AFMCWLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    // Count number of points in each channel/thread that were actually drawn
    // (after preprocessing).
    for (auto idx_channel = 0u; idx_channel < Description.Channels; ++idx_channel)
        RecordedPointsPerChannel[idx_channel] = RecordedHits[idx_channel].size();

    FMCWLidarData.ResetMemory();

    for (size_t idx_channel = 0u; idx_channel < Description.Channels; ++idx_channel) {
        for (size_t i = 0; i < RecordedHits[idx_channel].size(); ++i) {
            FFMCWDetection detection;
            ComputeRawDetection(RecordedHits[idx_channel][i], RecordedVelocities[idx_channel][i], SensorTransform,
                RecordedIndices[idx_channel][i], RecordedAzEl[idx_channel][i], detection);
            PostprocessDetection(detection);
            FMCWLidarData.WritePointSync(detection);
        }
    }
}

void AFMCWLidar::ComputeRawDetection(const FHitResult& HitInfo, const FVector& HitVelocity,
    const FTransform& SensorTransform, const std::pair<uint32_t, uint8_t>& IndexInfo,
    const std::pair<float, float>& AzElInfo, FFMCWDetection& Detection) const
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
    if (Intensity < Description.DropOffIntensityLimit) {
        Detection.valid &= RandomEngine->GetUniformFloat() < (DropOffAlpha * Intensity + DropOffBeta);
    }

    return true;
}