// Copyright Aeva 2024

#pragma once

#include <carla/sensor/data/FMCWLidarData.h>
#include <compiler/disable-ue4-macros.h>
#include <compiler/enable-ue4-macros.h>

#include <utility>

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/FMCWLidarPattern.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "FMCWLidar.generated.h"

/// A ray-cast based FMCW LiDAR sensor.
UCLASS()
class CARLA_API AFMCWLidar : public ASensor
{
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
    void ComputeRawDetection(const FHitResult& HitInfo, const FVector& HitVelocity, const FTransform& SensorTransform,
        const std::pair<uint32_t, uint8_t>& IndexInfo, const std::pair<float, float>& AzElInfo,
        FFMCWDetection& Detection) const;

    /// Compute velocities for static and dynamic objects in the scene.
    void ComputeCurrentSensorVelocity(const float DeltaTime);
    float ComputeDopplerVelocity(const FVector& TargetPosition, const FVector& TargetVelocity) const;

    /// Saving the hits the ray-cast returns per channel.
    void WritePointAsync(uint32_t channel, uint32_t IndexInBeam, uint8_t BeamIndex, const FHitResult& HitInfo,
        float AzimuthAngle, float ElevationAngle, float DeltaTime);

    /// Clear the recorded data structure.
    void ResetRecordedData(uint32_t Channels, uint32_t PointsPerChannel);

    void StoreBoneTransforms();

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

    // Last Carla Actor Skeleton Bone Transformations by id as map
    std::map<uint32_t, std::map<FName, FTransform>> LastBoneTransforms;

    // possible Vehicle names
    const std::vector<FString> vehicleNames = { "CrossBike", "LeisureBike", "Roadbike", "FusoRosa", "AudiA2",
        "Audi_ETron", "AudiTT", "BmwGranTourer", "BmwIsetta", "ChevroletImpala", "CitroenC3", "Cybertruck",
        "DodgeCharger", "ChargerCop2020", "Charger2022", "Ford_Crown", "JeepWranglerRubicon", "SeatLeon", "LincolnMKZ",
        "Lincoln2020", "Mercedes_C", "MercedesCCC", "Mini_C", "Mini2021", "Mustang66", "NissanMicra", "Nissan_Patrol",
        "NissanPatrol2021", "TeslaM3", "ToyotaPrius", "Harley", "KawasakiNinja", "Vespa", "Yamaha", "Ambulance",
        "CarlaCola", "Firetruck", "Sprinter", "Volkswagen_T2_2021", "VolkswagenT2", "European_HGV" };

    // possible Wheel diameters in order of vehicleNames
    const std::vector<double> wheelDiameters
        = { 51.771687, 69.6461, 67.37275, 127.311659, 64.74775, 74.608378, 69.861888, 58.48821, 50.753068, 69.621273,
              66.942, 103.92945, 74.07344, 73.499, 73.499, 69.30823, 81.085018, 68.280697, 70.02123, 71.513743, 82.4174,
              67.089996, 66.48552, 73.990615, 64.458635, 63.79911, 74.81815, 88.092889, 73.631041, 71.92228, 69.96661,
              59.2533, 43.1998, 63.7978, 74.30792, 91.311769, 114.718401, 73.470064, 67.355799, 72.16835, 114.718401 };
    // possible Wheel widths in order of vehicleNames
    const std::vector<double> wheelWidths = { 7.00528, 2.70052, 2.5962, 43.384, 21.2334, 26.5474, 18.9799, 26.2078,
        18.2967, 23.7719, 24.7984, 37.4969, 27.382, 23.7765, 23.7765, 19.6371, 30.3962, 22.8786, 23.321, 22.1041,
        25.1846, 20.2643, 29.6737, 26.174, 19.6687, 23.5598, 29.0039, 31.9336, 24.4971, 22.2986, 19.09902, 13.106,
        11.33857, 15.89646, 29.7871, 38.1707, 33.521, 28.0734, 15.4773, 21.3964, 33.521 };

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
