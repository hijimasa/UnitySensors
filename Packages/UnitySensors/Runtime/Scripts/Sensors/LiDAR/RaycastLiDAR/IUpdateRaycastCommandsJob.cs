using UnityEngine;

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UnitySensors.Sensor.LiDAR
{
    // TODO: Use ComputeShader to accelerate
    [BurstCompile]
    public struct IUpdateRaycastCommandsJob : IJobParallelFor
    {
        [ReadOnly]
        public Vector3 origin;
        [ReadOnly]
        public quaternion rotation;
        [ReadOnly]
        public float maxRange;
        [ReadOnly]
        public QueryParameters queryParameters;
        [ReadOnly]
        public NativeArray<float3> directions;
        [ReadOnly]
        public int indexOffset;
        [WriteOnly]
        public NativeArray<RaycastCommand> raycastCommands;

        public void Execute(int index)
        {
            raycastCommands[index] = new(origin, math.normalize(math.mul(rotation, directions[index + indexOffset])), queryParameters, maxRange);
        }
    }
}