using System;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using UnitySensors.DataType.Sensor;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Utils.Noise;

using Random = Unity.Mathematics.Random;
using System.Collections;
using UnitySensors.Utils.Texture;
using Unity.Burst;

#if UNITY_6000_0_OR_NEWER
using UnityEngine.Rendering;
#endif

namespace UnitySensors.Sensor.Camera
{
    /// <summary>
    /// Burst-compiled job to generate raycast commands for depth image generation
    /// </summary>
    [BurstCompile]
    public struct GenerateRaycastCommandsJob : IJobParallelFor
    {
        [ReadOnly] public float3 cameraPosition;
        [ReadOnly] public float3 forward;
        [ReadOnly] public float3 right;
        [ReadOnly] public float3 up;
        [ReadOnly] public float tanHalfFov;
        [ReadOnly] public float aspect;
        [ReadOnly] public int width;
        [ReadOnly] public int height;
        [ReadOnly] public float farClipPlane;
        [ReadOnly] public QueryParameters queryParameters;

        [WriteOnly] public NativeArray<RaycastCommand> raycastCommands;

        public void Execute(int index)
        {
            int x = index % width;
            int y = index / width;

            float normalizedX = (float)x / (width - 1);
            float normalizedY = (float)y / (height - 1);

            float ndcX = (2.0f * normalizedX) - 1.0f;
            float ndcY = (2.0f * normalizedY) - 1.0f;

            float viewX = ndcX * tanHalfFov * aspect;
            float viewY = ndcY * tanHalfFov;

            float3 rayDirection = math.normalize(forward + right * viewX + up * viewY);

            raycastCommands[index] = new RaycastCommand(cameraPosition, rayDirection, queryParameters, farClipPlane);
        }
    }

    /// <summary>
    /// Burst-compiled job to convert raycast hits to depth pixels
    /// </summary>
    [BurstCompile]
    public struct ConvertHitsToDepthJob : IJobParallelFor
    {
        [ReadOnly] public float farClipPlane;
        [ReadOnly] public NativeArray<RaycastHit> raycastHits;

        [WriteOnly] public NativeArray<Color32> pixels;

        public void Execute(int index)
        {
            float depth = 1.0f;
            RaycastHit hit = raycastHits[index];

            // Note: Cannot use hit.collider in Burst-compiled job (managed API)
            // Use distance > 0 to check for valid hits instead
            if (hit.distance > 0)
            {
                depth = math.clamp(hit.distance / farClipPlane, 0f, 1f);
            }

            byte depthByte = (byte)(depth * 255);
            pixels[index] = new Color32(depthByte, depthByte, depthByte, 255);
        }
    }

    [RequireComponent(typeof(UnityEngine.Camera))]
    public class DepthCameraSensor : CameraSensor, IPointCloudInterface<PointXYZ>
    {
        [SerializeField]
        protected float _minRange = 0.05f;
        [SerializeField]
        protected float _maxRange = 100.0f;
        [SerializeField]
        private float _gaussianNoiseSigma = 0.0f;
        [SerializeField]
        private Material _depthCameraMat;
        [SerializeField]
        private bool _convertToPointCloud = false;

        [Header("Performance Settings")]
        [SerializeField, Range(0.1f, 1.0f)]
        private float _raycastResolutionScale = 0.5f; // Reduce raycast resolution for better performance
        [SerializeField]
        private bool _useAdaptiveQuality = true; // Enable adaptive quality based on frame rate

        private TextureLoader _textureLoader;
        private Texture2D _depthTexture; // Reuse texture to avoid allocations
        private int _lastRaycastWidth, _lastRaycastHeight;
        private float _lastFrameTime;
        private Color32[] _pixelBuffer; // Pooled pixel buffer to avoid GC allocations

        // Batched raycast resources for URP depth generation
        private NativeArray<RaycastCommand> _raycastCommands;
        private NativeArray<RaycastHit> _raycastHits;
        private NativeArray<Color32> _nativePixelBuffer;
        private JobHandle _raycastJobHandle;

        private JobHandle _jobHandle;

        private IUpdateGaussianNoisesJob _updateGaussianNoisesJob;
        private ITextureToPointsJob _textureToPointsJob;

        private NativeArray<float> _noises;
        private NativeArray<float3> _directions;

        private PointCloud<PointXYZ> _pointCloud;
        private int _pointsNum;

        public PointCloud<PointXYZ> pointCloud { get => _pointCloud; }
        public int pointsNum { get => _pointsNum; }

        public bool convertToPointCloud
        {
            get => _convertToPointCloud;
            set
            {
                // Support enabling after Init() (e.g. when a visualizer is
                // attached at runtime): allocate the point cloud lazily.
                // _texture is only set from Init(), so before Init() the flag
                // alone is enough and Init() allocates as before.
                if (value && !_directions.IsCreated && _texture != null)
                {
                    SetupDirections();
                    SetupJob();
                }
                _convertToPointCloud = value;
            }
        }

        /// <summary>
        /// Configure depth camera sensor at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(Vector2Int resolution, float fov, float minRange, float maxRange)
        {
            base.Configure(resolution, fov);
            _minRange = minRange;
            _maxRange = maxRange;
        }

        protected override void Init()
        {
            base.Init();
            _camera.nearClipPlane = _minRange;
            _camera.farClipPlane = _maxRange;

#if UNITY_6000_0_OR_NEWER
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGBFloat);
            _rt.Create();

            bool isURP = GraphicsSettings.currentRenderPipeline != null;
            if (isURP)
            {
                Debug.Log("DepthCameraSensor: Unity 6000+ URP mode initialized");
            }
#else
            _rt = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.ARGBFloat);
#endif
            _camera.targetTexture = _rt;

            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBAFloat, false);

            _textureLoader = new TextureLoader
            {
                source = _rt,
                destination = _texture
            };

            if (_convertToPointCloud)
            {
                SetupDirections();
                SetupJob();
            }
        }

        private void SetupDirections()
        {
            _pointsNum = _resolution.x * _resolution.y;

            _directions = new NativeArray<float3>(_pointsNum, Allocator.Persistent);

            float z = _resolution.y * 0.5f / Mathf.Tan(m_camera.fieldOfView * 0.5f * Mathf.Deg2Rad);
            for (int y = 0; y < _resolution.y; y++)
            {
                for (int x = 0; x < _resolution.x; x++)
                {
                    Vector3 vec = new Vector3(-_resolution.x / 2 + x, -_resolution.y / 2 + y, z);
                    _directions[y * _resolution.x + x] = vec.normalized;
                }
            }
        }

        private void SetupJob()
        {
            _pointCloud = new PointCloud<PointXYZ>()
            {
                points = new NativeArray<PointXYZ>(_pointsNum, Allocator.Persistent)
            };

            _noises = new NativeArray<float>(pointsNum, Allocator.Persistent);

            _updateGaussianNoisesJob = new IUpdateGaussianNoisesJob()
            {
                sigma = _gaussianNoiseSigma,
                random = new Random((uint)Environment.TickCount),
                noises = _noises
            };

            _textureToPointsJob = new ITextureToPointsJob()
            {
                near = m_camera.nearClipPlane,
                far = m_camera.farClipPlane,
                directions = _directions,
                depthPixels = _texture.GetPixelData<Color>(0),
                noises = _noises,
                points = _pointCloud.points
            };
        }

        protected override IEnumerator UpdateSensor()
        {
#if UNITY_6000_0_OR_NEWER
            bool isURP = GraphicsSettings.currentRenderPipeline != null;

            if (isURP)
            {
                GenerateDepthImageUsingRaycast();
            }
            else
            {
                _camera.Render();
            }
#else
            _camera.Render();
#endif

            yield return _textureLoader.LoadTextureAsync();

            if (_textureLoader.success && _convertToPointCloud)
            {
                JobHandle updateGaussianNoisesJobHandle = _updateGaussianNoisesJob.Schedule(_pointsNum, 2048);
                _jobHandle = _textureToPointsJob.Schedule(_pointsNum, 2048, updateGaussianNoisesJobHandle);

                // Yield until job is completed instead of blocking (allows other work to continue)
                yield return new WaitUntil(() => _jobHandle.IsCompleted);
                _jobHandle.Complete(); // Final sync to ensure completion
            }
        }

        private void GenerateDepthImageUsingRaycast()
        {
            // Adaptive quality adjustment based on frame rate
            if (_useAdaptiveQuality)
            {
                float currentFrameTime = Time.unscaledDeltaTime;
                if (_lastFrameTime > 0)
                {
                    float currentFPS = 1.0f / currentFrameTime;
                    float targetFPS = frequency; // Use sensor frequency as target
                    if (currentFPS < targetFPS * 0.8f) // If FPS drops below 80% of target
                    {
                        _raycastResolutionScale = Mathf.Max(0.1f, _raycastResolutionScale - 0.05f);
                    }
                    else if (currentFPS > targetFPS * 1.1f) // If FPS is above 110% of target
                    {
                        _raycastResolutionScale = Mathf.Min(1.0f, _raycastResolutionScale + 0.02f);
                    }
                }
                _lastFrameTime = currentFrameTime;
            }

            // Calculate actual raycast resolution
            int raycastWidth = Mathf.Max(1, Mathf.RoundToInt(_rt.width * _raycastResolutionScale));
            int raycastHeight = Mathf.Max(1, Mathf.RoundToInt(_rt.height * _raycastResolutionScale));

            // Reuse texture and native buffers to avoid allocations
            int requiredBufferSize = raycastWidth * raycastHeight;
            if (_depthTexture == null || _lastRaycastWidth != raycastWidth || _lastRaycastHeight != raycastHeight)
            {
                // Dispose old native arrays
                if (_raycastCommands.IsCreated) _raycastCommands.Dispose();
                if (_raycastHits.IsCreated) _raycastHits.Dispose();
                if (_nativePixelBuffer.IsCreated) _nativePixelBuffer.Dispose();

                if (_depthTexture != null)
                    DestroyImmediate(_depthTexture);

                _depthTexture = new Texture2D(raycastWidth, raycastHeight, TextureFormat.RGBAFloat, false);
                _pixelBuffer = new Color32[requiredBufferSize]; // Reallocate only when size changes

                // Allocate persistent native arrays for batched raycast
                _raycastCommands = new NativeArray<RaycastCommand>(requiredBufferSize, Allocator.Persistent);
                _raycastHits = new NativeArray<RaycastHit>(requiredBufferSize, Allocator.Persistent);
                _nativePixelBuffer = new NativeArray<Color32>(requiredBufferSize, Allocator.Persistent);

                _lastRaycastWidth = raycastWidth;
                _lastRaycastHeight = raycastHeight;
            }

            RenderTexture.active = _rt;
            GL.Clear(true, true, Color.white);
            RenderTexture.active = null;

            // Pre-calculate camera parameters
            float fovRad = _camera.fieldOfView * Mathf.Deg2Rad;
            float aspect = (float)_rt.width / _rt.height;
            float tanHalfFov = Mathf.Tan(fovRad * 0.5f);

            Vector3 cameraPos = _camera.transform.position;
            Vector3 forward = _camera.transform.forward;
            Vector3 right = _camera.transform.right;
            Vector3 up = _camera.transform.up;

            // Use Burst-compiled job to generate raycast commands in parallel
            var generateJob = new GenerateRaycastCommandsJob
            {
                cameraPosition = cameraPos,
                forward = forward,
                right = right,
                up = up,
                tanHalfFov = tanHalfFov,
                aspect = aspect,
                width = raycastWidth,
                height = raycastHeight,
                farClipPlane = _camera.farClipPlane,
                queryParameters = new QueryParameters { layerMask = Physics.DefaultRaycastLayers },
                raycastCommands = _raycastCommands
            };

            // Schedule raycast command generation job
            JobHandle generateHandle = generateJob.Schedule(requiredBufferSize, 2048);

            // Schedule batched raycasts (runs in parallel on worker threads)
            _raycastJobHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHits, 2048, 1, generateHandle);

            // Convert hits to depth pixels using Burst-compiled job
            var convertJob = new ConvertHitsToDepthJob
            {
                farClipPlane = _camera.farClipPlane,
                raycastHits = _raycastHits,
                pixels = _nativePixelBuffer
            };

            JobHandle convertHandle = convertJob.Schedule(requiredBufferSize, 2048, _raycastJobHandle);
            convertHandle.Complete();

            // Copy native buffer to managed array for texture upload
            _nativePixelBuffer.CopyTo(_pixelBuffer);

            // Apply pixels and scale to target resolution
            _depthTexture.SetPixels32(_pixelBuffer);
            _depthTexture.Apply();

            // Scale to target resolution if needed
            if (raycastWidth != _rt.width || raycastHeight != _rt.height)
            {
                RenderTexture tempRT = RenderTexture.GetTemporary(_rt.width, _rt.height, 0, RenderTextureFormat.ARGBFloat);
                Graphics.Blit(_depthTexture, tempRT);
                Graphics.CopyTexture(tempRT, _rt);
                RenderTexture.ReleaseTemporary(tempRT);
            }
            else
            {
                RenderTexture.active = _rt;
                Graphics.CopyTexture(_depthTexture, _rt);
                RenderTexture.active = null;
            }
        }

        protected override void OnSensorDestroy()
        {
            // Dispose on allocation, not on the flag: the flag may have been
            // turned off again after a runtime visualizer allocated the arrays.
            if (_directions.IsCreated)
            {
                _jobHandle.Complete();
                _pointCloud.Dispose();
                _noises.Dispose();
                _directions.Dispose();
            }

            // Clean up batched raycast resources
            if (_raycastCommands.IsCreated) _raycastCommands.Dispose();
            if (_raycastHits.IsCreated) _raycastHits.Dispose();
            if (_nativePixelBuffer.IsCreated) _nativePixelBuffer.Dispose();

            // Clean up depth texture
            if (_depthTexture != null)
            {
                DestroyImmediate(_depthTexture);
                _depthTexture = null;
            }

            _rt.Release();
        }

        /// <summary>
        /// Completes any pending job that writes to pointCloud.
        /// Must be called before reading from pointCloud.points to avoid race conditions.
        /// </summary>
        public void CompleteJob()
        {
            _jobHandle.Complete();
        }

#if !UNITY_6000_0_OR_NEWER
        private void OnRenderImage(RenderTexture source, RenderTexture dest)
        {
            Graphics.Blit(null, dest, _depthCameraMat);
        }
#endif
    }
}
