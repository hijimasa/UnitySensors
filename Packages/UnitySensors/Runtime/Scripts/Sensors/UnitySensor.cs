using UnityEngine;
using UnitySensors.Interface.Std;
using System.Runtime.CompilerServices;
using System;
using System.Collections;
using UnityEngine.Rendering;

[assembly: InternalsVisibleTo("UnitySensorsEditor")]
[assembly: InternalsVisibleTo("UnitySensorsROSEditor")]
namespace UnitySensors.Sensor
{
    public abstract class UnitySensor : MonoBehaviour, ITimeInterface
    {
        [SerializeField, Min(0)]
        internal float _frequency = 10.0f;
        private static int _sensor_count = 0;
        private float _time;
        private float _dt;
        private float _frequency_inv;
        private int _sensor_id;

        public Action onSensorUpdateComplete;
        public float dt { get => _frequency_inv; }
        public float time { get => _time; }
        public float frequency
        {
            get => _frequency;
            set
            {
                _frequency = Mathf.Max(value, 0);
                _frequency_inv = 1.0f / _frequency;
                InitializeSensorOffset();
            }
        }

        private void Awake()
        {
            _frequency_inv = 1.0f / _frequency;

            _sensor_id = _sensor_count;
            _sensor_count++;

            InitializeSensorOffset();

            Init();
        }

        private void InitializeSensorOffset()
        {
            string sensorType = GetType().Name;
            int typeHash = sensorType.GetHashCode();

            // Combine sensor ID and type to create a more dispersed value
            // Use coprime numbers and operations to increase dispersion
            float seed = (_sensor_id * 16777619 + typeHash) * 0.618033988749895f;

            // Ensure the offset is in [0, 1)
            float normalizedOffset = seed % 1.0f;
            if (normalizedOffset < 0) normalizedOffset += 1.0f; // Ensure non-negative

            _dt = normalizedOffset * _frequency_inv;

            // Debug.Log($"Sensor {GetType().Name} ID:{_sensor_id} initialized with offset {normalizedOffset:F3} ({_dt:F3}s)");
        }
        private void Start()
        {
            StartCoroutine(UpdateSensorPeriodically());
        }
        private IEnumerator UpdateSensorPeriodically()
        {
            while (true)
            {
                yield return new WaitUntil(() =>
                {
                    _dt += Time.deltaTime;
                    return _dt >= _frequency_inv;
                });

                _time = Time.time;

                // 更新に要した時間も周期に算入する。UpdateSensorOnce は
                // 数フレームまたぐことがある (カメラは AsyncGPUReadback の完了を
                // 待つ) ので、待っている間の経過を数えないと、実際の周期が
                // 「指定周期 + 更新時間」になってしまう。
                //
                // 実測 (960x540 のカメラ、指定 10 Hz):
                //   描画 10 FPS -> 0.400 s (2.5 Hz)  読み戻し 3 フレーム
                //   描画 30 FPS -> 0.167 s (6.0 Hz)  読み戻し 2 フレーム
                //   描画 60 FPS -> 0.133 s (7.5 Hz)  読み戻し 2 フレーム
                // いずれも「1/f + 読み戻し / 描画レート」に一致していた。
                // これで周期は max(1/f, 更新時間) になる。
                float updateStart = Time.time;
                yield return UpdateSensorOnce();
                _dt += Time.time - updateStart;

                _dt -= _frequency_inv;

                // 更新が周期より遅いセンサで _dt が際限なく積み上がると、
                // 負荷が下がった瞬間にまとめて連続更新してしまう。1 周期ぶんで頭打ちにする。
                if (_dt > _frequency_inv) _dt = _frequency_inv;
            }
        }

        private void OnDestroy()
        {
            AsyncGPUReadback.WaitAllRequests();
            OnSensorDestroy();
        }
        private void OnValidate()
        {
            frequency = _frequency;
        }

        public virtual IEnumerator UpdateSensorOnce()
        {
            yield return UpdateSensor();
            try
            {
                onSensorUpdateComplete?.Invoke();
            }
            catch (Exception e)
            {
                // A faulty subscriber (e.g. a visualizer mid-teardown) must not
                // kill this coroutine: an uncaught exception here would silently
                // stop the sensor from ever updating again.
                Debug.LogException(e);
            }
        }

        protected abstract void Init();
        protected abstract IEnumerator UpdateSensor();
        protected abstract void OnSensorDestroy();
    }
}
