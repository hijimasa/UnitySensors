using UnityEditor;
using UnitySensors.Sensor.Camera;

namespace UnitySensors.Editor
{
    [CustomEditor(typeof(FisheyeCameraSensor))]
    public class FisheyeCameraEditor : UnityEditor.Editor
    {
        SerializedProperty cameraModelProp;
        SerializedProperty alphaProp;
        SerializedProperty betaProp;
        SerializedProperty xiProp;
        SerializedProperty kb4Prop;
        SerializedProperty affineCoeffsProp;
        SerializedProperty _a0Prop;
        SerializedProperty _a1Prop;
        SerializedProperty _a2Prop;
        SerializedProperty _a3Prop;
        SerializedProperty _a4Prop;
        SerializedProperty focalLengthProp;
        SerializedProperty principalPointProp;
        readonly string cameraModelLabel = nameof(FisheyeCameraSensor._cameraModel);
        readonly string alphaLabel = nameof(FisheyeCameraSensor._alpha);
        readonly string betaLabel = nameof(FisheyeCameraSensor._beta);
        readonly string xiLabel = nameof(FisheyeCameraSensor._xi);
        readonly string kb4Label = nameof(FisheyeCameraSensor._kb4);
        readonly string affineCoeffsLabel = nameof(FisheyeCameraSensor._affineCoeffs);
        readonly string _a0Label = nameof(FisheyeCameraSensor._a0);
        readonly string _a1Label = nameof(FisheyeCameraSensor._a1);
        readonly string _a2Label = nameof(FisheyeCameraSensor._a2);
        readonly string _a3Label = nameof(FisheyeCameraSensor._a3);
        readonly string _a4Label = nameof(FisheyeCameraSensor._a4);
        readonly string focalLengthLabel = nameof(FisheyeCameraSensor._focalLength);
        readonly string principalPointLabel = nameof(FisheyeCameraSensor._principalPoint);
        readonly string fovLabel = nameof(FisheyeCameraSensor._fov);
        readonly string scriptLabel = "m_Script";

        void OnEnable()
        {
            cameraModelProp = serializedObject.FindProperty(cameraModelLabel);
            alphaProp = serializedObject.FindProperty(alphaLabel);
            betaProp = serializedObject.FindProperty(betaLabel);
            xiProp = serializedObject.FindProperty(xiLabel);
            kb4Prop = serializedObject.FindProperty(kb4Label);
            affineCoeffsProp = serializedObject.FindProperty(affineCoeffsLabel);
            _a0Prop = serializedObject.FindProperty(_a0Label);
            _a1Prop = serializedObject.FindProperty(_a1Label);
            _a2Prop = serializedObject.FindProperty(_a2Label);
            _a3Prop = serializedObject.FindProperty(_a3Label);
            _a4Prop = serializedObject.FindProperty(_a4Label);
            focalLengthProp = serializedObject.FindProperty(focalLengthLabel);
            principalPointProp = serializedObject.FindProperty(principalPointLabel);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty(scriptLabel));
            EditorGUI.EndDisabledGroup();

            DrawPropertiesExcluding(serializedObject,
                cameraModelLabel, alphaLabel, betaLabel, xiLabel, kb4Label, affineCoeffsLabel, _a0Label, _a1Label, _a2Label, _a3Label, _a4Label, focalLengthLabel, principalPointLabel, fovLabel, scriptLabel);

            EditorGUILayout.PropertyField(cameraModelProp);
            switch (cameraModelProp.enumValueIndex)
            {
                case (int)FisheyeCameraSensor.CameraModel.UCM:
                    EditorGUILayout.PropertyField(alphaProp);
                    break;
                case (int)FisheyeCameraSensor.CameraModel.EUCM:
                    EditorGUILayout.PropertyField(alphaProp);
                    EditorGUILayout.PropertyField(betaProp);
                    break;
                case (int)FisheyeCameraSensor.CameraModel.DS:
                    EditorGUILayout.PropertyField(alphaProp);
                    EditorGUILayout.PropertyField(xiProp);
                    break;
                case (int)FisheyeCameraSensor.CameraModel.KB4:
                    EditorGUILayout.PropertyField(kb4Prop);
                    break;
                case (int)FisheyeCameraSensor.CameraModel.OCAM:
                    EditorGUILayout.PropertyField(affineCoeffsProp);
                    EditorGUILayout.PropertyField(_a0Prop);
                    EditorGUILayout.PropertyField(_a1Prop);
                    EditorGUILayout.PropertyField(_a2Prop);
                    EditorGUILayout.PropertyField(_a3Prop);
                    EditorGUILayout.PropertyField(_a4Prop);
                    break;
                case (int)FisheyeCameraSensor.CameraModel.Equidistant:
                    break;
            }
            EditorGUILayout.PropertyField(focalLengthProp);
            EditorGUILayout.PropertyField(principalPointProp);
            serializedObject.ApplyModifiedProperties();
        }
    }
}