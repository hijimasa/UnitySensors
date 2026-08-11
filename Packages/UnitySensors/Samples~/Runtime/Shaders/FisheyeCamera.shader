Shader "UnitySensors/FisheyeCamera"
{

    Properties
    {
        [NoScaleOffset] _MainTex ("Cubemap", Cube) = "" { }
        _Angle ("Angle", Range(90, 360)) = 180
        _CameraModel ("Camera Model", Range(0, 5)) = 0
        _alpha ("Alpha", Range(0, 1)) = 1
        _beta ("Beta", Float) = 1
        _xi ("Xi", Float) = 0.34
        _kb4 ("KB4 Coefficients", Vector) = (-0.01, 0.03, -0.02, 0.005)
        _affineCoeffs ("OCAM Affine Coefficients (c, d, e, 1)", Vector) = (1, 0, 0, 1)
        _a0 ("OCAM Unprojection Coefficients a0", float) = 190.87
        _a1 ("OCAM Unprojection Coefficients a1", float) = 0
        _a2 ("OCAM Unprojection Coefficients a2", float) = 0
        _a3 ("OCAM Unprojection Coefficients a3", float) = -0.000003
        _a4 ("OCAM Unprojection Coefficients a4", float) = 0
        _fx ("Normalized Focal Length X", Float) = 1
        _fy ("Normalized Focal Length Y", Float) = 1
        _cx ("Normalized Principal Point X", Float) = 0.5
        _cy ("Normalized Principal Point Y", Float) = 0.5
        _resolutionX ("Resolution X", Float) = 1024
        _resolutionY ("Resolution Y", Float) = 1024
    }

    Subshader
    {
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            

            samplerCUBE _MainTex;
            float _Angle;
            float4x4 _WorldTransform;
            float _alpha;
            float _beta;
            float _xi;
            float4 _kb4;
            float4 _affineCoeffs;
            float _a0;
            float _a1;
            float _a2;
            float _a3;
            float _a4;
            float _fx;
            float _fy;
            float _cx;
            float _cy;
            float _resolutionX;
            float _resolutionY;
            int _CameraModel;

            struct v2f
            {
                float4 pos : POSITION;
                float2 uv : TEXCOORD0;
            };

            v2f vert(appdata_img v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.texcoord.xy;
                return o;
            }
            // Reference: https://github.com/eowjd0512/fisheye-calib-adapter
            float3 UCMToDirection(float2 uv, float targetAngleRad)
            {
                float gamma = 1 - _alpha;
                float2 uv2 = (uv - float2(_cx, _cy)) / float2(_fx, _fy) * gamma;
                float r2 = dot(uv2, uv2);
                float xi = _alpha / gamma;
                float3 dir = (xi + sqrt(1 + (1 - xi * xi) * r2)) / (1 + r2) * float3(uv2, 1) - float3(0, 0, xi);
                dir = normalize(dir);
                return dir;
            }
            // Reference: https://github.com/eowjd0512/fisheye-calib-adapter
            float3 EUCMToDirection(float2 uv)
            {
                float2 uv2 = (uv - float2(_cx, _cy)) / float2(_fx, _fy);
                float r2 = dot(uv2, uv2);
                float gamma = 1 - _alpha;
                float z = (1 - _alpha * _alpha * _beta * r2) / (_alpha * sqrt(1 - (_alpha - gamma) * _beta * r2) + gamma);
                float3 dir = float3(uv2, z);
                dir = normalize(dir);
                return dir;
            }
            // Reference: https://github.com/eowjd0512/fisheye-calib-adapter
            float3 DSToDirection(float2 uv)
            {
                float2 uv2 = (uv - float2(_cx, _cy)) / float2(_fx, _fy);
                float r2 = dot(uv2, uv2);
                float mz = (1 - _alpha * _alpha *r2) / (_alpha * sqrt(1 - (2 * _alpha - 1) * r2) + 1 - _alpha);
                float factor = (mz * _xi + sqrt(mz * mz + (1 - _xi * _xi) *r2)) / (mz * mz + r2);
                float3 dir = factor * float3(uv2, mz) - float3(0, 0, _xi);
                dir = normalize(dir);
                return dir;
            }
            // Reference: https://github.com/eowjd0512/fisheye-calib-adapter
            float3 KB4ToDirection(float2 uv)
            {
                float2 uv2 = (uv - float2(_cx, _cy)) / float2(_fx, _fy);
                float r = length(uv2);
                float theta = r;
                for (int i = 0; i < 12; i++)
                {
                    float t2 = theta * theta;
                    float t4 = t2 * t2;
                    float t6 = t4 * t2;
                    float t8 = t4 * t4;
                    float f = theta * (1 + _kb4.x * t2 + _kb4.y * t4 + _kb4.z * t6 + _kb4.w * t8) - r;
                    float df = 1 + 3 * _kb4.x * t2 + 5 * _kb4.y * t4 + 7 * _kb4.z * t6 + 9 * _kb4.w * t8;
                    theta = theta - f / max(df, 1e-8);
                }
                float sin_theta = sin(theta);
                float cos_theta = cos(theta);
                float scale = sin_theta / max(r, 1e-8);
                float x = uv2.x * scale;
                float y = uv2.y * scale;
                float z = cos_theta;
                if (theta < 1e-8) // Handle the case when theta is very small to avoid numerical instability
                {
                    x = 0;
                    y = 0;
                    z = 1;
                }
                float3 dir = float3(x, y, z);
                dir = normalize(dir);
                return dir;
            }
            // Reference: https://github.com/eowjd0512/fisheye-calib-adapter
            float3 OCAMToDirection(float2 uv)
            {
                float2 uv2 = uv - float2(_cx, _cy);
                uv2 = uv2 * float2(_resolutionX, _resolutionY);
                float2 xy = float2(uv2.x - uv2.y * _affineCoeffs.y, - uv2.x * _affineCoeffs.z + uv2.y * _affineCoeffs.x) 
                / (_affineCoeffs.x - _affineCoeffs.y * _affineCoeffs.z);
                float r = length(xy);
                float r2 = r * r;
                float r3 = r2 * r;
                float r4 = r2 * r2;
                float mz = _a0 + _a1 * r + _a2 * r2 + _a3 * r3 + _a4 * r4;
                float3 dir = float3(xy, mz);
                dir = normalize(dir);
                return dir;
            }

            // Reference: https://github.com/prefrontalcortex/DomeTools
            float3 EquidistantToDirection(float2 uv, float targetAngleRad)
            {
                float2 uv2 = uv * 2 - 1;
                float phi = atan2(uv2.y, uv2.x);
                float radius = length(uv2);
                float theta = radius * targetAngleRad * 0.5;
                float3 dir = float3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
                dir = normalize(dir);
                return dir;
            }

            fixed4 frag(v2f i) : COLOR
            {
                float3 dir = float3(0, 0, 1);
               switch (_CameraModel)
                {
                    case 0: // UCM
                        dir = UCMToDirection(i.uv, radians(_Angle));
                        break;
                    case 1: // EUCM
                        dir = EUCMToDirection(i.uv);
                        break;
                    case 2: // DS
                        dir = DSToDirection(i.uv);
                        break;
                    case 3: // KB4
                        dir = KB4ToDirection(i.uv);
                        break;
                    case 4: // OCAM
                        dir = OCAMToDirection(i.uv);
                        break;
                    case 5: // Equidistant
                        dir = EquidistantToDirection(i.uv, radians(_Angle));
                        break;
                }
                // Apply fisheye mask based on the angle
                float angle = acos(dir.z);
                float3 fisheyeMask = angle <= radians(_Angle) * 0.5;

                float3 worldDir = mul((float3x3)_WorldTransform, dir);
                float3 color = texCUBE(_MainTex, worldDir);
                color *= fisheyeMask;
                return float4(color, 1);
            }

            ENDCG
        }
    }
    Fallback Off
}
