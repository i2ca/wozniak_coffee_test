Shader "Custom/CenterColorEdgesTransparent"
{
    Properties
    {
        _CenterColor ("Center Color", Color) = (1,0,0,1) // A cor do centro, por exemplo, vermelho
        _EdgeWidth ("Edge Width", Float) = 0.05 // A largura das bordas
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" }
        Pass
        {
            ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float4 pos : POSITION;
                float2 uv : TEXCOORD0;
            };

            float4 _CenterColor;
            float _EdgeWidth;

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            half4 frag (v2f i) : COLOR
            {
                // Calculate distance from the edges of the mesh
                float2 uv = i.uv;
                float edgeDist = min(uv.x, min(1.0 - uv.x, min(uv.y, 1.0 - uv.y)));
                
                // Determine alpha based on the distance to the edge
                half edgeAlpha = smoothstep(_EdgeWidth, _EdgeWidth + 0.01, edgeDist);
                
                // Set center color
                half4 centerColor = _CenterColor;
                
                // Output color: center color for the middle, transparent for edges
                half4 color = centerColor;
                color.a = 1.0 - edgeAlpha; // Transparent at the edges

                return color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}
