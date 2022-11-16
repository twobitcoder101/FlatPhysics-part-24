using System;
using System.Diagnostics.CodeAnalysis;
using System.Collections.Generic;
using FlatPhysics;

namespace Flat
{
    public static class FlatPolygon
    {
        public static float Area(FlatVector[] vertices)
        {
            float area = 0f;

            for (int i = 0; i < vertices.Length; i++)
            {
                FlatVector a = vertices[i];
                FlatVector b = vertices[(i + 1) % vertices.Length];

                float width = b.X - a.X;
                float height = (b.Y + b.X) * 0.5f;

                area += width * height;
            }

            return area;
        }

        public static bool PointInTriangle(FlatVector p, FlatVector a, FlatVector b, FlatVector c)
        {
            FlatVector ab = b - a;
            FlatVector bc = c - b;
            FlatVector ca = a - c;

            FlatVector ap = p - a;
            FlatVector bp = p - b;
            FlatVector cp = p - c;

            float c1 = FlatMath.Cross(ap, ab);
            float c2 = FlatMath.Cross(bp, bc);
            float c3 = FlatMath.Cross(cp, ca);

            if (c1 <= 0f || c2 <= 0f || c3 <= 0f)
            {
                return false;
            }

            return true;
        }

        private static bool AnyVerticesInTriangle(FlatVector[] vertices, FlatVector a, FlatVector b, FlatVector c)
        {
            for (int j = 0; j < vertices.Length; j++)
            {
                FlatVector p = vertices[j];

                if (FlatPolygon.PointInTriangle(p, a, b, c))
                {
                    return true;
                }
            }

            return false;
        }

        private static T GetItem<T>(List<T> list, int index)
        {
            int count = list.Count;

            if(index >= count)
            {
                return list[index % count];
            }
            else if(index < 0)
            {
                return list[(index % count) + count];
            }

            return list[index];
        }

        public static bool Triangulate(FlatVector[] vertices, [NotNullWhen(true)] out int[]? triangleIndices, out string errorMessage)
        {
            triangleIndices = null;
            errorMessage = string.Empty;

            if (vertices is null)
            {
                errorMessage = "Vertices array is null.";
                return false;
            }

            if (vertices.Length < 3)
            {
                errorMessage = "Vertices array must contain at least 3 items.";
                return false;
            }

            int triangleCount = vertices.Length - 2;
            int triangleIndicesCount = triangleCount * 3;

            triangleIndices = new int[triangleIndicesCount];
            int indexCount = 0;

            List<int> indices = new List<int>(vertices.Length);
            for (int i = 0; i < vertices.Length; i++)
            {
                indices.Add(i);
            }

            while (indices.Count > 3)
            {
                for (int i = 0; i < indices.Count; i++)
                {
                    int a = FlatPolygon.GetItem(indices, i - 1);
                    int b = FlatPolygon.GetItem(indices, i);
                    int c = FlatPolygon.GetItem(indices, i + 1);

                    FlatVector va = vertices[a];
                    FlatVector vb = vertices[b];
                    FlatVector vc = vertices[c];

                    // Test for convexity. If not convex move to next angle.
                    if (FlatMath.Cross(va - vb, vc - vb) <= 0f)
                    {
                        continue;
                    }

                    // Test for any points "inside" this triangle.
                    if (FlatPolygon.AnyVerticesInTriangle(vertices, va, vb, vc))
                    {
                        continue;
                    }

                    triangleIndices[indexCount++] = a;
                    triangleIndices[indexCount++] = b;
                    triangleIndices[indexCount++] = c;

                    indices.RemoveAt(i);

                    break;
                }
            }

            triangleIndices[indexCount++] = indices[0];
            triangleIndices[indexCount++] = indices[1];
            triangleIndices[indexCount++] = indices[2];

            return true;
        }
    }
}