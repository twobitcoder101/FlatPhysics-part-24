using System;

namespace FlatPhysics
{
    public static class Collisions
    {
        public static void PointSegmentDistance(FlatVector p, FlatVector a, FlatVector b, out float distanceSquared, out FlatVector cp)
        {
            FlatVector ab = b - a;
            FlatVector ap = p - a;

            float proj = FlatMath.Dot(ap, ab);
            float abLenSq = FlatMath.LengthSquared(ab);
            float d = proj / abLenSq;

            if(d <= 0f)
            {
                cp = a;
            }
            else if(d >= 1f)
            {
                cp = b;
            }
            else
            {
                cp = a + ab * d;
            }

            distanceSquared = FlatMath.DistanceSquared(p, cp);
        }

        public static bool IntersectAABBs(FlatAABB a, FlatAABB b)
        {
            if(a.Max.X <= b.Min.X || b.Max.X <= a.Min.X ||
                a.Max.Y <= b.Min.Y || b.Max.Y <= a.Min.Y)
            {
                return false;
            }

            return true;
        }

        public static void FindContactPoints(
            FlatBody bodyA, FlatBody bodyB, 
            out FlatVector contact1, out FlatVector contact2, 
            out int contactCount)
        {
            contact1 = FlatVector.Zero;
            contact2 = FlatVector.Zero;
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(),
                        out contact1, out contact2, out contactCount);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclePolygonContactPoint(bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindCirclePolygonContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclesContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, out contact1);
                    contactCount = 1;
                }
            }
        }

        private static void FindPolygonsContactPoints(
            FlatVector[] verticesA, FlatVector[] verticesB, 
            out FlatVector contact1, out FlatVector contact2, out int contactCount)
        {
            contact1 = FlatVector.Zero;
            contact2 = FlatVector.Zero;
            contactCount = 0;

            float minDistSq = float.MaxValue;

            for(int i = 0; i < verticesA.Length; i++)
            {
                FlatVector p = verticesA[i];

                for(int j = 0; j < verticesB.Length; j++)
                {
                    FlatVector va = verticesB[j];
                    FlatVector vb = verticesB[(j + 1) % verticesB.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if(FlatMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!FlatMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if(distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector p = verticesB[i];

                for (int j = 0; j < verticesA.Length; j++)
                {
                    FlatVector va = verticesA[j];
                    FlatVector vb = verticesA[(j + 1) % verticesA.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if (FlatMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!FlatMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }
        }

        private static void FindCirclePolygonContactPoint(
            FlatVector circleCenter, float circleRadius, 
            FlatVector polygonCenter, FlatVector[] polygonVertices, 
            out FlatVector cp)
        {
            cp = FlatVector.Zero;

            float minDistSq = float.MaxValue;

            for(int i = 0; i < polygonVertices.Length; i++)
            {
                FlatVector va = polygonVertices[i];
                FlatVector vb = polygonVertices[(i + 1) % polygonVertices.Length];

                Collisions.PointSegmentDistance(circleCenter, va, vb, out float distSq, out FlatVector contact);

                if(distSq < minDistSq)
                {
                    minDistSq = distSq;
                    cp = contact;
                }
            }
        }

        private static void FindCirclesContactPoint(FlatVector centerA, float radiusA, FlatVector centerB, out FlatVector cp)
        {
            FlatVector ab = centerB - centerA;
            FlatVector dir = FlatMath.Normalize(ab);
            cp = centerA + dir * radiusA;
        }

        public static bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectPolygons(
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    bool result = Collisions.IntersectCirclePolygon(
                        bodyB.Position, bodyB.Radius,
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        out normal, out depth);

                    normal = -normal;
                    return result;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectCirclePolygon(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    return Collisions.IntersectCircles(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.Radius,
                        out normal, out depth);
                }
            }

            return false;
        }

        public static bool IntersectCirclePolygon(FlatVector circleCenter, float circleRadius,
                                                    FlatVector polygonCenter, FlatVector[] vertices,
                                                    out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;

            FlatVector axis = FlatVector.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < vertices.Length; i++)
            {
                FlatVector va = vertices[i];
                FlatVector vb = vertices[(i + 1) % vertices.Length];

                FlatVector edge = vb - va;
                axis = new FlatVector(-edge.Y, edge.X);
                axis = FlatMath.Normalize(axis);

                Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
                Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            int cpIndex = Collisions.FindClosestPointOnPolygon(circleCenter, vertices);
            FlatVector cp = vertices[cpIndex];

            axis = cp - circleCenter;
            axis = FlatMath.Normalize(axis);

            Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
            Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

            if (minA >= maxB || minB >= maxA)
            {
                return false;
            }

            axisDepth = MathF.Min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }

            FlatVector direction = polygonCenter - circleCenter;

            if (FlatMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }

        private static int FindClosestPointOnPolygon(FlatVector circleCenter, FlatVector[] vertices)
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for(int i = 0; i < vertices.Length; i++)
            {
                FlatVector v = vertices[i];
                float distance = FlatMath.Distance(v, circleCenter);

                if(distance < minDistance)
                {
                    minDistance = distance;
                    result = i;
                }
            }

            return result;
        }

        private static void ProjectCircle(FlatVector center, float radius, FlatVector axis, out float min, out float max)
        {
            FlatVector direction = FlatMath.Normalize(axis);
            FlatVector directionAndRadius = direction * radius;

            FlatVector p1 = center + directionAndRadius;
            FlatVector p2 = center - directionAndRadius;

            min = FlatMath.Dot(p1, axis);
            max = FlatMath.Dot(p2, axis);

            if(min > max)
            {
                // swap the min and max values.
                float t = min;
                min = max;
                max = t;
            }
        }

        public static bool IntersectPolygons(FlatVector centerA, FlatVector[] verticesA, FlatVector centerB, FlatVector[] verticesB, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++)
            {
                FlatVector va = verticesA[i];
                FlatVector vb = verticesA[(i + 1) % verticesA.Length];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X);
                axis = FlatMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector va = verticesB[i];
                FlatVector vb = verticesB[(i + 1) % verticesB.Length];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X);
                axis = FlatMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            FlatVector direction = centerB - centerA;

            if (FlatMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }

        private static void ProjectVertices(FlatVector[] vertices, FlatVector axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            for(int i = 0; i < vertices.Length; i++)
            {
                FlatVector v = vertices[i];
                float proj = FlatMath.Dot(v, axis);

                if(proj < min) { min = proj; }
                if(proj > max) { max = proj; }
            }
        }

        public static bool IntersectCircles(
            FlatVector centerA, float radiusA, 
            FlatVector centerB, float radiusB, 
            out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = 0f;

            float distance = FlatMath.Distance(centerA, centerB);
            float radii = radiusA + radiusB;

            if(distance >= radii)
            {
                return false;
            }

            normal = FlatMath.Normalize(centerB - centerA);
            depth = radii - distance;

            return true;
        }

    }
}
