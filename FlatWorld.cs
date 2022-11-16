using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Diagnostics.Contracts;
using System.Drawing;

namespace FlatPhysics
{
    public sealed class FlatWorld
    {
        public static int TransformCount = 0;
        public static int NoTransformCount = 0;

        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;

        public static readonly float MinDensity = 0.5f;     // g/cm^3
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        private FlatVector gravity;
        private List<FlatBody> bodyList;
        private List<(int, int)> contactPairs;

        private FlatVector[] contactList;
        private FlatVector[] impulseList;
        private FlatVector[] raList;
        private FlatVector[] rbList;
        private FlatVector[] frictionImpulseList;
        private float[] jList;

        public int BodyCount
        {
            get { return this.bodyList.Count; }
        }

        public FlatWorld()
        {
            this.gravity = new FlatVector(0f, -9.81f);
            this.bodyList = new List<FlatBody>();
            this.contactPairs = new List<(int, int)>();

            this.contactList = new FlatVector[2];
            this.impulseList = new FlatVector[2];
            this.raList = new FlatVector[2];
            this.rbList = new FlatVector[2];
            this.frictionImpulseList = new FlatVector[2];
            this.jList = new float[2];
        }

        public void AddBody(FlatBody body)
        {
            this.bodyList.Add(body);
        }

        public bool RemoveBody(FlatBody body)
        {
            return this.bodyList.Remove(body);
        }

        public bool GetBody(int index, out FlatBody body)
        {
            body = null;

            if(index < 0 || index >= this.bodyList.Count)
            {
                return false;
            }

            body = this.bodyList[index];
            return true;
        }

        public void Step(float time, int totalIterations)
        {
            totalIterations = FlatMath.Clamp(totalIterations, FlatWorld.MinIterations, FlatWorld.MaxIterations);

            for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++)
            {
                this.contactPairs.Clear();
                this.StepBodies(time, totalIterations);
                this.BroadPhase();
                this.NarrowPhase();
            }
        }

        private void BroadPhase()
        {
            for (int i = 0; i < this.bodyList.Count - 1; i++)
            {
                FlatBody bodyA = this.bodyList[i];
                FlatAABB bodyA_aabb = bodyA.GetAABB();

                for (int j = i + 1; j < this.bodyList.Count; j++)
                {
                    FlatBody bodyB = this.bodyList[j];
                    FlatAABB bodyB_aabb = bodyB.GetAABB();

                    if (bodyA.IsStatic && bodyB.IsStatic)
                    {
                        continue;
                    }

                    if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                    {
                        continue;
                    }

                    this.contactPairs.Add((i, j));
                }
            }
        }

        private void NarrowPhase()
        {
            for (int i = 0; i < this.contactPairs.Count; i++)
            {
                (int, int) pair = this.contactPairs[i];
                FlatBody bodyA = this.bodyList[pair.Item1];
                FlatBody bodyB = this.bodyList[pair.Item2];

                if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
                {
                    this.SeparateBodies(bodyA, bodyB, normal * depth);
                    Collisions.FindContactPoints(bodyA, bodyB, out FlatVector contact1, out FlatVector contact2, out int contactCount);
                    FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                    this.ResolveCollisionWithRotationAndFriction(in contact);
                }

            }
        }

        public void StepBodies(float time, int totalIterations)
        {
            for (int i = 0; i < this.bodyList.Count; i++)
            {
                this.bodyList[i].Step(time, this.gravity, totalIterations);
            }
        }

        private void SeparateBodies(FlatBody bodyA, FlatBody bodyB, FlatVector mtv)
        {
            if (bodyA.IsStatic)
            {
                bodyB.Move(mtv);
            }
            else if (bodyB.IsStatic)
            {
                bodyA.Move(-mtv);
            }
            else
            {
                bodyA.Move(-mtv / 2f);
                bodyB.Move(mtv / 2f);
            }
        }

        public void ResolveCollisionBasic(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            float depth = contact.Depth;

            FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (FlatMath.Dot(relativeVelocity, normal) > 0f)
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            FlatVector impulse = j * normal;

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }

        public void ResolveCollisionWithRotation(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            FlatVector contact1 = contact.Contact1;
            FlatVector contact2 = contact.Contact2;
            int contactCount = contact.ContactCount;

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for(int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = FlatVector.Zero;
                this.raList[i] = FlatVector.Zero;
                this.rbList[i] = FlatVector.Zero;
            }

            for(int i = 0; i < contactCount; i++)
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity = 
                    (bodyB.LinearVelocity + angularLinearVelocityB) - 
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = FlatMath.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = FlatMath.Dot(raPerp, normal);
                float rbPerpDotN = FlatMath.Dot(rbPerp, normal);

                float denom = bodyA.InvMass + bodyB.InvMass + 
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia + 
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contactCount;

                FlatVector impulse = j * normal;
                impulseList[i] = impulse;
            }

            for(int i = 0; i < contactCount; i++)
            {
                FlatVector impulse = impulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, impulse) * bodyB.InvInertia;
            }
        }

        public void ResolveCollisionWithRotationAndFriction(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            FlatVector contact1 = contact.Contact1;
            FlatVector contact2 = contact.Contact2;
            int contactCount = contact.ContactCount;

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float sf = (bodyA.StaticFriction + bodyB.StaticFriction) * 0.5f;
            float df = (bodyA.DynamicFriction + bodyB.DynamicFriction) * 0.5f;

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = FlatVector.Zero;
                this.raList[i] = FlatVector.Zero;
                this.rbList[i] = FlatVector.Zero;
                this.frictionImpulseList[i] = FlatVector.Zero;
                this.jList[i] = 0f;
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = FlatMath.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = FlatMath.Dot(raPerp, normal);
                float rbPerpDotN = FlatMath.Dot(rbPerp, normal);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia +
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contactCount;

                jList[i] = j;

                FlatVector impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector impulse = impulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, impulse) * bodyB.InvInertia;
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector ra = contactList[i] - bodyA.Position;
                FlatVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                FlatVector raPerp = new FlatVector(-ra.Y, ra.X);
                FlatVector rbPerp = new FlatVector(-rb.Y, rb.X);

                FlatVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                FlatVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                FlatVector relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                FlatVector tangent = relativeVelocity - FlatMath.Dot(relativeVelocity, normal) * normal;

                if(FlatMath.NearlyEqual(tangent, FlatVector.Zero))
                {
                    continue;
                }
                else
                {
                    tangent = FlatMath.Normalize(tangent);
                }

                float raPerpDotT = FlatMath.Dot(raPerp, tangent);
                float rbPerpDotT = FlatMath.Dot(rbPerp, tangent);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotT * raPerpDotT) * bodyA.InvInertia +
                    (rbPerpDotT * rbPerpDotT) * bodyB.InvInertia;

                float jt = -FlatMath.Dot(relativeVelocity, tangent);
                jt /= denom;
                jt /= (float)contactCount;

                FlatVector frictionImpulse;
                float j = jList[i];

                if(MathF.Abs(jt) <= j * sf)
                {
                    frictionImpulse = jt * tangent;
                }
                else
                {
                    frictionImpulse = -j * tangent * df;
                }

                this.frictionImpulseList[i] = frictionImpulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                FlatVector frictionImpulse = this.frictionImpulseList[i];
                FlatVector ra = raList[i];
                FlatVector rb = rbList[i];

                bodyA.LinearVelocity += -frictionImpulse * bodyA.InvMass;
                bodyA.AngularVelocity += -FlatMath.Cross(ra, frictionImpulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += frictionImpulse * bodyB.InvMass;
                bodyB.AngularVelocity += FlatMath.Cross(rb, frictionImpulse) * bodyB.InvInertia;
            }
        }
    }
}
