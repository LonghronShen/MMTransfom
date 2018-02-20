using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VecMath;
using MMDataIO.Pmx;

namespace MMTransform
{
    public class PmxIK
    {
        public PmxBone IKBone { get; }
        public PmxBone TargetBone { get; private set; }

        public int Depth { get; }
        public float LimitAngle { get; }

        public PmxIKChild[] Childs { get; private set; }

        public PmxIK(PmxBone bone, PmxBoneData data)
        {
            IKBone = bone;
            Depth = data.Depth;
            LimitAngle = data.AngleLimit;
        }

        public void Init(PmxBoneData data, PmxBones bones)
        {
            TargetBone = bones[data.IkTargetId];

            Childs = new PmxIKChild[data.IkChilds.Length];
            for (int i = 0; i < Childs.Length; i++)
            {
                var childData = data.IkChilds[i];
                Childs[i] = new PmxIKChild(bones[childData.ChildId], childData.AngleMin, childData.AngleMax);
            }
        }

        public void Update()
        {
            IKBone.UpdateWorldMatrix();

            for (int j = Childs.Length - 1; j >= 0; j--)
            {
                Childs[j].ChildBone.UpdateWorldMatrix();
            }
            TargetBone.UpdateWorldMatrix();

            for (int i = 0; i < 255; i++)
            {
                if (UpdateChildBones()) return;
            }
        }

        private Quaternion ClampEular(Quaternion rot, Vector3 min, Vector3 max)
        {
            Vector3 eular = RotationEularMatrixXZY(rot);
            eular = Vector3.Clamp(eular, min, max);
            return RotationEularXZY(eular);
        }

        public static Quaternion RotationEularXZY(Vector3 eular)
        {
            var x = Quaternion.RotationAxis(Vector3.UnitX, eular.x);
            var y = Quaternion.RotationAxis(Vector3.UnitY, eular.y);
            var z = Quaternion.RotationAxis(Vector3.UnitZ, eular.z);

            return x * z * y;
        }

        public static Vector3 RotationEularMatrixXZY(Matrix3 m)
        {
            Vector3 eular = Vector3.Zero;

            double sinZ = -m.m01;
            eular.z = (float)Math.Asin(sinZ);

            if (-1 < sinZ && sinZ < 1)
            {
                double cosZ = Math.Cos(eular.z);

                double sinY = m.m02 / cosZ;
                double cosY = m.m00 / cosZ;
                eular.y = (float)Math.Atan2(sinY, cosY);

                double sinX = m.m21 / cosZ;
                double cosX = m.m11 / cosZ;
                eular.x = (float)Math.Atan2(sinX, cosX);
            }
            else
            {
                double sinX = -m.m20;
                double cosX = -m.m10;
                eular.x = (float)Math.Atan2(sinX, cosX);
            }

            return eular;
        }

        private bool UpdateChildBones()
        {
            for (int i = 0; i < Childs.Length; i++)
            {
                var child = Childs[i];
                Matrix4 invMat = ~child.ChildBone.WorldMat;

                Vector3 ikPos = (Vector4)IKBone.WorldMat.Translation * invMat;
                Vector3 targetPos = (Vector4)TargetBone.WorldMat.Translation * invMat;

                if ((ikPos - targetPos).Length() < 0.01F)
                {
                    return true;
                }

                var scaling = new Matrix3(0, 1, 1);

                var ikVec = +(ikPos * scaling);
                var targetVec = +(targetPos * scaling);

                float angle = (float)Math.Acos(ikVec * targetVec);
                angle = MathUtil.Clamp(angle, -LimitAngle, LimitAngle);

                var axis = targetVec ^ ikVec;

                if (Math.Abs(angle) < 0.0001F || axis.Length() < 0.0001F)
                {
                    continue;
                }

                Quaternion rot = Quaternion.RotationAxis(axis, angle);

                if (child.Limit)
                {
                    rot = ClampEular(rot, child.AngleMin, child.AngleMax);
                }

                child.ChildBone.Rot = rot * child.ChildBone.Rot;
                child.ChildBone.UpdateLocalMatrix();

                for (int j = i; j >= 0; j--)
                {
                    Childs[j].ChildBone.UpdateWorldMatrix();
                }
                TargetBone.UpdateWorldMatrix();
            }
            return false;
        }
    }

    public struct PmxIKChild
    {
        public PmxBone ChildBone { get; }
        public Vector3 AngleMin { get; }
        public Vector3 AngleMax { get; }

        public bool Limit { get; }

        public PmxIKChild(PmxBone bone, Vector3 min, Vector3 max)
        {
            ChildBone = bone;
            Limit = min != Vector3.Zero || max != Vector3.Zero;
            AngleMin = min;
            AngleMax = max;
        }
    }
}
