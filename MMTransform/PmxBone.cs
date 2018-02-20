using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VecMath;
using MMDataIO.Pmx;

namespace MMTransform
{
    public class PmxBone
    {
        public string BoneName { get; }

        public int BoneIndex { get; set; }
        public int Depth { get; }

        public Matrix4 SkinningMat { get; private set; } = Matrix4.Identity;
        public Matrix4 InvInitMat { get; } = Matrix4.Identity;

        public Matrix4 WorldMat { get; private set; } = Matrix4.Identity;
        public Matrix4 LocalMat { get; private set; } = Matrix4.Identity;

        public Matrix4 LinkMat { get; private set; } = Matrix4.Identity;

        public Vector3 Pos { get; set; }
        public Quaternion Rot { get; set; } = Quaternion.Identity;

        public Vector3 OffsetPos { get; private set; }

        public PmxBone ParentBone { get; private set; }
        public PmxIK IK { get; }

        public BoneFlags Flag { get; }

        public PmxBone LinkParent { get; private set; }
        public float LinkWeight { get; }

        public PmxBone(PmxBoneData data)
        {
            BoneName = data.BoneName;
            Depth = data.Depth;
            Flag = data.Flag;
            LinkWeight = data.LinkWeight;

            OffsetPos = data.Pos;
            InvInitMat = new Matrix4(Matrix3.Identity, -OffsetPos);

            if (Flag.HasFlag(BoneFlags.IK))
            {
                IK = new PmxIK(this, data);
            }
        }

        public void Init(PmxBoneData data, PmxBones bones)
        {
            if (0 <= data.ParentId && data.ParentId < bones.Count)
            {
                ParentBone = bones[data.ParentId];
                OffsetPos = (Vector4)OffsetPos * ParentBone.InvInitMat;
            }

            if (Flag.HasFlag(BoneFlags.MOVE_LINK) | Flag.HasFlag(BoneFlags.ROTATE_LINK))
            {
                LinkParent = bones[data.LinkParentId];
            }

            if (Flag.HasFlag(BoneFlags.IK))
            {
                IK.Init(data, bones);
            }
        }

        public void UpdateLocalMatrix()
        {
            var localRot = Quaternion.Identity;
            var localPos = OffsetPos;

            if (Flag.HasFlag(BoneFlags.ROTATE_LINK))
            {
                localRot *= (Quaternion)(LinkMat = new Matrix4(UpdateLinkRot(), LinkMat.Translation));
            }

            if (Flag.HasFlag(BoneFlags.MOVE_LINK))
            {
                localPos += (LinkMat = new Matrix4(LinkMat.Rotation, UpdateLinkPos())).Translation;
            }

            localRot *= Rot;
            localPos += Pos;

            LocalMat = new Matrix4(localRot, localPos);
        }

        public void UpdateWorldMatrix()
        {
            if (ParentBone != null)
            {
                WorldMat = LocalMat * ParentBone.WorldMat;
            }
            else
            {
                WorldMat = LocalMat;
            }
            SkinningMat = InvInitMat * WorldMat;
        }

        private Quaternion UpdateLinkRot()
        {
            var linkRot = Quaternion.Identity;

            if (Flag.HasFlag(BoneFlags.LOCAL_LINK))
            {
                linkRot *= (Quaternion)LinkParent.LocalMat;
            }
            else
            {
                if (LinkParent.Flag.HasFlag(BoneFlags.ROTATE_LINK))
                {
                    linkRot *= (Quaternion)LinkParent.LinkMat;
                }
                else
                {
                    linkRot *= LinkParent.Rot;
                }
            }
            return linkRot ^ LinkWeight;
        }

        private Vector3 UpdateLinkPos()
        {
            var linkPos = Vector3.Zero;

            if (Flag.HasFlag(BoneFlags.LOCAL_LINK))
            {
                linkPos += LinkParent.LocalMat.Translation;
            }
            else
            {
                if (LinkParent.Flag.HasFlag(BoneFlags.MOVE_LINK))
                {
                    linkPos += LinkParent.LinkMat.Translation;
                }
                else
                {
                    linkPos += LinkParent.Pos;
                }
            }

            return linkPos ^ LinkWeight;
        }
    }
}
