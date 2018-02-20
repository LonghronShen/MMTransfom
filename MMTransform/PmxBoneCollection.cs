using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MMDataIO.Pmx;
using VecMath;

namespace MMTransform
{
    public class PmxBones : List<PmxBone>
    {
        public PmxBone this[string name] { get => Find(b => name == b.BoneName); }

        public void SetBoneData(PmxBoneData[] data)
        {
            for (int i = 0; i < data.Length; i++)
            {
                Add(CreateBone(data[i], i));
            }

            for (int i = 0; i < Count; i++)
            {
                this[i].Init(data[i], this);
            }
        }

        public void UpdateMatries()
        {
            foreach (var bone in from bone in this orderby bone.Depth, bone.BoneIndex select bone)
            {
                bone.UpdateLocalMatrix();
                bone.UpdateWorldMatrix();
            }
            ForEach(b => b.IK?.Update());
        }

        protected virtual PmxBone CreateBone(PmxBoneData data, int idx)
        {
            return new PmxBone(data) { BoneIndex = idx };
        }
    }
}

