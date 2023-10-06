from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import os

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_tree import VmdBoneFrameTrees

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BakeUsecase:
    def bake_ik(
        self,
        model: PmxModel,
        motion: VmdMotion,
        max_worker: int,
    ) -> VmdMotion:
        """IK焼き込み"""

        with ThreadPoolExecutor(thread_name_prefix="bake", max_workers=max_worker) as executor:
            futures: list[Future] = []

            for bone in model.bones:
                if bone.is_ik and bone.ik.bone_index in model.bones:
                    futures.append(executor.submit(self.bake_ik_bone, model, motion, bone))

        output_motion = VmdMotion()

        for future in as_completed(futures):
            if future.exception():
                raise future.exception()
            ik_bone_index, fnos, matrixes = future.result()
            bake_matrixes: VmdBoneFrameTrees = matrixes

            ik_target_bone = model.bones[model.bones[ik_bone_index].ik.bone_index]
            for fno in fnos:
                bf = output_motion.bones[ik_target_bone.name][fno]
                bf.rotation = bake_matrixes[fno, ik_target_bone.name].frame_rotation
                bf.register = True
                output_motion.insert_bone_frame(bf)

            for link in model.bones[ik_bone_index].ik.links:
                if link.bone_index not in model.bones:
                    continue
                bone_name = model.bones[link.bone_index].name
                for fno in fnos:
                    bf = output_motion.bones[bone_name][fno]
                    bf.rotation = bake_matrixes[fno, bone_name].frame_rotation
                    bf.register = True
                    output_motion.append_bone_frame(bf)

        return output_motion

    def bake_ik_bone(
        self,
        model: PmxModel,
        motion: VmdMotion,
        bone: Bone,
    ) -> tuple[Bone, list[int], VmdBoneFrameTrees]:
        logger.info("IK焼き込み: {b}", b=bone.name, decoration=MLogger.Decoration.LINE)

        bone_names: list[str] = [model.bones[bone.ik.bone_index].name]

        fnos = set(motion.bones[bone.name].register_indexes)
        fnos |= set(motion.bones[model.bones[bone.ik.bone_index].name].register_indexes)
        for link in bone.ik.links:
            if link.bone_index not in model.bones:
                continue
            bone_name = model.bones[link.bone_index].name
            fnos = fnos | set(motion.bones[bone_name].register_indexes)
            bone_names.append(bone_name)

        sorted_fnos = sorted(fnos)

        return bone.index, sorted_fnos, motion.animate_bone(sorted_fnos, model, bone_names, out_fno_log=True)
