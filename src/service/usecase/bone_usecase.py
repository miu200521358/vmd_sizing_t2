import os

from mlib.core.logger import MLogger
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_reader import PmxReader

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BoneUsecase:
    def load_motion(self, motion_path: str, cache_motions: dict[str, VmdMotion]) -> tuple[str, VmdMotion]:
        reader = VmdReader()
        digest = reader.read_hash_by_filepath(motion_path)
        original_motion = cache_motions.get(digest)

        if not original_motion:
            original_motion = reader.read_by_filepath(motion_path)

        return digest, original_motion

    def load_model(self, model_path: str, cache_models: dict[str, PmxModel]) -> tuple[str, PmxModel]:
        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)

        return digest, original_model
