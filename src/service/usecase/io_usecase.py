import os

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_reader import PmxReader
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.vmd.vmd_writer import VmdWriter

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class IoUsecase:
    def load_motion(self, sizing_idx: int, motion_path: str, cache_motions: dict[str, VmdMotion]) -> tuple[int, str, VmdMotion, VmdMotion]:
        """モーションの読み込み"""
        reader = VmdReader()
        digest = reader.read_hash_by_filepath(motion_path)
        original_motion = cache_motions.get(digest)

        if not original_motion:
            original_motion = reader.read_by_filepath(motion_path)

        return sizing_idx, digest, original_motion, original_motion.copy()

    def load_model(self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel]) -> tuple[int, str, PmxModel, PmxModel]:
        """モデルの読み込み"""
        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)

        model = original_model.copy()

        return sizing_idx, digest, original_model, model

    def load_model_no_copy(self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel]) -> tuple[int, str, PmxModel]:
        """モデルの読み込み"""
        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)

        return sizing_idx, digest, original_model

    def save(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        motion: VmdMotion,
        output_path: str,
    ) -> None:
        """結果保存"""
        logger.info("【No.{i}】結果保存", i=sizing_idx + 1, decoration=MLogger.Decoration.LINE)

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        VmdWriter(motion, output_path, dest_model.name).save()

        logger.info("【No.{i}】結果保存成功\n{p}", i=sizing_idx + 1, p=output_path, decoration=MLogger.Decoration.BOX)
