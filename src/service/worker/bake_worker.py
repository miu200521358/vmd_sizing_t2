import os
from concurrent.futures import ThreadPoolExecutor

import wx

from mlib.core.logger import MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_frame import BaseFrame
from mlib.utils.file_utils import get_root_dir
from service.usecase.bake_usecase import BakeUsecase
from service.usecase.io_usecase import IoUsecase

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BakeWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)
        self.max_worker = (
            1
            if frame.is_saving
            else max(1, int(min(32, (os.cpu_count() or 0) + 4) / 2))
        )

    def thread_execute(self):
        # まずは読み込み
        self.load()

        # IK焼き込み
        self.bake_ik()

        # 保存
        self.save()

        self.result_data = []

    def bake_ik(self):
        """IK焼き込み"""

        logger.info("IK焼き込み", decoration=MLogger.Decoration.BOX)

        usecase = BakeUsecase()
        bake_panel = self.frame.bake_panel

        bake_panel.output_motion_ctrl.data = usecase.bake_ik(
            bake_panel.model_ctrl.data,
            bake_panel.output_motion_ctrl.data,
            self.max_worker,
        )

    def save(self):
        """結果保存"""
        usecase = IoUsecase()
        bake_panel = self.frame.bake_panel

        usecase.save(
            0,
            bake_panel.model_ctrl.data,
            bake_panel.output_motion_ctrl.data,
            bake_panel.output_motion_ctrl.path,
        )

    def load(self):
        """データ読み込み"""
        usecase = IoUsecase()
        bake_panel = self.frame.bake_panel

        with ThreadPoolExecutor(
            thread_name_prefix="load", max_workers=self.max_worker
        ) as executor:
            motion_future = executor.submit(
                usecase.load_motion,
                0,
                bake_panel.motion_ctrl.path,
                self.frame.cache_motions,
            )
            model_future = executor.submit(
                usecase.load_model_no_copy,
                0,
                bake_panel.model_ctrl.path,
                self.frame.cache_models,
            )

        sizing_idx, digest, original_motion, motion = motion_future.result()
        bake_panel.motion_ctrl.data = original_motion
        bake_panel.output_motion_ctrl.data = motion

        sizing_idx, digest, model = model_future.result()
        bake_panel.model_ctrl.data = model

    def output_log(self):
        bake_panel = self.frame.bake_panel
        output_log_path = os.path.join(
            get_root_dir(),
            f"{os.path.basename(bake_panel.output_motion_ctrl.path)}.log",
        )
        # 出力されたメッセージを全部出力
        bake_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
