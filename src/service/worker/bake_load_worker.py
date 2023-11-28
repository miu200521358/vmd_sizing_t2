import os
from concurrent.futures import ThreadPoolExecutor

import wx
from service.usecase.io_usecase import IoUsecase

from mlib.core.logger import MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_panel import BasePanel
from mlib.utils.file_utils import get_root_dir

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BakeLoadWorker(BaseWorker):
    def __init__(self, panel: BasePanel, result_event: wx.Event) -> None:
        super().__init__(panel, result_event)

    def thread_execute(self):
        # まずは読み込み
        self.load()

        self.result_data = []

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
