import os

import wx
from service.usecase.bake_usecase import BakeUsecase
from service.usecase.io_usecase import IoUsecase

from mlib.core.logger import MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_panel import BasePanel
from mlib.utils.file_utils import get_root_dir

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BakeWorker(BaseWorker):
    def __init__(self, panel: BasePanel, result_event: wx.Event) -> None:
        super().__init__(panel, result_event)

    def thread_execute(self):
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
            bake_panel.selected_bone_names,
            bake_panel.bake_interval_choice_ctrl.GetSelection(),
            bake_panel.bake_grain_slider.GetValue(),
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

    def output_log(self):
        bake_panel = self.frame.bake_panel
        output_log_path = os.path.join(
            get_root_dir(),
            f"{os.path.basename(bake_panel.output_motion_ctrl.path)}.log",
        )
        # 出力されたメッセージを全部出力
        bake_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
