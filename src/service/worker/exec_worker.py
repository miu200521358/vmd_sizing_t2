import os

import wx

from mlib.core.logger import MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_frame import BaseFrame
from mlib.utils.file_utils import get_root_dir
from service.usecase.exec_usecase import ExecUsecase

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class ExecWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)

    def thread_execute(self):
        file_panel = self.frame.file_panel
        usecase = ExecUsecase()

        self.result_data = []

        logger.info("お着替えモデル読み込み完了", decoration=MLogger.Decoration.BOX)

    def output_log(self):
        file_panel = self.frame.file_panel
        output_log_path = os.path.join(get_root_dir(), f"{os.path.basename(file_panel.output_pmx_ctrl.path)}.log")
        # 出力されたメッセージを全部出力
        file_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
