import os
from typing import Optional

import wx

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.shader import MShader
from mlib.service.form.base_panel import BasePanel
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.vmd.vmd_collection import VmdMotion
from service.form.panel.bake_panel import BakePanel
from service.form.panel.bone_panel import BonePanel
from service.form.widgets.morph_sub_window import MorphSubCanvasWindow

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class MainFrame(NotebookFrame):
    def __init__(self, app: wx.App, title: str, size: wx.Size, is_saving: bool, *args, **kw) -> None:
        super().__init__(
            app,
            history_keys=["vmd", "org_pmx", "rep_pmx", "camera_vmd", "camera_pmx", "ik_pmx"],
            title=title,
            size=size,
            is_saving=is_saving,
        )
        self.selected_tab_idx = 0
        # タブ追加時に on_change_tab が動かないように一旦True
        self.running_worker = True

        # ボーンモーションサイジング
        self.bone_panel = BonePanel(self, 0)
        self.notebook.AddPage(self.bone_panel, __("ボーン"), True)

        # IK焼き込み
        self.bake_panel = BakePanel(self, 1)
        self.notebook.AddPage(self.bake_panel, __("IK焼き込み"), True)

        self.cache_models: dict[str, PmxModel] = {}
        self.cache_motions: dict[str, VmdMotion] = {}

        self.morph_sub_window_size = wx.Size(300, 400)
        self.morph_sub_window: Optional[MorphSubCanvasWindow] = None

        # 初期化が終わったらFalseに戻す
        self.running_worker = False

    def show_morph_sub_window(self, event: wx.Event, panel: BasePanel) -> None:
        self.create_morph_sub_window(panel)

        if self.morph_sub_window:
            if not self.morph_sub_window.IsShown():
                model: Optional[PmxModel] = panel.model_ctrl.data
                if model:
                    self.morph_sub_window.panel.canvas.clear_model_set()
                    self.morph_sub_window.panel.canvas.append_model_set(model, VmdMotion(), bone_alpha=0.0, is_sub=True)
                    self.morph_sub_window.panel.canvas.vertical_degrees = 5
                    self.morph_sub_window.panel.canvas.look_at_center = model.bones["頭"].position.copy()
                    self.morph_sub_window.panel.canvas.Refresh()
                    self.morph_sub_window.panel.canvas.camera_offset_position.y = (
                        model.bones["頭"].position.y - MShader.INITIAL_CAMERA_POSITION_Y
                    )

                frame_x, frame_y = self.GetPosition()
                self.morph_sub_window.SetPosition(wx.Point(max(0, frame_x + self.GetSize().GetWidth() + 10), max(0, frame_y + 30)))

                self.morph_sub_window.Show()
            elif self.morph_sub_window.IsShown():
                self.morph_sub_window.Hide()
        event.Skip()

    def create_morph_sub_window(self, panel: BasePanel) -> None:
        model: Optional[PmxModel] = panel.model_ctrl.data
        if not self.morph_sub_window and model:
            self.morph_sub_window = MorphSubCanvasWindow(
                self, __("モーフプレビュー"), self.morph_sub_window_size, [model.name], [model.bones.names], model.morphs.names
            )

    def on_change_tab(self, event: wx.Event) -> None:
        if self.running_worker:
            # 処理が動いている場合、動かさない
            self.notebook.ChangeSelection(self.selected_tab_idx)
            event.Skip()
            return

        # 処理が終わっている場合、動かしてOK
        self.selected_tab_idx = self.notebook.GetSelection()

    def Enable(self, enable: bool):
        self.bone_panel.Enable(enable)

    def is_align(self):
        """セットのどれかに位置合わせが入っているか"""
        for sizing_set in self.bone_panel.sizing_sets:
            if sizing_set.align_check_ctrl.GetValue():
                return True
        return False
