import os

import wx

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.service.form.base_panel import BasePanel

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class TreeDialog(wx.Dialog):
    def __init__(self, parent: BasePanel, title: str, model: PmxModel):
        super(TreeDialog, self).__init__(parent, title=title, size=(450, 600))

        self.model = model
        self.panel = wx.Panel(self)
        self.sizer = wx.BoxSizer(wx.VERTICAL)

        self.description_ctrl = wx.StaticText(
            self.panel,
            wx.ID_ANY,
            __("焼き込みたいIK管理下のボーン(足・ひざ・足首等)を選択してください\n")
            + __("Shiftキーを押しながら選択すると、複数ボーンをまとめて選択できます\n")
            + __("Ctrlキーを押しながら選択すると、複数ボーンをそれぞれ選択できます\n")
            + __("（IKボーンも一緒に選択しても問題ありません）"),
        )
        self.sizer.Add(self.description_ctrl, 0, wx.ALL, 2)

        self.tree_ctrl = wx.TreeCtrl(
            self.panel, style=wx.TR_DEFAULT_STYLE | wx.TR_MULTIPLE
        )

        # ツリービューの設定
        root = self.tree_ctrl.AddRoot(model.name)
        for bone in model.bones:
            if bone.ik:
                item = self.tree_ctrl.AppendItem(root, bone.name)
                for ik_link in bone.ik.links:
                    self.tree_ctrl.AppendItem(
                        item, model.bones[ik_link.bone_index].name
                    )

        self.tree_ctrl.ExpandAll()
        self.sizer.Add(self.tree_ctrl, 1, wx.EXPAND | wx.ALL, 5)

        # ボタン ------------------
        self.btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.ok_btn_ctrl = wx.Button(self.panel, label="OK", id=wx.ID_OK)
        self.btn_sizer.Add(self.ok_btn_ctrl, 0, wx.ALL, 5)

        self.cancel_btn_ctrl = wx.Button(self.panel, label="Cancel", id=wx.ID_CANCEL)
        self.btn_sizer.Add(self.cancel_btn_ctrl, 0, wx.ALL, 5)

        self.sizer.Add(self.btn_sizer, 0, wx.ALIGN_RIGHT)
        self.panel.SetSizer(self.sizer)

    def get_selected_names(self) -> list[str]:
        return [
            self.tree_ctrl.GetItemText(idx)
            for idx in self.tree_ctrl.GetSelections()
            if self.tree_ctrl.GetItemText(idx) in self.model.bones
            and not self.model.bones[self.tree_ctrl.GetItemText(idx)].ik
        ]
