import os
from datetime import datetime

import wx
from service.form.widgets.tree_dialog import TreeDialog
from service.worker.bake_load_worker import BakeLoadWorker
from service.worker.bake_worker import BakeWorker

from mlib.core.logger import ConsoleHandler, MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.console_ctrl import ConsoleCtrl
from mlib.service.form.widgets.exec_btn_ctrl import ExecButton
from mlib.service.form.widgets.file_ctrl import MPmxFilePickerCtrl, MVmdFilePickerCtrl
from mlib.service.form.widgets.float_slider_ctrl import FloatSliderCtrl
from mlib.utils.file_utils import save_histories

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BakePanel(NotebookPanel):
    def __init__(self, frame: NotebookFrame, tab_idx: int, *args, **kw) -> None:
        super().__init__(frame, tab_idx, *args, **kw)

        self.load_worker = BakeLoadWorker(self, self.on_prepare_result)
        self.bake_worker = BakeWorker(self, self.on_exec_result)

        self._initialize_ui()

        self.Enable(False)
        self.EnableLoad(True)

    def _initialize_ui(self) -> None:
        # ヘッダー -----------------------------
        self.header_sizer = wx.BoxSizer(wx.VERTICAL)

        self.description_ctrl = wx.StaticText(
            self,
            wx.ID_ANY,
            __("任意のIKボーンの計算を行い、IK管理下のボーンの角度を算出したモーションデータを生成します。\n")
            + __("極力MMDに近づけて計算していますが、差異が出てくる場合があります。"),
        )
        self.header_sizer.Add(self.description_ctrl, 0, wx.ALL, 2)

        self.file_box_sizer = wx.StaticBoxSizer(
            wx.StaticBox(self, wx.ID_ANY, __("ファイル")), orient=wx.VERTICAL
        )

        self.motion_ctrl = MVmdFilePickerCtrl(
            self,
            self.frame,
            self,
            key="vmd",
            title="焼き込み対象モーション",
            is_show_name=True,
            name_spacer=1,
            is_save=False,
            tooltip="IK焼き込みの対象となるVMDモーションデータを指定してください",
            file_change_event=self.on_change_motion,
        )
        self.motion_ctrl.set_parent_sizer(self.file_box_sizer)

        self.model_ctrl = MPmxFilePickerCtrl(
            self,
            self.frame,
            self,
            key="ik_pmx",
            title="焼き込み元モデル",
            is_show_name=True,
            name_spacer=4,
            is_save=False,
            tooltip="モーションを適用させたいモデルを指定してください\nこのモデルに合わせてモーションのIK部分をIK焼き込みします",
            file_change_event=self.on_change_model_pmx,
        )
        self.model_ctrl.set_parent_sizer(self.file_box_sizer)

        self.output_motion_ctrl = MVmdFilePickerCtrl(
            self,
            self.frame,
            self,
            title="IK焼き込みモーション出力先",
            is_show_name=False,
            is_save=True,
            tooltip="IK焼き込みモーションの出力ファイルパスです\n任意の値に変更可能です",
        )
        self.output_motion_ctrl.set_parent_sizer(self.file_box_sizer)

        self.header_sizer.Add(self.file_box_sizer, 1, wx.EXPAND | wx.ALL, 3)
        self.root_sizer.Add(self.header_sizer, 1, wx.EXPAND | wx.ALL, 3)

        # 選択肢 -----------------

        self.config_box_sizer = wx.StaticBoxSizer(
            wx.StaticBox(self, wx.ID_ANY, __("焼き込み設定")), orient=wx.VERTICAL
        )
        self.config_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.target_bone_title_ctrl = wx.StaticText(self, wx.ID_ANY, __("対象ボーン"))
        self.config_sizer.Add(self.target_bone_title_ctrl)

        self.target_bone_txt_ctrl = wx.TextCtrl(
            self,
            wx.ID_ANY,
            "",
            wx.DefaultPosition,
            wx.Size(-1, -1),
            wx.TE_READONLY,
        )
        self.target_bone_txt_ctrl.SetBackgroundColour(
            wx.SystemSettings.GetColour(wx.SYS_COLOUR_3DLIGHT)
        )
        self.target_bone_txt_ctrl.SetToolTip(__("焼き込み対象のボーン一覧"))
        self.config_sizer.Add(self.target_bone_txt_ctrl, 1, wx.ALL, 2)

        self.target_bone_btn_ctrl = wx.Button(
            self, wx.ID_ANY, __("ボーン選択"), size=wx.Size(110, 30)
        )
        self.target_bone_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_select_bake_target)
        self.target_bone_btn_ctrl.SetToolTip(__("焼き込み対象のボーンを選択できます"))
        self.config_sizer.Add(self.target_bone_btn_ctrl, 0, wx.ALL, 2)

        self.bake_grain_title_ctrl = wx.StaticText(self, wx.ID_ANY, __("細かさ"))
        self.config_sizer.Add(self.bake_grain_title_ctrl)

        self.bake_grain_slider = FloatSliderCtrl(
            parent=self,
            value=0.5,
            min_value=0,
            max_value=1,
            increment=0.01,
            spin_increment=0.1,
            border=3,
            size=wx.Size(120, -1),
            tooltip=__("焼き込みの細かさを設定できます\n値が大きいほど細かい変動に反応するようになります"),
        )
        self.config_sizer.Add(self.bake_grain_slider.sizer, 0, wx.ALL | wx.BOTTOM, 2)

        self.config_box_sizer.Add(self.config_sizer, 1, wx.GROW | wx.ALL, 3)
        self.root_sizer.Add(self.config_box_sizer, 1, wx.GROW | wx.ALL, 3)

        # ボタン -------------------------
        self.btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.prepare_btn_ctrl = ExecButton(
            self,
            self.frame,
            __("データ読み込み"),
            __("データ読み込み停止"),
            self.prepare,
            250,
            __("指定されたモデルとモーションを読み込みます。"),
        )
        self.btn_sizer.Add(self.prepare_btn_ctrl, 0, wx.ALL, 3)

        self.exec_btn_ctrl = ExecButton(
            self,
            self,
            __("IK焼き込み実行"),
            __("IK焼き込み実行停止"),
            self.exec,
            250,
            __("IK焼き込みを実行します\nモデルとモーションを設定後、クリックできるようになります"),
        )
        self.exec_btn_ctrl.exec_worker = self.bake_worker
        self.btn_sizer.Add(self.exec_btn_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.btn_sizer, 0, wx.ALIGN_CENTER | wx.SHAPED, 3)

        # コンソール -----------------
        self.console_ctrl = ConsoleCtrl(self, self.frame, self, rows=350)
        self.console_ctrl.set_parent_sizer(self.root_sizer)

    def on_select_bake_target(self, event: wx.Event) -> None:
        if not self.motion_ctrl.data:
            logger.warning("モーションが読み込まれていない為、処理を中断します")
        if not self.model_ctrl.data:
            logger.warning("モデルが読み込まれていない為、処理を中断します")

        with TreeDialog(self, __("焼き込み対象ボーン選択"), self.model_ctrl.data) as dialog:
            if dialog.ShowModal() == wx.ID_CANCEL:
                return

            self.selected_bone_names = dialog.get_selected_names()

            # 選択されている場合、ボタンの色を変える
            if 0 < len(self.selected_bone_names):
                self.target_bone_btn_ctrl.SetBackgroundColour(
                    self.active_background_color
                )
                self.target_bone_txt_ctrl.SetValue(", ".join(self.selected_bone_names))
            else:
                self.target_bone_btn_ctrl.SetBackgroundColour(
                    wx.SystemSettings.GetColour(wx.SYS_COLOUR_BTNFACE)
                )
                self.target_bone_txt_ctrl.SetValue("")

            self.EnableExec(True)

    def on_change_model_pmx(self, event: wx.Event) -> None:
        self.model_ctrl.unwrap()
        if self.model_ctrl.read_name():
            self.model_ctrl.read_digest()
            self.create_output_path()
        self.Enable(False)
        self.EnableLoad(True)

    def on_change_motion(self, event: wx.Event) -> None:
        self.motion_ctrl.unwrap()
        if self.motion_ctrl.read_name():
            self.motion_ctrl.read_digest()
            self.create_output_path()
        self.Enable(False)
        self.EnableLoad(True)

    def create_output_path(self) -> None:
        if self.motion_ctrl.valid() and self.model_ctrl.valid():
            (
                motion_dir_path,
                motion_file_name,
                motion_file_ext,
            ) = self.motion_ctrl.separated_path
            (
                model_dir_path,
                model_file_name,
                model_file_ext,
            ) = self.model_ctrl.separated_path

            self.output_motion_ctrl.path = os.path.join(
                motion_dir_path,
                "".join(
                    [
                        f"{motion_file_name}_{model_file_name}_{datetime.now():%Y%m%d_%H%M%S}{motion_file_ext}",
                    ]
                ),
            )

    def save_histories(self) -> None:
        self.motion_ctrl.save_path()
        self.model_ctrl.save_path()

        save_histories(self.frame.histories)

    def prepare(self, event: wx.Event) -> None:
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)

        if not self.load_worker.started:
            if not self.motion_ctrl.valid():
                self.Enable(False)
                self.EnableLoad(True)

                logger.warning("モーション欄に有効なパスが設定されていない為、読み込みを中断します。")
                return
            if not self.model_ctrl.valid():
                self.Enable(False)
                self.EnableLoad(True)

                logger.warning("人物モデル欄に有効なパスが設定されていない為、読み込みを中断します。")
                return

            if not self.model_ctrl.data or not self.motion_ctrl.data:
                # 読み込む
                self.save_histories()

                self.frame.running_worker = True
                self.Enable(False)
                self.EnableLoad(True)
                self.load_worker.start()
            else:
                self.EnableExec(True)

    def exec(self, event: wx.Event) -> None:
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.frame.running_worker = True
        self.save_histories()

        self.Enable(False)
        self.exec_btn_ctrl.Enable(True)
        self.bake_worker.start()

    def on_exec_result(self, result: bool, data: list, elapsed_time: str):
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.console_ctrl.write(f"\n----------------\n{elapsed_time}")

        self.Enable(True)
        self.frame.running_worker = False
        self.frame.on_sound()

        logger.info("IK焼き込み完了", decoration=MLogger.Decoration.BOX)

    def on_prepare_result(self, result: bool, data: list, elapsed_time: str):
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.console_ctrl.write(f"\n----------------\n{elapsed_time}")

        self.Enable(False)
        self.EnableLoad(True)
        self.EnableConfig(True)
        self.frame.running_worker = False
        self.frame.on_sound()

        logger.info(
            "読み込み完了\n「焼き込み対象ボーン選択」ボタンから焼き込みたいIK管理下ボーンを選択してください",
            decoration=MLogger.Decoration.BOX,
        )

    def Enable(self, enable: bool) -> None:
        self.EnableLoad(enable)
        self.EnableConfig(enable)
        self.EnableExec(enable)

    def EnableLoad(self, enable: bool) -> None:
        self.motion_ctrl.Enable(enable)
        self.output_motion_ctrl.Enable(enable)
        self.model_ctrl.Enable(enable)
        self.prepare_btn_ctrl.Enable(enable)

    def EnableConfig(self, enable: bool) -> None:
        self.target_bone_txt_ctrl.Enable(enable)
        self.target_bone_btn_ctrl.Enable(enable)
        self.bake_grain_slider.Enable(enable)

    def EnableExec(self, enable: bool) -> None:
        self.exec_btn_ctrl.Enable(enable)
