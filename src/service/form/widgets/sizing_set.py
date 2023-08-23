import os
from datetime import datetime

import wx

from mlib.service.form.widgets.file_ctrl import MPmxFilePickerCtrl
from mlib.service.form.widgets.file_ctrl import MVmdFilePickerCtrl
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.core.logger import MLogger
from mlib.service.form.widgets.float_slider_ctrl import FloatSliderCtrl


logger = MLogger(os.path.basename(__file__))
__ = logger.get_text

STANCE_DETAIL_CHOICES = [
    __("センターXZ補正: 【注目点: 腰あたりの位置】【補正対象: センターXZ】【有効例: ターン】"),
    __("センターY補正:【注目点: 手首の接地】【補正対象: センターYの位置】【有効例: 倒立】"),
    __("上半身補正:【注目点: 頭の位置】【補正対象: 上半身・上半身2の傾き】【有効例: 上体反らし】"),
    __("肩補正:【注目点: 手首の位置】【補正対象: 肩Pを肩に合併・肩の角度】【有効例: 肩の傾き違い】"),
    __("捩り分散:【注目点: 腕の回転】【補正対象: 腕を腕捩りなどに分散】【有効例: 捩りなしモーション】"),
    __("下半身補正:【注目点: 足ボーンの傾き】【補正対象: 下半身の傾き】【有効例: 四つ足モデル】"),
    __("足IK補正:【注目点: 足首の位置】【補正対象: 足IKの位置】【有効例: 低頭身モデル】"),
    __("足D補正:【注目点: 足D系の回転】【補正対象: 足D系を足に合併】【有効例: 足D使用モーション】"),
    __("つま先補正:【注目点: つま先の接地】【補正対象: 足IKの位置】【有効例: つま先立ち】"),
    __("つま先IK補正:【注目点: 足首の向き】【補正対象: つま先IKを足IKに合併】【有効例: つま先IK使用モーション】"),
]

INITIAL_STANCE_DETAIL_CHOICES = [0, 1, 2, 3, 5, 7, 8, 9]


class SizingSet:
    def __init__(self, window: wx.Window, frame: NotebookFrame, panel: NotebookPanel, sizing_idx: int) -> None:
        self.window = window
        self.frame = frame
        self.panel = panel
        self.sizing_idx = sizing_idx
        self.selected_stance_correct: list[int] = []

        self._initialize_ui()

    def _initialize_ui(self) -> None:
        self.sizer = wx.BoxSizer(wx.VERTICAL)

        self.background_color = wx.Colour("LIGHT GREY") if 0 == self.sizing_idx % 2 else wx.Colour("LIGHT BLUE")
        self.box = wx.StaticBox(self.window, wx.ID_ANY, f"No.{self.sizing_idx + 1}")
        self.box.SetBackgroundColour(self.background_color)

        self.box_sizer = wx.StaticBoxSizer(self.box, orient=wx.HORIZONTAL)

        # ----------------------------------
        self.file_sizer = wx.BoxSizer(wx.VERTICAL)

        self.motion_ctrl = MVmdFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="vmd",
            title="サイジング対象モーション",
            is_show_name=True,
            name_spacer=1,
            is_save=False,
            tooltip="サイジングの対象となるVMDモーションデータを指定してください",
            file_change_event=self.on_change_motion,
        )
        self.motion_ctrl.set_parent_sizer(self.file_sizer)
        self.motion_ctrl.set_color(self.background_color)

        self.src_model_ctrl = MPmxFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="org_pmx",
            title="モーション作成元モデル",
            is_show_name=True,
            name_spacer=2,
            is_save=False,
            tooltip="モーションを作成した時に使用されたトレースモデル（或いは類似モデル）を指定してください",
            file_change_event=self.on_change_src_model_pmx,
        )
        self.src_model_ctrl.set_parent_sizer(self.file_sizer)
        self.src_model_ctrl.set_color(self.background_color)

        self.dest_model_ctrl = MPmxFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="rep_pmx",
            title="サイジング先モデル",
            is_show_name=True,
            name_spacer=4,
            is_save=False,
            tooltip="モーションを適用させたいモデルを指定してください\nこのモデルに合わせてモーションをサイジングします",
            file_change_event=self.on_change_dest_model_pmx,
        )
        self.dest_model_ctrl.set_parent_sizer(self.file_sizer)
        self.dest_model_ctrl.set_color(self.background_color)

        self.output_motion_ctrl = MVmdFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            title="サイジングモーション出力先",
            is_show_name=False,
            is_save=True,
            tooltip="サイジングモーションの出力ファイルパスです\n任意の値に変更可能です",
        )
        self.output_motion_ctrl.set_parent_sizer(self.file_sizer)
        self.output_motion_ctrl.set_color(self.background_color)

        self.box_sizer.Add(self.file_sizer, 6, wx.ALL, 2)

        # ----------------------------------
        self.config_sizer = wx.BoxSizer(wx.VERTICAL)

        # スタンス追加補正 ----------------------
        self.stance_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.stance_correct_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("スタンス追加補正"), wx.DefaultPosition, wx.DefaultSize, 0)
        self.stance_correct_check_ctrl.SetToolTip(__("元モーションの動きにより近くなるよう追加で補正を行います\n補正内容は任意で選択できます"))
        self.stance_correct_check_ctrl.SetBackgroundColour(self.background_color)
        self.stance_correct_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_stance_correct)
        self.stance_sizer.Add(self.stance_correct_check_ctrl, 0, wx.ALL, 2)

        self.stance_correct_btn_ctrl = wx.Button(self.window, wx.ID_ANY, __("補正内訳"), size=wx.Size(120, 20))
        self.stance_correct_btn_ctrl.Bind(wx.EVT_BUTTON, self.on_select_stance_correct)
        self.stance_correct_btn_ctrl.SetToolTip(__("スタンス追加補正で行う補正を取捨選択することができます"))
        self.stance_sizer.Add(self.stance_correct_btn_ctrl, 0, wx.ALL, 2)

        self.config_sizer.Add(self.stance_sizer, 0, wx.ALL, 2)

        self.config_sizer.Add(wx.StaticLine(self.window, wx.ID_ANY), 1, wx.EXPAND | wx.ALL, 10)

        # 位置合わせ ------------------------
        self.align_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.align_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("手首位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0)
        self.align_check_ctrl.SetToolTip(__("手首の位置を元モーションと大体同じ位置になるよう合わせます"))
        self.align_check_ctrl.SetBackgroundColour(self.background_color)
        self.align_sizer.Add(self.align_check_ctrl, 0, wx.ALL, 2)

        self.separate2 = wx.StaticText(self.window, wx.ID_ANY, "     |     ")
        self.separate2.SetBackgroundColour(self.background_color)
        self.align_sizer.Add(self.separate2)

        self.align_finger_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("指位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0)
        self.align_finger_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_align_ctrl)
        self.align_finger_check_ctrl.SetToolTip(__("指の位置を元モーションと大体同じ位置になるよう合わせます"))
        self.align_finger_check_ctrl.SetBackgroundColour(self.background_color)
        self.align_sizer.Add(self.align_finger_check_ctrl, 0, wx.ALL, 2)

        self.separate3 = wx.StaticText(self.window, wx.ID_ANY, "     |     ")
        self.separate3.SetBackgroundColour(self.background_color)
        self.align_sizer.Add(self.separate3)

        self.align_floor_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("床位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0)
        self.align_floor_check_ctrl.Bind(wx.EVT_CHECKBOX, self.on_check_align_ctrl)
        self.align_floor_check_ctrl.SetToolTip(__("手首の位置が床から沈まないよう合わせます"))
        self.align_floor_check_ctrl.SetBackgroundColour(self.background_color)
        self.align_sizer.Add(self.align_floor_check_ctrl, 0, wx.ALL, 2)

        self.config_sizer.Add(self.align_sizer, 0, wx.ALL, 2)

        self.config_sizer.Add(wx.StaticLine(self.window, wx.ID_ANY), 1, wx.EXPAND | wx.ALL, 10)

        # 足IKオフセット ------------------------
        self.offset_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.leg_ik_offset_label = wx.StaticText(self.window, wx.ID_ANY, __("足IKオフセット"))
        self.leg_ik_offset_label.SetBackgroundColour(self.background_color)
        self.offset_sizer.Add(self.leg_ik_offset_label, 0, wx.ALL, 2)

        self.leg_ik_offset_slider = FloatSliderCtrl(
            parent=self.window,
            value=0,
            min_value=-5,
            max_value=5,
            increment=0.01,
            spin_increment=0.01,
            border=3,
            size=wx.Size(270, -1),
            change_event=None,
            tooltip=__("モーションで足が重なってしまう場合などで、足を少し開き気味にするなどのオフセットを追加できます"),
        )
        self.leg_ik_offset_slider._slider.SetBackgroundColour(self.background_color)
        self.offset_sizer.Add(self.leg_ik_offset_slider.sizer, 0, wx.ALL, 2)

        self.config_sizer.Add(self.offset_sizer, 0, wx.ALL, 2)

        self.box_sizer.Add(self.config_sizer, 4, wx.ALL, 0)
        self.sizer.Add(self.box_sizer, 1, wx.EXPAND | wx.ALL, 0)

    def on_check_align_ctrl(self, event: wx.Event) -> None:
        if not self.align_finger_check_ctrl.GetValue() and not self.align_floor_check_ctrl.GetValue():
            self.align_check_ctrl.SetValue(0)
        else:
            self.align_check_ctrl.SetValue(1)

    def on_check_stance_correct(self, event: wx.Event) -> None:
        if self.stance_correct_check_ctrl.GetValue():
            # チェックがONになった場合、スタンス追加補正のINDEXを追加
            if not self.selected_stance_correct:
                self.selected_stance_correct = INITIAL_STANCE_DETAIL_CHOICES
            self.stance_correct_btn_ctrl.SetBackgroundColour(self.panel.active_background_color)
            self.stance_correct_check_ctrl.SetValue(1)
        else:
            self.selected_stance_correct = []
            self.stance_correct_btn_ctrl.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_BTNFACE))
            self.stance_correct_check_ctrl.SetValue(0)

    def on_select_stance_correct(self, event: wx.Event) -> None:
        with wx.MultiChoiceDialog(
            self.window,
            __("スタンス追加補正のうち、チェックが入っている補正のみ実施します"),
            caption=__("スタンス追加補正選択"),
            choices=STANCE_DETAIL_CHOICES,
            style=wx.CHOICEDLG_STYLE,
        ) as choiceDialog:
            choiceDialog.SetSelections(self.selected_stance_correct or INITIAL_STANCE_DETAIL_CHOICES)

            if choiceDialog.ShowModal() == wx.ID_CANCEL:
                return

            self.selected_stance_correct = choiceDialog.GetSelections()

            # 選択されている場合、ボタンの色を変える
            if 0 < len(self.selected_stance_correct):
                self.stance_correct_btn_ctrl.SetBackgroundColour(self.panel.active_background_color)
                self.stance_correct_check_ctrl.SetValue(1)
            else:
                self.stance_correct_btn_ctrl.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_BTNFACE))
                self.stance_correct_check_ctrl.SetValue(0)

    def on_change_src_model_pmx(self, event: wx.Event) -> None:
        self.src_model_ctrl.unwrap()
        if self.src_model_ctrl.read_name():
            self.src_model_ctrl.read_digest()
            self.create_output_path()
        if not self.src_model_ctrl.data:
            self.frame.Enable(False)

    def on_change_dest_model_pmx(self, event: wx.Event) -> None:
        self.dest_model_ctrl.unwrap()
        if self.dest_model_ctrl.read_name():
            self.dest_model_ctrl.read_digest()
            self.create_output_path()
        if not self.dest_model_ctrl.data:
            self.frame.Enable(False)

    def on_change_motion(self, event: wx.Event) -> None:
        self.motion_ctrl.unwrap()
        if self.motion_ctrl.read_name():
            self.motion_ctrl.read_digest()
        if not self.motion_ctrl.data:
            self.frame.Enable(False)

    def create_output_path(self) -> None:
        if self.motion_ctrl.valid() and self.src_model_ctrl.valid() and self.dest_model_ctrl.valid():
            motion_dir_path, motion_file_name, motion_file_ext = self.motion_ctrl.separated_path
            # src_model_dir_path, src_model_file_name, src_model_file_ext = self.src_model_ctrl.separated_path
            dest_model_dir_path, dest_model_file_name, dest_model_file_ext = self.dest_model_ctrl.separated_path

            sizing_types: list[str] = []
            self.output_motion_ctrl.path = os.path.join(
                motion_dir_path,
                "".join(
                    [
                        f"{motion_file_name}_{dest_model_file_name}_{''.join(sizing_types)}",
                        f"{('_' if sizing_types else '')}{datetime.now():%Y%m%d_%H%M%S}{motion_file_ext}",
                    ]
                ),
            )
