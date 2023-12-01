SIZING_BONE_PREFIX = "[SZ]"


class BoneNames:
    @staticmethod
    def root():
        return f"{SIZING_BONE_PREFIX}ROOT"

    @staticmethod
    def shoulder_root(direction: str) -> str:
        return f"{direction}肩根元"

    @staticmethod
    def shoulder_p(direction: str) -> str:
        return f"{direction}肩P"

    @staticmethod
    def shoulder(direction: str) -> str:
        return f"{direction}肩"

    @staticmethod
    def shoulder_c(direction: str) -> str:
        return f"{direction}肩C"

    @staticmethod
    def shoulder_center(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}肩中点"

    @staticmethod
    def shoulder_ik_parent(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}肩IK親"

    @staticmethod
    def shoulder_ik(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}肩IK"

    @staticmethod
    def arm(direction: str) -> str:
        return f"{direction}腕"

    @staticmethod
    def arm_twist(direction: str, index: int = 0):
        return f"{direction}腕捩" if index == 0 else f"{direction}腕捩{index}"

    @staticmethod
    def arm_ik_parent(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}腕IK親"

    @staticmethod
    def arm_ik(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}腕IK"

    @staticmethod
    def elbow(direction: str) -> str:
        return f"{direction}ひじ"

    @staticmethod
    def elbow_ik_parent(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}ひじIK親"

    @staticmethod
    def elbow_ik(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}ひじIK"

    @staticmethod
    def elbow_center(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}ひじ中点"

    @staticmethod
    def elbow_rotate(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}ひじ回転"

    @staticmethod
    def wrist_twist(direction: str, index: int = 0):
        return f"{direction}手捩" if index == 0 else f"{direction}手捩{index}"

    @staticmethod
    def wrist(direction: str) -> str:
        return f"{direction}手首"

    @staticmethod
    def wrist_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}手首先"

    @staticmethod
    def wrist_rotate(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}手首回転"

    @staticmethod
    def wrist_ik_parent(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}手首IK親"

    @staticmethod
    def wrist_ik(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}手首IK"

    @staticmethod
    def thumb0(direction: str) -> str:
        return f"{direction}親指０"

    @staticmethod
    def thumb_ik_parent(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}親IK親"

    @staticmethod
    def thumb_ik(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}親IK"

    @staticmethod
    def thumb1(direction: str) -> str:
        return f"{direction}親指１"

    @staticmethod
    def thumb2(direction: str) -> str:
        return f"{direction}親指２"

    @staticmethod
    def thumb_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}親先"

    @staticmethod
    def index1(direction: str) -> str:
        return f"{direction}人指１"

    @staticmethod
    def index2(direction: str) -> str:
        return f"{direction}人指２"

    @staticmethod
    def index3(direction: str) -> str:
        return f"{direction}人指３"

    @staticmethod
    def index_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}人先"

    @staticmethod
    def middle1(direction: str) -> str:
        return f"{direction}中指１"

    @staticmethod
    def middle2(direction: str) -> str:
        return f"{direction}中指２"

    @staticmethod
    def middle3(direction: str) -> str:
        return f"{direction}中指３"

    @staticmethod
    def middle_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}中先"

    @staticmethod
    def ring1(direction: str) -> str:
        return f"{direction}薬指１"

    @staticmethod
    def ring2(direction: str) -> str:
        return f"{direction}薬指２"

    @staticmethod
    def ring3(direction: str) -> str:
        return f"{direction}薬指３"

    @staticmethod
    def ring_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}薬先"

    @staticmethod
    def pinky1(direction: str) -> str:
        return f"{direction}小指１"

    @staticmethod
    def pinky2(direction: str) -> str:
        return f"{direction}小指２"

    @staticmethod
    def pinky3(direction: str) -> str:
        return f"{direction}小指３"

    @staticmethod
    def pinky_tail(direction: str) -> str:
        return f"{SIZING_BONE_PREFIX}{direction}小先"

    @staticmethod
    def fingers(direction: str) -> list[str]:
        return [
            BoneNames.thumb0(direction),
            BoneNames.thumb1(direction),
            BoneNames.thumb2(direction),
            BoneNames.thumb_tail(direction),
            BoneNames.index1(direction),
            BoneNames.index2(direction),
            BoneNames.index3(direction),
            BoneNames.index_tail(direction),
            BoneNames.middle1(direction),
            BoneNames.middle2(direction),
            BoneNames.middle3(direction),
            BoneNames.middle_tail(direction),
            BoneNames.ring1(direction),
            BoneNames.ring2(direction),
            BoneNames.ring3(direction),
            BoneNames.ring_tail(direction),
            BoneNames.pinky1(direction),
            BoneNames.pinky2(direction),
            BoneNames.pinky3(direction),
            BoneNames.pinky_tail(direction),
        ]

    @staticmethod
    def parent() -> str:
        return "全ての親"

    @staticmethod
    def center() -> str:
        return "センター"

    @staticmethod
    def groove() -> str:
        return "グルーブ"

    @staticmethod
    def leg(direction: str) -> str:
        return f"{direction}足"

    @staticmethod
    def knee(direction: str) -> str:
        return f"{direction}ひざ"

    @staticmethod
    def ankle(direction: str) -> str:
        return f"{direction}足首"

    @staticmethod
    def leg_ik_parent(direction: str) -> str:
        return f"{direction}足IK親"

    @staticmethod
    def leg_ik(direction: str) -> str:
        return f"{direction}足ＩＫ"

    @staticmethod
    def toe_ik(direction: str) -> str:
        return f"{direction}つま先ＩＫ"
