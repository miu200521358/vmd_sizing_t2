SIZING_BONE_PREFIX = "[SZ]"


class BoneNames:
    @classmethod
    def root(cls):
        return f"{SIZING_BONE_PREFIX}ROOT"

    @classmethod
    def shoulder_root(cls, direction: str):
        return f"{direction}肩根元"

    @classmethod
    def shoulder_p(cls, direction: str):
        return f"{direction}肩P"

    @classmethod
    def shoulder(cls, direction: str):
        return f"{direction}肩"

    @classmethod
    def shoulder_c(cls, direction: str):
        return f"{direction}肩C"

    @classmethod
    def shoulder_center(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}肩中点"

    @classmethod
    def shoulder_ik_parent(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}肩IK親"

    @classmethod
    def shoulder_ik(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}肩IK"

    @classmethod
    def arm(cls, direction: str):
        return f"{direction}腕"

    @classmethod
    def arm_twist(cls, direction: str, index: int = 0):
        return f"{direction}腕捩" if index == 0 else f"{direction}腕捩{index}"

    @classmethod
    def arm_ik_parent(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}腕IK親"

    @classmethod
    def arm_ik(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}腕IK"

    @classmethod
    def elbow(cls, direction: str):
        return f"{direction}ひじ"

    @classmethod
    def hand_twist(cls, direction: str, index: int = 0):
        return f"{direction}手捩" if index == 0 else f"{direction}手捩{index}"

    @classmethod
    def wrist(cls, direction: str):
        return f"{direction}手首"

    @classmethod
    def wrist_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}手首先"

    @classmethod
    def wrist_ik_parent(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}手首IK親"

    @classmethod
    def wrist_ik(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}手首IK"

    @classmethod
    def thumb0(cls, direction: str):
        return f"{direction}親指０"

    @classmethod
    def thumb_ik_parent(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}親０IK親"

    @classmethod
    def thumb_ik(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}親０IK"

    @classmethod
    def thumb1(cls, direction: str):
        return f"{direction}親指１"

    @classmethod
    def thumb2(cls, direction: str):
        return f"{direction}親指２"

    @classmethod
    def thumb_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}親先"

    @classmethod
    def index1(cls, direction: str):
        return f"{direction}人指１"

    @classmethod
    def index2(cls, direction: str):
        return f"{direction}人指２"

    @classmethod
    def index3(cls, direction: str):
        return f"{direction}人指３"

    @classmethod
    def index_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}人先"

    @classmethod
    def middle1(cls, direction: str):
        return f"{direction}中指１"

    @classmethod
    def middle2(cls, direction: str):
        return f"{direction}中指２"

    @classmethod
    def middle3(cls, direction: str):
        return f"{direction}中指３"

    @classmethod
    def middle_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}中先"

    @classmethod
    def ring1(cls, direction: str):
        return f"{direction}薬指１"

    @classmethod
    def ring2(cls, direction: str):
        return f"{direction}薬指２"

    @classmethod
    def ring3(cls, direction: str):
        return f"{direction}薬指３"

    @classmethod
    def ring_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}薬先"

    @classmethod
    def pinky1(cls, direction: str):
        return f"{direction}小指１"

    @classmethod
    def pinky2(cls, direction: str):
        return f"{direction}小指２"

    @classmethod
    def pinky3(cls, direction: str):
        return f"{direction}小指３"

    @classmethod
    def pinky_tail(cls, direction: str):
        return f"{SIZING_BONE_PREFIX}{direction}小先"
