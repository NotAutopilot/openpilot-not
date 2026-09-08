from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  # Capability contract v1 fields. Pin capnp ordinals @6-@16; do not reuse @0-@5.
  madsFullSettingsAvailable: bool = auto_field()
  madsMainCruiseInputKind: 'CarParamsSP.MadsMainCruiseInputKind' = field(
    default_factory=lambda: CarParamsSP.MadsMainCruiseInputKind.none
  )
  madsMainCruiseAllowed: bool = auto_field()
  madsRequired: bool = auto_field()
  teslaCoopSteeringAvailable: bool = auto_field()
  madsUnifiedEngagementMode: bool = auto_field()
  madsSteeringMode: 'CarParamsSP.MadsSteeringMode' = field(
    default_factory=lambda: CarParamsSP.MadsSteeringMode.remainActive
  )
  madsCapabilityContractVersion: int = auto_field()
  madsHandsOnPauseAvailable: bool = auto_field()
  preapLateralEngagementMode: 'CarParamsSP.PreapLateralEngagementMode' = field(
    default_factory=lambda: CarParamsSP.PreapLateralEngagementMode.independent
  )
  radarOffset: float = auto_field()

  class MadsMainCruiseInputKind(StrEnum):
    none = auto()
    stateful = auto()
    momentary = auto()

  class MadsSteeringMode(StrEnum):
    remainActive = auto()
    pause = auto()
    disengage = auto()

  class PreapLateralEngagementMode(StrEnum):
    independent = auto()
    cruiseCoupled = auto()
    longitudinalOnly = auto()

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()
  # Pre-AP intent outbox. Pin capnp ordinals @1-@4; speedLimit remains @0.
  preapLateralIntent: 'CarStateSP.PreapLateralIntent' = field(
    default_factory=lambda: CarStateSP.PreapLateralIntent.none
  )
  preapLongitudinalIntent: 'CarStateSP.PreapLongitudinalIntent' = field(
    default_factory=lambda: CarStateSP.PreapLongitudinalIntent.none
  )
  preapIntentSequence: int = auto_field()
  preapIntentEpoch: int = auto_field()
  # Stock-CC transaction outbox. Pin capnp ordinals @5-@8.
  preapStockCcState: 'CarStateSP.PreapStockCcTransactionState' = field(
    default_factory=lambda: CarStateSP.PreapStockCcTransactionState.idle
  )
  preapStockCcBoundCounter: int = auto_field()
  preapStockCcHostDiConfirmed: bool = auto_field()
  preapStockCcEnablePending: bool = auto_field()
  # Pre-AP pedal authority and VDAS diagnostics. Pin capnp ordinals @9-@19.
  # enableLongControl is @20.
  pedalMaxRegen: bool = auto_field()
  pedalLongActive: bool = auto_field()
  pedalAuthorityRequested: bool = auto_field()
  pedalAuthorityState: int = auto_field()
  pedalAuthorityAction: int = auto_field()
  pedalCommandCounter: int = auto_field()
  pedalFeedbackState: int = auto_field()
  pedalFeedbackCounter: int = auto_field()
  pedalCommandDi: float = auto_field()
  pedalAuthorityFailed: bool = auto_field()
  vdasLimitedAccel: float = auto_field()
  # Pre-AP FSM longitudinal intent. Pin capnp ordinal @20. Survives gas override.
  enableLongControl: bool = auto_field()

  class PreapLateralIntent(StrEnum):
    none = auto()
    mainCruiseRequest = auto()
    forceDisable = auto()

  class PreapLongitudinalIntent(StrEnum):
    none = auto()
    enable = auto()
    disable = auto()

  class PreapStockCcTransactionState(StrEnum):
    idle = auto()
    cancelRequested = auto()
    awaitingCancelConfirmation = auto()
    awaitingSecondPull = auto()
    reengageRequested = auto()
    awaitingDiConfirmation = auto()
    confirmed = auto()
    cancelledOrFailed = auto()
