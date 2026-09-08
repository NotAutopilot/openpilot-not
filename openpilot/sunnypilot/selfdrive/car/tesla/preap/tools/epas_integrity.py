"""EPAS firmware and bootloader integrity. Verify size, SHA-256, and MD5 before panda connect."""
from __future__ import annotations

import hashlib
import lzma
from pathlib import Path

BOOTLOADER_NAME = "epas-bootloader-0x3ff7000-0x3ffacbd.bin"
BOOTLOADER_SIZE = 15550
BOOTLOADER_MD5SUM = "09cdd03705692725290942f4065c2706"
BOOTLOADER_SHA256 = "5fdd472626f110ad83ea04b625b64cb4e0a761c6976110a67ed6993df1167b90"
BOOTLOADER_ADDR = 0x3ff7000

FW_NAME = "epas-firmware-0x7000-0x45fff.bin"
FW_PACKAGED_NAME = FW_NAME + ".xz"
FW_MD5SUM = "9e51ddd80606fbdaaf604c73c8dde0d1"
FW_SHA256 = "93dbe0ea4953611fd56c26ef918ea05897e0f02e776700a28ceb6a3df04126cd"
FW_START_ADDR = 0x7000
FW_END_ADDR = 0x45FFF
FW_SIZE = FW_END_ADDR - FW_START_ADDR + 1

_XZ_MAGIC = b"\xfd7zXZ\x00"


class BootloaderIntegrityError(Exception):
  """Bootloader file failed size/SHA-256/MD5 verification."""


class FirmwareIntegrityError(Exception):
  """Stock EPAS image failed size/SHA-256/MD5 verification."""


def _firmware_dir() -> Path:
  return Path(__file__).resolve().parent / "firmware"


def firmware_packaged_path() -> Path:
  return _firmware_dir() / FW_PACKAGED_NAME


def firmware_runtime_path() -> Path:
  return _firmware_dir() / FW_NAME


def decode_firmware_image(path: str | Path) -> bytes:
  """Stdlib lzma decode at the load boundary. Raw bytes pass through."""
  blob = Path(path).read_bytes()
  if Path(path).suffix == ".xz" or blob.startswith(_XZ_MAGIC):
    return lzma.decompress(blob, format=lzma.FORMAT_AUTO)
  return blob


def verify_firmware(data: bytes) -> bytes:
  size = len(data)
  md5 = hashlib.md5(data).hexdigest()
  sha256 = hashlib.sha256(data).hexdigest()
  if size != FW_SIZE or md5 != FW_MD5SUM or sha256 != FW_SHA256:
    raise FirmwareIntegrityError(
      "firmware integrity check failed: "
      + f"expected size={FW_SIZE} md5={FW_MD5SUM} sha256={FW_SHA256}, "
      + f"got size={size} md5={md5} sha256={sha256}"
    )
  return data


def load_stock_firmware() -> bytes:
  """Read the packaged xz image and verify the decoded payload."""
  path = firmware_packaged_path()
  if not path.is_file():
    raise FirmwareIntegrityError(f"packaged firmware missing: {path}")
  return verify_firmware(decode_firmware_image(path))


def bootloader_path(explicit: str | None = None) -> Path:
  if explicit:
    path = Path(explicit)
    if not path.is_absolute():
      path = Path(__file__).resolve().parent / "firmware" / path
    return path
  return Path(__file__).resolve().parent / "firmware" / BOOTLOADER_NAME


def verify_bootloader(path: str | Path | None = None) -> bytes:
  """Read and verify the bootloader. Must run before opening panda."""
  bl_path = Path(path) if path is not None else bootloader_path()
  if not bl_path.is_file():
    raise BootloaderIntegrityError(f"bootloader missing: {bl_path}")
  data = bl_path.read_bytes()
  size = len(data)
  md5 = hashlib.md5(data).hexdigest()
  sha256 = hashlib.sha256(data).hexdigest()
  if size != BOOTLOADER_SIZE or md5 != BOOTLOADER_MD5SUM or sha256 != BOOTLOADER_SHA256:
    raise BootloaderIntegrityError(
      "bootloader integrity check failed: "
      + f"expected size={BOOTLOADER_SIZE} md5={BOOTLOADER_MD5SUM} sha256={BOOTLOADER_SHA256}, "
      + f"got size={size} md5={md5} sha256={sha256}"
    )
  return data
