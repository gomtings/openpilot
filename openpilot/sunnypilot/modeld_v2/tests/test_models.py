import os
import pytest
from openpilot.common.file_chunker import open_file_chunked
from openpilot.sunnypilot.modeld_v2.helpers import load_oob


class TestLegacyModels:
  def test_legacy_model_load(self):
    base_name = os.environ.get("MODEL_BASE_NAME")
    if not base_name:
      pytest.skip("MODEL_BASE_NAME env var not set, skipping integration test.")
    chunk_dir = os.environ.get("MODEL_CHUNK_DIR", "/tmp/model_chunks")
    base_path = os.path.join(chunk_dir, base_name)

    try:
      f = open_file_chunked(base_path)
    except Exception as e:
      pytest.fail(f"Failed to open chunked file {base_path}: {e}")

    obj = load_oob(f)
    assert isinstance(obj, dict), "Parsed object is not a dictionary"
    assert 'metadata' in obj, "Metadata key is missing"
