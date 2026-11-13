"""Reproducible sim-to-real randomization sampling for the navigation stack.

Frozen interface, see ``plan/2026-11-12-autonomy-randomization.md`` section 3.2:

* profiles live in ``config/randomization/<profile_id>.json``;
* a profile is a dict of parameter groups whose entries are ``[lo, hi]`` uniform
  ranges, a bare number (fixed value) or ``null`` (keep the profile baseline);
* :func:`sample` returns ``{"profile_id", "seed", "values", "provenance"}`` with
  flat keys such as ``lidar.range_noise_m``;
* :func:`apply_to_lidar` returns only the five ``config["lidar"]`` override keys
  and :func:`executor_parameters` returns the flat ``odometry.*`` /
  ``actuation.*`` / ``initial_pose.*`` values;
* :func:`record` builds a JSON-serializable audit record with per-parameter
  provenance (``sim_assumed`` / ``vendor_reference`` / ``measured_pending``).

Every draw uses a private ``numpy.random.Generator`` seeded from the run seed, so
sampling never touches the global ``numpy.random`` state and the same
``(profile, seed)`` pair always reproduces bit-identical values.
"""

import json
from pathlib import Path

import numpy as np


SCHEMA_VERSION = 1
PROVENANCE_LABELS = ("sim_assumed", "vendor_reference", "measured_pending")
DEFAULT_PROVENANCE = "sim_assumed"

PROJECT = Path(__file__).resolve().parents[3]
PROFILES_DIRECTORY = PROJECT / "config" / "randomization"

# ``lidar.<flat>`` -> the key accepted by the LiDAR observation adapter.  Only
# these five keys may be returned by :func:`apply_to_lidar`.
LIDAR_PARAMETER_MAP = {
    "lidar.range_noise_m": "noise_m",
    "lidar.range_bias_m": "range_bias_m",
    "lidar.dropout_probability": "dropout_probability",
    "lidar.beam_block_probability": "beam_block_probability",
    "lidar.scan_latency_s": "scan_latency_s",
}
EXECUTOR_GROUPS = ("odometry", "actuation", "initial_pose")

# Top-level profile keys that describe the profile instead of defining parameters.
METADATA_KEYS = {
    "schema_version", "profile_id", "description", "notes", "source", "schema",
    "provenance", "baseline", "tags", "created_utc",
}


def available_profiles(base_dir=None):
    """Return the sorted profile ids found in the randomization directory."""
    directory = _profiles_directory(base_dir)
    if not directory.is_dir():
        return []
    return sorted(path.stem for path in directory.glob("*.json"))


def load_profile(path_or_id, base_dir=None):
    """Load and normalize a randomization profile.

    ``path_or_id`` may be a profile id (``"sim2real_v1"``), a path to a ``.json``
    file, or a path relative to ``base_dir``.  ``base_dir`` is normally the
    ``lite2_sim`` project directory; omitted means this checkout's directory.
    """
    path = _resolve_profile_path(path_or_id, base_dir)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError("Invalid randomization profile %s: %s" % (path, error)) from error
    return normalize_profile(raw, source_path=_source_label(path), profile_id=path.stem)


def normalize_profile(raw, source_path=None, profile_id=None):
    """Normalize a profile mapping into ranged, provenance-tagged parameters."""
    if not isinstance(raw, dict):
        raise ValueError("A randomization profile must be a JSON object.")
    schema_version = raw.get("schema_version", SCHEMA_VERSION)
    if schema_version != SCHEMA_VERSION:
        raise ValueError("Unsupported randomization profile schema_version: %r" % (schema_version,))
    resolved_id = raw.get("profile_id", profile_id)
    if not isinstance(resolved_id, str) or not resolved_id:
        raise ValueError("A randomization profile requires a non-empty profile_id.")

    baselines = _baseline_values(raw.get("baseline"))
    provenance_table = raw.get("provenance", {})

    parameters = {}
    for group in sorted(key for key in raw if key not in METADATA_KEYS):
        entries = raw[group]
        if not isinstance(entries, dict):
            raise ValueError("Randomization group %r must be a JSON object." % (group,))
        for name in sorted(entries):
            flat_key = "%s.%s" % (group, name)
            spec = entries[name]
            baseline = _lookup_baseline(baselines, group, name, flat_key)
            distribution, low, high, parameter_baseline, spec_provenance = _normalize_parameter(
                spec, baseline, flat_key)
            parameters[flat_key] = {
                "group": group,
                "name": name,
                "distribution": distribution,
                "low": low,
                "high": high,
                "baseline": parameter_baseline,
                "provenance": _resolve_provenance(
                    provenance_table, group, name, flat_key, spec_provenance),
            }

    if not parameters:
        raise ValueError("Randomization profile %s defines no parameters." % (resolved_id,))
    return {
        "schema_version": SCHEMA_VERSION,
        "profile_id": resolved_id,
        "source_path": source_path,
        "parameters": parameters,
    }


def sample(profile, seed):
    """Draw one reproducible sample from a profile.

    ``profile`` is a normalized profile from :func:`load_profile`, a raw profile
    mapping, or a profile id / path.  ``seed`` must be a non-negative integer.
    """
    normalized = _as_profile(profile)
    seed = _validate_seed(seed)
    generator = np.random.default_rng(seed)
    values, provenance = {}, {}
    for flat_key, parameter in normalized["parameters"].items():
        low, high = parameter["low"], parameter["high"]
        value = float(generator.uniform(low, high)) if high > low else float(low)
        values[flat_key] = value
        provenance[flat_key] = parameter["provenance"]
    return {"profile_id": normalized["profile_id"], "seed": seed,
            "values": values, "provenance": provenance}


def apply_to_lidar(values, lidar_settings=None):
    """Return the ``config["lidar"]`` override dict for a sampled value mapping.

    Only the five frozen keys may appear; keys whose flat ``lidar.*`` parameter
    is absent from ``values`` are omitted so the caller's ``dict.update()`` keeps
    the existing setting untouched.  ``lidar_settings`` is accepted for the
    frozen call signature and is never mutated.
    """
    overrides = {}
    for flat_key, target_key in LIDAR_PARAMETER_MAP.items():
        if flat_key in values:
            overrides[target_key] = float(values[flat_key])
    return overrides


def executor_parameters(values, groups=EXECUTOR_GROUPS):
    """Return the flat numeric ``odometry.*``/``actuation.*``/``initial_pose.*`` values."""
    selected = {}
    for flat_key in sorted(values):
        group = flat_key.split(".", 1)[0]
        if group in groups:
            selected[flat_key] = float(values[flat_key])
    return selected


def record(sampled, sample_time_utc=None):
    """Build the JSON-serializable audit record for a sampled randomization."""
    if not isinstance(sampled, dict):
        raise ValueError("record() requires a sampled randomization mapping.")
    missing = [key for key in ("profile_id", "seed", "values", "provenance") if key not in sampled]
    if missing:
        raise ValueError("Sampled randomization is missing keys: %s" % (", ".join(missing),))
    values = sampled["values"]
    provenance = sampled["provenance"]
    if set(values) != set(provenance):
        raise ValueError("Sampled values and provenance must cover the same parameters.")
    invalid = sorted({label for label in provenance.values() if label not in PROVENANCE_LABELS})
    if invalid:
        raise ValueError("Unknown provenance labels: %s" % (", ".join(map(str, invalid)),))
    audit = {
        "schema_version": SCHEMA_VERSION,
        "profile_id": str(sampled["profile_id"]),
        "seed": int(sampled["seed"]),
        "values": {key: float(value) for key, value in values.items()},
        "provenance": {key: str(label) for key, label in provenance.items()},
        "provenance_summary": {label: sum(1 for value in provenance.values() if value == label)
                               for label in PROVENANCE_LABELS
                               if label in set(provenance.values())},
        "parameter_count": len(values),
    }
    if sample_time_utc is not None:
        audit["sample_time_utc"] = str(sample_time_utc)
    return audit


def write_record(sampled, path, sample_time_utc=None):
    """Write :func:`record` output as pretty JSON and return the record."""
    audit = record(sampled, sample_time_utc=sample_time_utc)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    return audit


def apply_sampling(config, path_or_id, seed, base_dir=None):
    """Sample a profile and set ``config["randomization_sampled"]``.

    Returns the sampled mapping; ``config["randomization_sampled"]`` stays
    ``None`` when randomization is disabled and is never set by this module
    otherwise.
    """
    sampled = sample(load_profile(path_or_id, base_dir=base_dir), seed)
    config["randomization_sampled"] = sampled
    return sampled


def _as_profile(profile):
    if isinstance(profile, dict):
        if "parameters" in profile and "profile_id" in profile:
            return profile
        return normalize_profile(profile)
    return load_profile(profile)


def _validate_seed(seed):
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("Randomization seed must be a non-negative integer, got %r." % (seed,))
    if seed < 0:
        raise ValueError("Randomization seed must be a non-negative integer, got %r." % (seed,))
    return int(seed)


def _normalize_parameter(spec, baseline, flat_key):
    """Return ``(distribution, low, high, baseline, provenance_hint)``."""
    provenance_hint = None
    if isinstance(spec, dict):
        provenance_hint = spec.get("provenance")
        if "baseline" in spec:
            baseline = _finite_number(spec["baseline"], flat_key)
        if "range" in spec:
            spec = spec["range"]
        elif "value" in spec:
            spec = spec["value"]
        elif "baseline" in spec:
            spec = None
        else:
            raise ValueError("Parameter %s must use range/value/baseline." % (flat_key,))
    if spec is None:
        return "constant", baseline, baseline, baseline, provenance_hint
    if isinstance(spec, bool):
        raise ValueError("Parameter %s must be numeric, not a boolean." % (flat_key,))
    if isinstance(spec, (int, float)):
        value = _finite_number(spec, flat_key)
        return "constant", value, value, baseline, provenance_hint
    if isinstance(spec, (list, tuple)):
        if len(spec) == 1:
            value = _finite_number(spec[0], flat_key)
            return "constant", value, value, baseline, provenance_hint
        if len(spec) == 2:
            low = _finite_number(spec[0], flat_key)
            high = _finite_number(spec[1], flat_key)
            if low > high:
                raise ValueError("Parameter %s has an inverted range [%r, %r]." % (flat_key, low, high))
            distribution = "constant" if low == high else "uniform"
            return distribution, low, high, baseline, provenance_hint
    raise ValueError("Parameter %s must be a number, [lo, hi] range or null." % (flat_key,))


def _finite_number(value, flat_key):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Parameter %s must be a finite number, got %r." % (flat_key, value))
    value = float(value)
    if not np.isfinite(value):
        raise ValueError("Parameter %s must be a finite number, got %r." % (flat_key, value))
    return value


def _baseline_values(block):
    """Flatten the optional ``baseline`` block (nested groups or flat keys)."""
    if block is None:
        return {}
    if not isinstance(block, dict):
        raise ValueError("The profile baseline block must be a JSON object.")
    flat = {}
    for key, value in block.items():
        if isinstance(value, dict):
            for name, nested in value.items():
                flat["%s.%s" % (key, name)] = nested
        else:
            flat[key] = value
    return flat


def _lookup_baseline(baselines, group, name, flat_key):
    value = baselines.get(flat_key, baselines.get(name, 0.0))
    return _finite_number(value, flat_key)


def _resolve_provenance(table, group, name, flat_key, parameter_hint):
    if parameter_hint is not None:
        label = parameter_hint
    elif isinstance(table, str):
        label = table
    elif isinstance(table, dict):
        label = table.get(flat_key)
        if label is None:
            group_table = table.get(group)
            if isinstance(group_table, dict):
                label = group_table.get(name, group_table.get("default"))
        if label is None:
            label = table.get("default", DEFAULT_PROVENANCE)
    else:
        label = DEFAULT_PROVENANCE
    if label not in PROVENANCE_LABELS:
        raise ValueError("Parameter %s has unknown provenance %r (expected one of %s)."
                         % (flat_key, label, ", ".join(PROVENANCE_LABELS)))
    return label


def _profiles_directory(base_dir):
    if base_dir is None:
        return PROFILES_DIRECTORY
    return Path(base_dir) / "config" / "randomization"


def _resolve_profile_path(path_or_id, base_dir):
    if isinstance(path_or_id, Path):
        candidate = path_or_id
        if not candidate.is_absolute() and base_dir is not None:
            candidate = Path(base_dir) / candidate
        return _require_file(candidate)
    if not isinstance(path_or_id, str) or not path_or_id:
        raise ValueError("A randomization profile id or path is required.")
    looks_like_path = path_or_id.endswith(".json") or "/" in path_or_id or "\\" in path_or_id
    if looks_like_path:
        candidate = Path(path_or_id)
        if not candidate.is_absolute() and base_dir is not None:
            candidate = Path(base_dir) / candidate
        return _require_file(candidate)
    candidate = _profiles_directory(base_dir) / (path_or_id + ".json")
    if not candidate.is_file():
        known = available_profiles(base_dir)
        raise FileNotFoundError("Unknown randomization profile %r under %s (available: %s)."
                                % (path_or_id, candidate.parent,
                                   ", ".join(known) if known else "none"))
    return candidate


def _require_file(path):
    if not Path(path).is_file():
        raise FileNotFoundError("Randomization profile not found: %s" % (path,))
    return Path(path)


def _source_label(path):
    try:
        return Path(path).resolve().relative_to(PROJECT).as_posix()
    except ValueError:
        return str(Path(path).resolve()).replace("\\", "/")