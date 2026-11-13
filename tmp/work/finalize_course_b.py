import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

project = Path('F:/airace/lite2_sim')
sys.path.insert(0, str(project / 'tools'))
from build_navigation_acceptance import _course_evidence

config_path = project / 'config/ruins_reference.json'
config = json.loads(config_path.read_text(encoding='utf-8'))
directories = sorted((project / 'runs').glob('mujoco_20261113_150640*'))
assert len(directories) == 3
evidence = _course_evidence(directories)
hashes = []
for path, entry in zip(directories, evidence):
    snapshot = json.loads((path / 'config_snapshot.json').read_text(encoding='utf-8'))
    assert snapshot['course'] == config['course']
    summary = json.loads((path / 'navigation_summary.json').read_text(encoding='utf-8'))
    with (path / 'velocity_commands.csv').open(encoding='utf-8', newline='') as source:
        commands = list(csv.DictReader(source))
    events = [json.loads(line) for line in (path / 'course_events.jsonl').read_text().splitlines() if line]
    entry.update(
        seed=summary['random_seed'],
        goal_error_m=summary['evaluation_only']['goal_error_m'],
        collision_events=len(json.loads((path / 'collision_events.json').read_text())),
        reverse_command_count=sum(float(row['linear_x_m_s']) < -0.001 for row in commands),
        command_count=len(commands),
        recovery_event_count=sum(event['event'].startswith('recovery_') for event in events),
    )
    assert entry['passed'] and all(stage['passed'] for stage in entry['stages'])
    assert entry['collision_events'] == entry['reverse_command_count'] == 0
    hashes.append(json.loads((path / 'source_hashes.json').read_text()))
assert all(value == hashes[0] for value in hashes)
video = Path('F:/airace/plan/videos/nav_full_course_headfirst_20261113.mp4')
assert video.is_file() and video.stat().st_size > 0
record = dict(
    passed=True,
    generated_at_utc=datetime.now(timezone.utc).isoformat(timespec='seconds'),
    scope='mujoco_velocity_level_full_course',
    course_variant='B',
    decision='User selected B. Keep dead_end_exit exit checkpoint and remove its reverse requirement.',
    config_path=str(config_path),
    config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
    identical_source_hashes=True,
    course=sorted(evidence, key=lambda entry: entry['seed']),
    regression_tests=dict(count=130, passed=True, log='F:/airace/tmp/work/course_B_tests.log'),
    video=dict(path=str(video), source_run='runs/mujoco_20261113_150640_c2ad05',
               sha256=hashlib.sha256(video.read_bytes()).hexdigest(), fps=12, sample_step_s=0.25),
    assumptions=dict(locomotion='velocity-level mocap fixture with held joint pose',
                     physical_gait_validated=False, hardware_validation='pending'),
)
(project / 'mujoco/course_acceptance.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
print(json.dumps({'passed': True, 'seeds': [entry['seed'] for entry in record['course']]}))
