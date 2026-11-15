"""Check, benchmark, train and independently evaluate local navigation PPO."""

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import shutil
import time

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from rl.environment import LocalNavigationEnv
from navigation_core.local_policy import LocalObservationEncoder

import numpy as np


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def environment_factory(task, baseline=False, seconds=None):
    def make():
        return LocalNavigationEnv(task=task, baseline=baseline, max_seconds=seconds)
    return make


def check(args):
    from gymnasium.utils.env_checker import check_env as gym_check
    from stable_baselines3.common.env_checker import check_env as sb3_check
    env = LocalNavigationEnv(task=args.task)
    try:
        gym_check(env, skip_render_check=True)
        sb3_check(env)
        actions = np.random.default_rng(9).uniform(-1, 1, size=(80, 2)).astype(np.float32)
        traces = []
        for _ in range(2):
            observation, _ = env.reset(seed=123)
            trace = [observation.copy()]
            for action in actions:
                observation, reward, terminated, truncated, _ = env.step(action)
                trace.append(np.r_[observation, reward, terminated, truncated])
                if terminated or truncated:
                    break
            traces.append(trace)
        repeatable = all(np.array_equal(a, b) for a, b in zip(*traces)) and len(traces[0]) == len(traces[1])
        assert repeatable
        env.reset(seed=123)
        assert env.now == 0 and env.executor.collisions == 0 and not env.executor.pending_commands
        assert np.all(env.executor.velocity == 0)
        second = LocalNavigationEnv(task='obstacle')
        try:
            second.reset(seed=123)
            assert not np.shares_memory(env.model.geom_pos, second.model.geom_pos)
        finally:
            second.close()
        # Exercise an actual local task using the existing planner.
        env.close()
        env = LocalNavigationEnv(task=args.task, baseline=True)
        env.reset(seed=123)
        done = False
        while not done:
            _, _, terminated, truncated, info = env.step(np.array([-1, 0], dtype=np.float32))
            done = terminated or truncated
        result = dict(passed=True, deterministic_reset=repeatable, independent_models=True,
                      observation_dimension=LocalObservationEncoder.dimension,
                      baseline_episode=info['episode_metrics'])
        write_json(args.out / 'environment_check.json', result)
        print(json.dumps(result), flush=True)
    finally:
        env.close()


def benchmark(args):
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    results = []
    for count in (1, 2, 4):
        cls = DummyVecEnv if count == 1 else SubprocVecEnv
        factories = [environment_factory(args.task) for _ in range(count)]
        kwargs = {} if count == 1 else {'start_method': 'spawn'}
        env = cls(factories, **kwargs)
        try:
            env.seed(200)
            env.reset()
            resets = env.get_attr('reset_duration_s')
            started = time.perf_counter()
            for _ in range(args.steps):
                env.step(np.tile(np.array([0.0, 0.1], dtype=np.float32), (count, 1)))
            elapsed = time.perf_counter() - started
            speed = count * args.steps / elapsed
            entry = dict(environments=count, transitions=count * args.steps, wall_s=elapsed, steps_per_s=speed,
                         reset_mean_s=float(np.mean(resets)), reset_p95_s=float(np.percentile(resets, 95)),
                         estimated_sampling_hours={str(n): n / speed / 3600 for n in (100000, 500000, 1000000)},
                         component_timings=env.get_attr('timings'))
            results.append(entry)
            write_json(args.out / 'benchmark.json', results)
            print(json.dumps(entry), flush=True)
        finally:
            env.close()


def train(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    torch.set_num_threads(1)
    args.out.mkdir(parents=True, exist_ok=True)
    factories = [environment_factory(args.task) for _ in range(args.jobs)]
    def monitored(factory):
        def make():
            return Monitor(factory())
        return make
    factories = [monitored(factory) for factory in factories]
    cls = DummyVecEnv if args.jobs == 1 else SubprocVecEnv
    env = cls(factories, **({} if args.jobs == 1 else {'start_method': 'spawn'}))
    env.seed(args.seed)
    try:
        if args.model:
            model = PPO.load(str(args.model), env=env, device='cpu')
            model.set_random_seed(args.seed)
        else:
            from rl.features import NavigationFeatures
            model = PPO('MlpPolicy', env, seed=args.seed, device='cpu', n_steps=256, batch_size=128,
                        n_epochs=5, learning_rate=3e-4, gamma=0.995, verbose=1,
                        policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0,
                                           features_extractor_class=NavigationFeatures))
        sources = list((PROJECT / 'rl').glob('*.py')) + list((PROJECT / 'controllers/sensor_capture/navigation_core').rglob('*.py'))
        sources.extend([Path(__file__), PROJECT / 'mujoco_navigation.py', PROJECT / 'requirements-rl.txt'])
        sources.extend((PROJECT / 'config').glob('*.json'))
        hashes = {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
        for source in sources:
            target = args.out / 'source_snapshot' / source.relative_to(PROJECT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        metadata = dict(task=args.task, seed=args.seed, requested_timesteps=args.steps, jobs=args.jobs,
                        parent_model=None if not args.model else str(args.model.resolve()),
                        versions={name: importlib.metadata.version(name) for name in ('gymnasium', 'stable-baselines3', 'torch', 'mujoco', 'numpy')},
                        source_hashes=hashes, encoder_dimension=LocalObservationEncoder.dimension,
                        observation_normalization='fixed clipped scales in LocalObservationEncoder',
                        action_mapping='vx=(a0+1)*0.5*configured_max; omega=a1*configured_max',
                        truth_usage='training path progress, success verification, collision and independent metrics')
        write_json(args.out / 'training_manifest.json', metadata)
        started = time.perf_counter()
        callback = CheckpointCallback(save_freq=max(1, 4096 // args.jobs), save_path=str(args.out / 'checkpoints'))
        model.learn(total_timesteps=args.steps, callback=callback)
        model.save(str(args.out / 'policy.zip'))
        metadata.update(actual_timesteps=model.num_timesteps, wall_s=time.perf_counter() - started,
                        model_sha256=hashlib.sha256((args.out / 'policy.zip').read_bytes()).hexdigest())
        write_json(args.out / 'training_manifest.json', metadata)
        print('TRAINING_FINISHED ' + json.dumps(metadata), flush=True)
    finally:
        env.close()


def evaluate(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    torch.set_num_threads(1)
    count = args.jobs
    factories = [environment_factory(args.task, baseline=args.model is None) for _ in range(count)]
    cls = DummyVecEnv if count == 1 else SubprocVecEnv
    model = None if args.model is None else PPO.load(str(args.model), device='cpu')
    results = []
    report_path = args.out / 'evaluation.json'
    model_hash = None if args.model is None else hashlib.sha256(args.model.read_bytes()).hexdigest()
    if args.resume and report_path.exists():
        existing = json.loads(report_path.read_text(encoding='utf-8'))
        existing_model = existing.get('model')
        expected_model = None if args.model is None else args.model.resolve()
        actual_model = None if existing_model in (None, 'None') else Path(existing_model).resolve()
        if existing.get('task') != args.task or actual_model != expected_model:
            raise ValueError('Saved evaluation belongs to a different task or model.')
        if existing.get('model_sha256', model_hash) != model_hash:
            raise ValueError('Saved evaluation model checksum differs.')
        results = existing.get('episodes', [])
        seeds = [entry['seed'] for entry in results]
        if len(seeds) != len(set(seeds)) or not set(seeds).issubset(range(args.seed, args.seed + args.episodes)):
            raise ValueError('Saved evaluation seeds differ from the requested range.')
    completed_seeds = {entry['seed'] for entry in results}
    env = cls(factories, **({} if count == 1 else {'start_method': 'spawn'}))
    try:
        for base_seed in range(args.seed, args.seed + args.episodes, count):
            pending = {index for index in range(min(count, args.seed + args.episodes - base_seed))
                       if base_seed + index not in completed_seeds}
            if not pending:
                continue
            env.seed(base_seed)
            observation = env.reset()
            if base_seed == args.seed and 0 in pending:
                env.env_method('start_recording', str(args.out / ('episode_%d' % base_seed)), indices=0)
            inference_times = []
            batched_cycle_times = []
            while pending:
                cycle_started = time.perf_counter()
                action = np.tile(np.array([-1, 0], dtype=np.float32), (count, 1))
                inference_started = time.perf_counter()
                if model is not None:
                    action, _ = model.predict(observation, deterministic=True)
                inference_times.append(time.perf_counter() - inference_started)
                observation, _, done, infos = env.step(action)
                batched_cycle_times.append(time.perf_counter() - cycle_started)
                for index in list(pending):
                    if done[index]:
                        result = dict(seed=base_seed + index, **infos[index]['episode_metrics'])
                        result['batch_inference_p95_s'] = float(np.percentile(inference_times, 95))
                        result['batched_cycle_p95_s'] = float(np.percentile(batched_cycle_times, 95))
                        results.append(result)
                        completed_seeds.add(result['seed'])
                        pending.remove(index)
                        write_json(report_path, dict(task=args.task, model=str(args.model),
                                                    model_sha256=model_hash, episodes=results))
            print('EVALUATED %d/%d' % (len(results), args.episodes), flush=True)
        successes = sum(r['success'] for r in results)
        n = len(results)
        p, z = successes / n, 1.96
        center = (p + z*z/(2*n)) / (1 + z*z/n)
        half = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / (1 + z*z/n)
        report = dict(task=args.task, model=None if args.model is None else str(args.model.resolve()),
                      model_sha256=None if args.model is None else hashlib.sha256(args.model.read_bytes()).hexdigest(),
                      episodes=results, aggregate=dict(successes=successes, episodes=n, success_rate=p,
                          success_rate_wilson_95=[max(0.0, center-half), min(1.0, center+half)],
                          collisions=sum(r['collisions'] for r in results),
                          interventions=sum(r['interventions'] for r in results),
                          success_elapsed_mean_s=float(np.mean([r['elapsed_s'] for r in results if r['success']])) if successes else None))
        write_json(args.out / 'evaluation.json', report)
        print(json.dumps(report['aggregate']), flush=True)
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['check', 'benchmark', 'train', 'evaluate'])
    parser.add_argument('--task', choices=['tracking', 'obstacle', 'dynamic', 'obstacle_bypass'], default='tracking')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=256)
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--seed', type=int, default=10000)
    parser.add_argument('--episodes', type=int, default=30)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--resume', action='store_true', help='Continue saved evaluation episodes with matching seeds and model.')
    args = parser.parse_args()
    if min(args.steps, args.jobs, args.episodes) < 1:
        parser.error('steps, jobs and episodes must be positive')
    args.out = args.out.resolve()
    {'check': check, 'benchmark': benchmark, 'train': train, 'evaluate': evaluate}[args.mode](args)


if __name__ == '__main__':
    main()
