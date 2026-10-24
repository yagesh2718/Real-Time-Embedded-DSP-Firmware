#!/usr/bin/env python3
"""
Single End-to-End Orchestration Script:
1. Configures and builds the project via CMake.
2. Executes the automated test suite (dsp_firmware_tests).
3. Executes the firmware simulator and benchmark runner (dsp_firmware_sim).
4. Generates high-resolution visualization plots from exported CSV telemetry.
"""

import os
import subprocess
import sys

def run_cmd(cmd, cwd=None):
    print(f"\n[EXEC] {' '.join(cmd) if isinstance(cmd, list) else cmd}")
    res = subprocess.run(cmd, cwd=cwd, shell=True)
    if res.returncode != 0:
        print(f"[ERROR] Command failed with exit code {res.returncode}")
        sys.exit(res.returncode)

def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print("========================================================================")
    print("   END-TO-END WORKFLOW: REAL-TIME EMBEDDED DSP FIRMWARE SIMULATOR       ")
    print("========================================================================")

    # 1. CMake Configure & Build
    build_dir = os.path.join(repo_root, "build")
    run_cmd(f'cmake -B "{build_dir}" -G Ninja', cwd=repo_root)
    run_cmd(f'cmake --build "{build_dir}"', cwd=repo_root)

    # 2. Run Automated Unit Tests
    test_exe = os.path.join(build_dir, "dsp_firmware_tests.exe" if os.name == 'nt' else "dsp_firmware_tests")
    run_cmd(f'"{test_exe}"', cwd=repo_root)

    # 3. Run Simulation & Benchmarks
    sim_exe = os.path.join(build_dir, "dsp_firmware_sim.exe" if os.name == 'nt' else "dsp_firmware_sim")
    run_cmd(f'"{sim_exe}"', cwd=repo_root)

    # 4. Generate Spectral and Benchmark Plots
    plot_script = os.path.join(repo_root, "scripts", "generate_plots.py")
    run_cmd(f'python "{plot_script}"', cwd=repo_root)

    print("\n========================================================================")
    print("   WORKFLOW COMPLETED SUCCESSFULLY!                                      ")
    print(f"   - Signal Telemetry CSV:  {os.path.join(repo_root, 'signal_simulation.csv')}")
    print(f"   - Benchmark CSV:         {os.path.join(repo_root, 'benchmark_results.csv')}")
    print(f"   - Signal Analysis Plot:  {os.path.join(repo_root, 'signal_analysis.png')}")
    print(f"   - Benchmark Plot:        {os.path.join(repo_root, 'benchmark_analysis.png')}")
    print("========================================================================\n")

if __name__ == '__main__':
    main()
