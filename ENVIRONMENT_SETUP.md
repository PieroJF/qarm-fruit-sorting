# Claude Code Environment Setup — Migration Guide

Reproduce the exact Claude Code working environment from Piero's machine on any Windows PC.

---

## 1. Claude Code Installation

```bash
# Install Claude Code (requires Node.js 18+)
npm install -g @anthropic-ai/claude-code

# Login
claude login
```

---

## 2. Global Settings (`~/.claude/settings.json`)

Create or replace `C:\Users\<USER>\.claude\settings.json`:

```json
{
  "permissions": {
    "defaultMode": "bypassPermissions"
  },
  "enabledPlugins": {
    "superpowers@claude-plugins-official": true,
    "context7@claude-plugins-official": true,
    "code-simplifier@claude-plugins-official": true
  },
  "autoUpdatesChannel": "latest",
  "skipDangerousModePermissionPrompt": true,
  "attribution": {
    "commit": "",
    "pr": ""
  }
}
```

### What each setting does

| Setting | Value | Purpose |
|---------|-------|---------|
| `permissions.defaultMode` | `"bypassPermissions"` | No permission prompts — Claude executes tools immediately |
| `enabledPlugins.superpowers` | `true` | Brainstorming, TDD, debugging, code review, plan writing/execution skills |
| `enabledPlugins.context7` | `true` | Fetches live library/framework documentation from context7 |
| `enabledPlugins.code-simplifier` | `true` | Code quality review and simplification skill |
| `autoUpdatesChannel` | `"latest"` | Always use newest Claude Code version |
| `skipDangerousModePermissionPrompt` | `true` | Skip the bypass-permissions confirmation dialog |
| `attribution.commit` | `""` | **No Co-Authored-By in commits** — Claude leaves zero trace in git |
| `attribution.pr` | `""` | **No "Generated with Claude Code" in PRs** |

---

## 3. Plugins Installed

These install automatically from the `enabledPlugins` setting above. If they don't, install manually:

```bash
# In Claude Code, run:
/install superpowers@claude-plugins-official
/install context7@claude-plugins-official
/install code-simplifier@claude-plugins-official
```

### Plugin descriptions

| Plugin | Version | What it provides |
|--------|---------|------------------|
| **superpowers** | 5.0.7 | Skills: brainstorming, writing-plans, executing-plans, TDD, systematic-debugging, code-review (requesting + receiving), verification-before-completion, subagent-driven-development, using-git-worktrees, dispatching-parallel-agents, finishing-a-development-branch |
| **context7** | latest | MCP server that fetches up-to-date library docs (React, Next.js, Quanser SDK, etc.) on demand |
| **code-simplifier** | 1.0.0 | Skill: review recently changed code for reuse, quality, and efficiency |

---

## 4. MCP Servers

### 4.1 MATLAB MCP Server (project-scoped)

Download `matlab-mcp-core-server.exe` and place it at:
```
C:\Users\<USER>\.claude\tools\matlab-mcp-core-server.exe
```

This is registered per-project in `.claude.json` (see section 6). It provides:
- `evaluate_matlab_code` — run MATLAB code from Claude
- `run_matlab_file` — execute .m scripts
- `check_matlab_code` — syntax check
- `detect_matlab_toolboxes` — list installed toolboxes

### 4.2 Sequential Thinking (global, currently broken on this machine)

Registered globally but depends on npx which has a broken npm installation. On a clean machine:

```json
// In ~/.claude.json under "mcpServers":
{
  "sequential-thinking": {
    "type": "stdio",
    "command": "npx",
    "args": ["-y", "@modelcontextprotocol/server-sequential-thinking"],
    "env": {}
  }
}
```

### 4.3 Context7 (via plugin)

Provided automatically by the context7 plugin. No manual config needed.

---

## 5. Project Memory Files

Create directory: `C:\Users\<USER>\.claude\projects\<SANITIZED-PROJECT-PATH>\memory\`

The sanitized path replaces `\` and `:` with `-`. For this project:
```
C:\Users\<USER>\.claude\projects\<SANITIZED>\memory\
```

### MEMORY.md (index)

```markdown
- [FruitSorting project state](project_fruitsorting.md) — QArm + D415 sort, Simulink facade over Python, 5 sim slices passing, deadline 2026-05-01.
- [No QUARC — Quanser SDK only](feedback_no_quarc.md) — Teammates confirmed QUARC not needed; stale docs say otherwise, treat as "verify SDK".
```

### project_fruitsorting.md

```markdown
---
name: FruitSorting project state
description: Applied Robotics final — QArm + D415 fruit sorting, Simulink facade over Python core. Deadline 2026-05-01 14:00.
type: project
---

**Project:** Sort 14 fruits (6 strawberry -> A, 3 banana -> B, 5 tomato -> C) with Quanser QArm 4-DOF + Intel RealSense D415. Two modes required: autonomous + remote teleop. University of Birmingham Applied Robotics final.

**Team:** Piero Flores, Zihen Huang, Ran Zhang, Yichang Chao.

**Architecture (locked):** Simulink is the facade, Python does the real work. MATLAB Function blocks call `py.*` via `coder.extrinsic` — no TCP/socket shim. Python core in `python/` is untouched; wrappers live in `matlab_facade/`.

**Current state (2026-04-16):** Python core 100% complete and offline-validated. All 5 Simulink slices PASS in simulation mode. Hardware verified: QArm reads/writes OK, D415 captures frames. teach_points.py and calibrate_hand_eye.py built and tested. Gripper safety implemented (GRIP_CLOSE_CMD=0.65, GRIP_OPEN_CMD=0.10, no idle spamming, actual readback).

**Why:** ~15 days to deadline; lab calibration is the critical-path bottleneck.

**How to apply:** Resume by reading `PROGRESS.md` + `PROJECT_CONTEXT.md`. Never rewrite Python core — only wrap it.
```

### feedback_no_quarc.md

```markdown
---
name: No QUARC — Quanser SDK only
description: User's teammates confirmed QUARC is not required; only the Quanser SDK (already installed) is needed for this project.
type: feedback
---

**Rule:** Do NOT plan for or require QUARC installation. The project runs on the Quanser SDK for Windows alone. QUARC is explicitly not needed.

**Why:** Teammates confirmed (2026-04-15). All hardware I/O routes through `py.qarm_driver.QArmDriver` (Quanser SDK Python bindings). Simulink never touches the arm directly via QUARC blocks.

**How to apply:**
- Replace "QUARC" with "Quanser SDK" in docs when touched.
- Do not suggest installing QUARC.
- The `qarm_mode=1` hardware path in `py_qarm_io.m` only needs the Quanser SDK Python wheels.
```

---

## 6. Project-Level MCP Config

If your teammate's project needs the MATLAB MCP server, create or update `~/.claude.json` to include in the project entry:

```json
{
  "mcpServers": {
    "matlab": {
      "type": "stdio",
      "command": "C:/Users/<USER>/.claude/tools/matlab-mcp-core-server.exe",
      "args": [
        "--matlab-display-mode=nodesktop",
        "--initial-working-folder=<PATH-TO-FinalProject_FruitSorting>"
      ],
      "env": {}
    }
  }
}
```

---

## 7. VS Code Settings

Create `.vscode/settings.json` in the project root (already committed to the repo):

```json
{
  "python.defaultInterpreterPath": "C:\\Python313\\python.exe",
  "python.terminal.activateEnvironment": false,
  "terminal.integrated.env.windows": {
    "PYTHON": "C:\\Python313\\python.exe"
  },
  "python-envs.defaultEnvManager": "ms-python.python:system"
}
```

Adjust the Python path if Python 3.13 is installed elsewhere on the new machine.

---

## 8. Software Prerequisites

| Software | Version | Path (default) | Notes |
|----------|---------|----------------|-------|
| Python | 3.13.x | `C:\Python313\` | **Must be 3.13** — Quanser SDK wheels are version-specific |
| MATLAB | R2025a | `C:\Program Files\MATLAB\R2025a` | With Simulink, Simscape, Stateflow |
| Quanser SDK | 2026.1.21 | `C:\Program Files\Quanser\Quanser SDK\` | NOT QUARC |
| Node.js | 18+ | `C:\Program Files\nodejs\` | For Claude Code and npx-based MCPs |
| Git | any | PATH | For version control |
| VS Code | any | — | Optional but recommended |

### Python packages

```bash
C:\Python313\python.exe -m pip install numpy opencv-python reportlab
```

### Quanser SDK Python wheels

```bash
cd "C:\Program Files\Quanser\Quanser SDK\python"
install_quanser_python_api.bat
```

### Verify everything

```bash
C:\Python313\python.exe -c "import numpy, cv2, quanser.hardware, quanser.multimedia, quanser.devices; print('all OK')"
```

---

## 9. Quick Setup Script (copy-paste into PowerShell)

```powershell
# 1. Create Claude config directory
$claudeDir = "$env:USERPROFILE\.claude"
New-Item -ItemType Directory -Force -Path "$claudeDir\tools"

# 2. Write global settings
@'
{
  "permissions": {
    "defaultMode": "bypassPermissions"
  },
  "enabledPlugins": {
    "superpowers@claude-plugins-official": true,
    "context7@claude-plugins-official": true,
    "code-simplifier@claude-plugins-official": true
  },
  "autoUpdatesChannel": "latest",
  "skipDangerousModePermissionPrompt": true,
  "attribution": {
    "commit": "",
    "pr": ""
  }
}
'@ | Set-Content "$claudeDir\settings.json"

# 3. Install Python dependencies
& C:\Python313\python.exe -m pip install numpy opencv-python reportlab

# 4. Install Quanser SDK wheels (if SDK is installed)
if (Test-Path "C:\Program Files\Quanser\Quanser SDK\python\install_quanser_python_api.bat") {
    Push-Location "C:\Program Files\Quanser\Quanser SDK\python"
    .\install_quanser_python_api.bat
    Pop-Location
}

# 5. Clone the repo
git clone https://github.com/PieroJF/qarm-fruit-sorting.git
cd qarm-fruit-sorting

# 6. Verify Python environment
& C:\Python313\python.exe python\validate_python.py

Write-Host "Setup complete. Run 'claude' in the project directory to start."
```

---

## 10. Key Behavioral Rules (what Claude "knows")

These are enforced via memory files and conversation history:

1. **Never add Co-Authored-By** — attribution is disabled globally
2. **Never suggest QUARC** — only Quanser SDK is needed
3. **Never rewrite Python core** — only wrap it via Simulink facade
4. **Gripper safety** — never command gripper to 0.0 (open) or 1.0 (closed); use GRIP_OPEN_CMD=0.10, GRIP_CLOSE_CMD=0.65
5. **No idle command spamming** — QArm position-mode PID holds setpoints on its own
6. **Use Python 3.13** — not 3.14 or any other version (SDK wheels are version-specific)
7. **Simulink is the facade, Python does the real work** — architecture is locked

---

## 11. Files NOT in the Repo (session-specific)

These files are generated locally and excluded by `.gitignore`:

| File | Purpose | How to regenerate |
|------|---------|-------------------|
| `teach_points.json` | Recorded workspace waypoints | Run `teach_points.py`, save points with `n` |
| `calibration.json` | Hand-eye calibration T matrix | Run `calibrate_hand_eye.py` after recording 6+ `cal_*` points |
| `logs/*.log` | Robot trace logs | Generated automatically by `teach_points.py` |
| `figures/live_*.png` | Camera captures | Generated by camera scripts |
| `figures/preview_*.png` | Camera preview snapshots | Generated by `camera_preview.py` |
