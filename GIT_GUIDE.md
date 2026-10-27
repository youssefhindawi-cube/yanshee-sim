# Git guide for this repo (youssefhindawi-cube)

Quick reference for committing and pushing to **youssefhindawi-cube/yanshee-sim** from this laptop,
which also has two other GitHub accounts. Keep it open when you work in the repo.

---

## The short version

```bash
cd "/home/youssef/Downloads/humanoid_robot/yanshee_sim"
git-whoami                     # 1. confirm you are the cube account
git status                     # 2. see what changed
gsave "Describe the change"    # 3. add + commit + push in one go
```

Or the same with plain git, step by step:

```bash
git add -A
git commit -m "Describe the change"
git push
```

This repo already pushes to the cube account, so the plain git commands are safe here.

---

## 1. How the three accounts are kept apart

Each account has its own **SSH key** and **SSH host alias** (in `~/.ssh/config`). Every repo stores
**its own** commit name and email (`git config` without `--global`). The global git config stays on
the nodogoro account, so other repos are not affected.

| Account name for the helpers | GitHub user | SSH host alias | Key file | Commits as |
|---|---|---|---|---|
| `cube` | youssefhindawi-cube | `github-cube` | `~/.ssh/github_yanshee` | yousef.hendawe@cubeconsult.org |
| `personal` | YoussefHindi | `github-personal` | `~/.ssh/personal_git` | youssefhindawi@hotmail.com |
| `nodogoro` (default) | youssefhindawi-nodogoro | `github.com` | `~/.ssh/id_ed25519` | youssef.hindawi@nodogoro.com |

The **host part of the remote URL picks the account**:

```
git@github-cube:youssefhindawi-cube/yanshee-sim.git      -> pushes as youssefhindawi-cube
git@github-personal:YoussefHindi/some-repo.git           -> pushes as YoussefHindi
git@github.com:youssefhindawi-nodogoro/some-repo.git     -> pushes as youssefhindawi-nodogoro
```

This repo's remote is `git@github-cube:youssefhindawi-cube/yanshee-sim.git`.

---

## 2. One-time setup: enable the helper commands

The helpers live in `~/.git-accounts.sh`. Load them in every new terminal by running this once:

```bash
echo '[ -f ~/.git-accounts.sh ] && . ~/.git-accounts.sh' >> ~/.bashrc
source ~/.bashrc
```

Check it worked:

```bash
type git-whoami      # should print "git-whoami is a function"
```

---

## 3. Helper commands

| Command | What it does | Example |
|---|---|---|
| `git-whoami` | Shows the commit name/email for this repo, the `origin` URL, and (for an SSH GitHub remote) the account GitHub actually sees. | `git-whoami` |
| `git-as ACCOUNT` | Makes the **current repo** commit and push as that account: sets the repo-local name/email and, if `origin` is a GitHub URL, rewrites it to `git@<account alias>:OWNER/REPO.git`. | `git-as cube` |
| `gsave "msg"` | `git add -A` + `git commit -m "msg"` + `git push` | `gsave "Add bow motion"` |
| `git-clone-as ACCOUNT OWNER/REPO [DIR]` | Clones through the right account and sets its identity. | `git-clone-as cube youssefhindawi-cube/yanshee-sim` |
| `git-new-remote ACCOUNT OWNER/REPO` | Inside a repo: replaces `origin` with the new, empty GitHub repo and sets the identity. | `git-new-remote cube youssefhindawi-cube/new-repo` |

`ACCOUNT` is `cube`, `personal` or `nodogoro`. `git-whoami`, `git-as` and `git-new-remote` must run
inside a git repo (a subfolder is fine). Outside one they print `not inside a git repository` and
return 1. Otherwise they return 0.

Expected `git-whoami` output in this repo:

```
commit as : youssefhindawi-cube <yousef.hendawe@cubeconsult.org>
origin    : git@github-cube:youssefhindawi-cube/yanshee-sim.git
push as   : youssefhindawi-cube
```

If any line shows another account, run `git-as cube` and check again.

The `push as` check asks GitHub over SSH (`ssh -T`) and waits up to 25 s. Other things it can show:

| `push as` line | Meaning |
|---|---|
| `(no origin remote)` | The repo has no `origin` yet. Use `git-new-remote`. |
| `(https remote: uses a token, not an SSH key; run git-as ACCOUNT to switch to SSH)` | `origin` is `https://github.com/...`, so the SSH keys do not apply. `git-as cube` switches it to `git@github-cube:...`. |
| `(not a GitHub remote; not checked)` | `origin` is on another host (GitLab, a local path, ...). It is never contacted. |
| `(unknown: no answer from HOST within 25 s or key rejected; try ssh -T git@HOST)` | SSH timed out or the key was refused. See Troubleshooting. |

If the `commit as` line ends with `(global default, not set for this repo)`, the repo has no
identity of its own and would commit as the global (nodogoro) account. Run `git-as ACCOUNT`.

**Which remotes `git-as` rewrites:** only GitHub ones. That covers `git@github.com:...`,
`git@github-cube:...` and the other `github-*` aliases, `ssh://git@github.com[:port]/...`,
`https://[user@]github.com/...` (also `www.github.com` and `http://`), and `git://github.com/...`.
A trailing `/` and a missing `.git` are fine. Any other host is left unchanged, with the message
`origin '...' is not a GitHub remote; left it unchanged (only the identity was set)`. A GitHub URL
that is not `OWNER/REPO` (for example `https://github.com/owner`) is refused with
`... is not an OWNER/REPO GitHub URL; nothing changed`, and the command returns 1.

---

## 4. Everyday workflow in this repo

### Commit and push a change

```bash
cd "/home/youssef/Downloads/humanoid_robot/yanshee_sim"
git pull                          # get the latest first (if you work from more than one machine)
python3 tests/test_models.py      # optional: make sure nothing broke
python3 tests/test_mirror.py
git status                        # review what changed
git diff                          # see the exact changes
git add -A                        # stage everything (or: git add path/to/file)
git commit -m "Add bow motion"    # commit with a clear message
git push                          # send to GitHub
```

### Commit only some files

```bash
git add motions/bow.json README.md
git commit -m "Add bow motion and document it"
git push
```

### Check what is on GitHub vs local

```bash
git fetch
git status                        # "ahead by N" = commits not pushed yet
git log --oneline -5              # last 5 local commits
git log --oneline origin/main -5  # last 5 commits on GitHub
```

### Undo things

| Situation | Command |
|---|---|
| Unstage a file (keep the changes) | `git restore --staged FILE` |
| Throw away uncommitted changes to a file | `git restore FILE` (cannot be undone) |
| Fix the last commit's message (not pushed yet) | `git commit --amend -m "New message"` |
| Add a forgotten file to the last commit (not pushed yet) | `git add FILE && git commit --amend --no-edit` |
| Undo the last commit but keep its changes (not pushed yet) | `git reset --soft HEAD~1` |

Don't amend or reset commits that are **already pushed**. Make a new commit instead.

### Work on a branch (for bigger changes)

```bash
git switch -c walking-gait         # create and switch to a new branch
# ...edit, then commit as usual...
git push -u origin walking-gait    # first push of the branch
git switch main                    # back to main
git merge walking-gait             # bring the branch in
git push
```

---

## 5. Git LFS (large files)

`source_model/robot.json` (62 MB) is stored with **Git LFS**. Without it, GitHub warns above
50 MB and refuses above 100 MB.

```bash
git lfs install                    # once per machine, before cloning
git lfs ls-files                   # list files stored in LFS
git lfs pull                       # download LFS files if you only see small pointer files
git lfs checkout                   # restore them from the local LFS store (no network) if already downloaded
git lfs track "*.bag"              # store a new kind of large file in LFS (commit .gitattributes too)
```

Free GitHub plan: 1 GB of LFS storage and 1 GB of download bandwidth per month. Put large binaries
(videos, rosbags, datasets) in LFS or keep them out of the repo.

---

## 6. Using the cube account for other repos

**New repo:** create it empty (no README) on github.com while logged in as youssefhindawi-cube, then:

```bash
cd my-project
git init -b main
git-new-remote cube youssefhindawi-cube/my-project
git add -A && git commit -m "Initial commit"
git push -u origin main
```

**Existing clone that should push as cube:**

```bash
cd some-repo
git-as cube
```

**Clone:**

```bash
git-clone-as cube youssefhindawi-cube/my-project
```

Without the helpers, use the alias in the URL and set the identity yourself:

```bash
git clone git@github-cube:youssefhindawi-cube/my-project.git
cd my-project
git config user.name  "youssefhindawi-cube"
git config user.email "yousef.hendawe@cubeconsult.org"
```

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| `git-whoami: command not found` | Do the one-time setup in section 2, or run `source ~/.git-accounts.sh`. |
| `not inside a git repository` | `cd` into the repo first. For a new project, run `git init -b main` first (section 6). |
| Commit shows the wrong author | `git-as cube` (or `git config user.email ...`), then fix the last unpushed commit with `git commit --amend --reset-author --no-edit`. |
| `Permission denied (publickey)` or "Repository not found" | The remote uses the wrong host. Check `git remote -v`; it must say `github-cube`. Fix with `git-as cube`. Test with `ssh -T git@github-cube`, which should answer "Hi youssefhindawi-cube!". |
| `git-as` says `is not a GitHub remote; left it unchanged` | `origin` points at another host, so only the name/email were set. If it should be on GitHub, run `git-new-remote cube OWNER/REPO`. |
| `rejected ... (fetch first)` / `non-fast-forward` | GitHub has commits you don't have. Run `git pull --rebase` (or `git pull`), then `git push`. |
| `robot.json` is ~130 bytes (`git lfs ls-files` shows `-` instead of `*`) | It is the Git LFS pointer. Run `git lfs checkout source_model/robot.json` if the object is already downloaded, otherwise `git lfs install && git lfs pull`. |
| `push as : (unknown: no answer ...)` | GitHub's SSH can take 5–15 s from this network, and `git-whoami` gives up after 25 s. Retry, or run `ssh -T git@github-cube` to see the actual error. |
| Need to see the SSH setup | `grep -A5 "Host github-cube" ~/.ssh/config` |

### When a force-push is needed

Only needed after rewriting history that is already on GitHub (for example `git lfs migrate`, or
amending a pushed commit). Prefer `--force-with-lease`: it refuses if GitHub has commits you haven't seen.

```bash
git fetch
git push --force-with-lease origin main
```

Never force-push a branch other people have cloned without telling them.
