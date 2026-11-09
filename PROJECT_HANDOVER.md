# Handover: the Yanshee humanoid project

This is the project-wide index, copied from the folder above the two repositories on the development laptop.

Index for whoever takes over the UBTECH Yanshee work at Cube Consultants. Written 9 Nov 2026 for the exhibition on
**Monday 7 December 2026**. Each repository has its own detailed handover:

- `HANDOVER.md` in this repository: simulator, web GUI, walking, get-up, the robot link, every hardware measurement and
  what is still open on the robot.
- `HANDOVER.md` in the sibling repository `yanshee-llm-model` (`llm_model/` on the development laptop): Q's brain, Egyptian-Arabic speech recognition, voices, the robot's microphone and
  speakers over the network.

Read both before the first robot session.

## The two repositories and how they meet

| Repository | GitHub | Purpose | State at handover |
|---|---|---|---|
| `yanshee_sim` | `youssefhindawi-cube/yanshee-sim` | Move the robot safely: sim first, then the real robot through a safety gate | 201 tests pass; walking, turns, watchdog, get-up and speed ramp verified on the robot on 2 Nov 2026 |
| `llm_model` | `youssefhindawi-cube/yanshee-llm-model` | Make the robot hear and answer in Egyptian Arabic | Live pipeline works end to end on the robot's own mic and speakers (4 Nov 2026); brain is keyword matching, no LLM yet |

They are independent today. The planned join is "gestures while Q talks": `llm_model` produces the spoken reply,
`yanshee_sim` plays head and arm motions during it. Both talk to the robot over ssh (`pi` / `raspberry`) and REST
on port 9090, and both find it by MAC `b8:27:eb:20:d3:af` with `yanshee_sim/tools/find_robot.py`. When both run at
once, note that a second full ssh handshake starves the Pi's audio (see `llm_model/HANDOVER.md` §6).

## Milestones

| Milestone | Date | Content |
|---|---|---|
| M1 (protected) | Fri 20 Nov 2026 | Presentation, basic movements, interaction, face recognition of enrolled team members |
| M2 | Fri 27 Nov 2026 | M1 + Q answering spoken or typed questions through the robot |
| M3 (cut first) | Fri 4 Dec 2026 | M2 + replies shown as visuals on a screen next to the robot |

## Other folders on the development laptop (`/home/youssef/Downloads/humanoid_robot/`, next to this repository; not in git)

| Folder | Contents |
|---|---|
| `yanshee robot/` | The official material: manual (`yanshee.pdf`), SDK and examples (`yanshee/`, `YanAPI.py`), the 411-page course, OTA package, and two digests written from them: `YANSHEE_KNOWLEDGE_BASE.md` and `YANSHEE_COURSE_DIGEST.md` (copies also in `yanshee_sim/knowledge/`). |
| `humanoids robots/` | Market research and RFQs for other robots (UBTECH Cruzr, Walker C1, Unitree, AgiBot), a comparison deck and `Cube_Robot_Handover.pdf`. Not needed for the Yanshee show. |
| `cube design/` | STL files of the "Q Robot" shell design. |
| `reports/`, `research_notes/` | Prior-work survey for the longer-term "expert knowledge twin" idea (the Yanshee as capture hardware for a digital twin of a professor). Separate from the exhibition. |

## Planning pages (private artifacts; share from the page's menu)

- Hardware test plan, the running status board: https://claude.ai/artifact/PMgMatZwBzEmuRWtiMM3sC
- Exhibition plan for management: https://claude.ai/artifact/X2cmA5CbkPJYvduR9iYaiz
- Technical plan: https://claude.ai/artifact/377NDYzeH5PsaXg2iiTnCW
- Wi-Fi setup card: https://claude.ai/artifact/BqDEeddY2eqashYtfKQ4Yr
- Handling guide (where to hold the robot, IMU postures): https://claude.ai/artifact/DneKcUR3E36bps9Fqiim8j
- Fall recovery test: https://claude.ai/artifact/WCCUDnLxZ5tTT2AT7L3kjk

## The five rules

1. Someone stands ready to catch the robot during every walk, turn and get-up. Never move it while it charges or
   while nobody is beside it.
2. Smooth hard floor. Face-down get-up fails on mats.
3. Never call the hal write services (`hal_write_sn`, `hal_upgrade`, `hal_servo_calibration`,
   `hal_servo_offset_write`, `hal_shutdown_set`) and never start vision tracking over REST during a show.
4. Show network: a dedicated router on channel 1 or 11, laptop wired, no internet uplink.
5. Never commit `yanshee_sim/robot_dump/` (UBTECH source and office photos) or `llm_model/.env` (API keys).
