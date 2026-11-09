# tv-cast

Tools to put images, videos, and web pages onto a Samsung Smart TV over the local network,
and to have the **Yanshee robot cast a slideshow to the TV by itself**.

Built and verified against a Samsung UA85DU7000UXEG (85" Crystal UHD, 2024 Tizen) at `192.168.89.9`,
with the robot (Raspberry Pi, Raspbian Stretch, Python 3.5) at `192.168.89.80`.

## What works, and what does not

- **DLNA (video and image push) works and is reliable.** The TV plays a video file or shows an image
  that we serve over HTTP and point it at with a UPnP `SetAVTransportURI` + `Play` call.
- **Miracast / screen mirroring does NOT work from the test laptop.** The laptop's Intel Wi-Fi does
  Wi-Fi Direct only on 5 GHz channels 149-161, the TV only on 36-48; they share no 5 GHz channel, and
  the TV refuses the 2.4 GHz fallback, so group formation times out. This is a radio limitation, not a bug.
- **The TV's built-in web browser cannot be launched over the remote API** on this 2024 model; the call is
  accepted but nothing opens. Show a web page by rendering it to an image or a scrolling video instead.
- **The TV's photo viewer is flaky.** Direct image display works but can get stuck after repeated pushes.
  For anything that must be dependable (an exhibition), use the **video** path, which never got stuck.

## Robot casts to the TV (the main use)

The robot streams a prebuilt slideshow video to the TV. It needs no extra packages: the caster is a
single Python-3.5-compatible, standard-library-only script. **The robot's Python is not modified.**

1. One time, build the slideshow video on the laptop (needs `ffmpeg`):

       build/build-slideshow.sh /path/to/images 5

   This writes `~/robot-slideshow.mp4`. Images are ordered by any leading number in the filename
   (`-30, -15, 0, 15, 30`).

2. One time, give the laptop key-based SSH to the robot, then deploy:

       ssh-copy-id pi@192.168.89.80
       deploy/robot-deploy.sh pi@192.168.89.80 192.168.89.9

   This copies `rcast.py` and the video to the robot and installs the service (stopped, manual start).

3. Cast it (plays once, then stops):

       ssh pi@192.168.89.80
       sudo systemctl start rcast-slideshow     # cast once
       sudo systemctl stop  rcast-slideshow     # stop early
       systemctl is-active  rcast-slideshow     # check

   The service is installed but **disabled on boot** and **plays the slideshow once** then exits.
   To start it automatically at power-on: `sudo systemctl enable rcast-slideshow`.
   To loop instead of play once: add `--loop` to `ExecStart` in the unit.

If the TV shows nothing, give the set a quick source change or Home press first (clears a stuck photo
app), then start the service again.

## Files

### robot/ (runs on the Yanshee, Python 3.5, stdlib only)
- `rcast.py` — DLNA video caster. `rcast.py <file> --tv <ip> [--loop] [--daemon]`. Default plays once and exits.
- `rcast-slideshow.service` — systemd unit. Play once, manual start, not enabled on boot.

### laptop/ (helpers for the laptop; most need Python 3.6+, some need ffmpeg or Firefox)
- `tvcast` — one app: web control panel + CLI for an image slideshow, a video, etc.
  `tvcast serve` (web UI), `tvcast slides <folder>`, `tvcast video <file>`, `tvcast stop`.
- `dlna-push` — push one video or image file to the TV. `dlna-push <file> [--tv ip]`, `dlna-push --stop`.
- `tv-slides` — build a crossfade slideshow **video** from an images folder (used by build-slideshow.sh).
- `tv-scroll` — turn a long web page into a slow auto-scroll video and play it.
- `tv-web` — render a web page to a still image and show it (no browser login on the TV).
- `tv-open` — Samsung remote-control API (pairs with a token); browser launch is blocked on the 2024 model.

### deploy/ and build/
- `deploy/robot-deploy.sh` — copy the caster + video to the robot and install the service.
- `build/build-slideshow.sh` — build `robot-slideshow.mp4` from an images folder on the laptop.

## Notes

- Media files (`*.mp4`, images) and logs are git-ignored; regenerate the video with `build/build-slideshow.sh`.
- SSH to the robot is key-based (`pi@192.168.89.80`). A plain backgrounded process on the robot is killed
  when the SSH session ends (systemd-logind session cleanup), which is why the caster runs as a systemd unit.
- The robot and the TV must be on the same network; both were on `192.168.89.x`.
