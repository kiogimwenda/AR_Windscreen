# Offline test data

`data/footage/` is gitignored (video is large and re-downloadable). It holds recorded or downloaded
road video used to exercise the perception pipeline before the Phase 4 camera exists
(docs/BUILD_GUIDE.md Part 13.2's offline replay idea, applied early).

## `footage/krakow_0-120s.webm`

- **Source:** "City Driving 4K- Kraków Poland 2024" by **Relaxing Roads 4K**, via Wikimedia
  Commons: https://commons.wikimedia.org/wiki/File:City_Driving_4K-_Krak%C3%B3w_Poland_2024.webm
- **Licence:** Creative Commons Attribution 3.0 Unported (CC BY 3.0). Any frame or result shown in
  the report or presentation must credit the author as above.
- **What:** the first 120 s (VP9, 2560×1440, 60 fps, 7,200 frames): forward-facing daytime city
  driving with traffic and pedestrians. It matches the planned camera resolution exactly.
- **How it was obtained:** one contiguous HTTP range request for the first 200 MB (seeking inside
  the remote file tripped Wikimedia's rate limit), then
  `ffmpeg -i krakow_head.webm -c copy -t 120 krakow_0-120s.webm`. No re-encoding.
- **Limitation:** European city streets, not Kenyan roads. Good for "does the pipeline work
  end to end", not for judging accuracy on the target domain. That needs the project's own
  recordings once the camera exists.

## `footage/kenya/` — Kenyan street imagery for the sign-detector evaluation (2026-09-25)

Real Kenyan road images used to judge the traffic-sign detector by eye (there are no ground-truth
labels). 300 still images, gitignored like the rest of `footage/`. Every file's source, author and
licence is listed per file in the two manifests beside the images; both licences below require
that attribution to be kept with any image shown in the report or presentation.

- **KartaView (formerly OpenStreetCam)**: 199 dashcam frames from 24 public sequences, shot
  2017–2025, in Nairobi (138) and on the A104 towards Limuru/Naivasha/Nakuru (61). Authors
  (KartaView usernames): **ToffeHoff** (179), **bruceyv83** (10), **alchimista** (10). Licence:
  **CC BY-SA 4.0** (KartaView's imagery licence). Per-image author, sequence, GPS position,
  capture date and a link to the KartaView page are in `kenya/manifest.csv`. The already-blurred
  ("proc") images were downloaded. Fetched by `host/scripts/fetch_kenya_imagery.py`: sequences
  listed by 0.05° tiles, every 12th frame, at most 10 per sequence. KartaView returned no
  sequences for Thika town, Mombasa, Eldoret or Kisumu through this API, and one storage host
  returned HTTP 500 for much of the night, which capped the A104 set.
- **Wikimedia Commons**: 101 photos from "Road signs in Kenya", "Warning road signs in Kenya",
  "Street signs in Kenya", "Road sign gantries in Kenya", "Roads in Nairobi", "Streets in
  Nairobi" and "Roads in Kenya". Only files whose Commons licence is CC0, CC BY or CC BY-SA were
  kept: CC BY-SA 4.0 (69), CC BY 2.0 (16), CC BY-SA 3.0 (11), CC BY 4.0 (3), CC BY-SA 2.0 (1),
  CC0 (1). Author, licence and file page per image are in `kenya/commons_manifest.csv`. Fetched
  as 1280 px thumbnails by `host/scripts/fetch_commons_kenya.py` (the run stopped early on HTTP
  429 rate limiting). Many are general street scenes, not all show a sign.
- **Limitation:** most KartaView frames come from one contributor's phone on a dashboard, and
  many frames contain no sign at all. This is an evaluation set for "what does the model do on
  Kenyan roads", not a benchmark.

`footage/kenya_annotated/` holds the detector's output on these images, and
`footage/krakow_signs/` its output on every 30th frame of the Kraków clip.
