Three.js r160 (0.160.0), MIT license; bundled for the offline demo.

Upstream: https://github.com/mrdoob/three.js/tree/r160
Script: https://raw.githubusercontent.com/mrdoob/three.js/r160/build/three.min.js
License: three.LICENSE.txt (upstream LICENSE, unchanged)
Retrieved: 2026-11-08
SHA-256 (three.min.js):
170c6789f43217c96b3170f4b42fafe135de7f7cd48497a4218f9757ee1d49fa

This last standalone build exposes window.THREE for the existing dashboard
script tag. No CDN, module import, external fonts or OrbitControls are needed
at runtime. The upstream build emits a deprecation warning recommending ES
modules; the pinned local build still works without internet access.
