/* Offline LiDAR view. three.min.js is vendored locally; no network assets. */
(() => {
  "use strict";

  function start() {
    const view = document.getElementById("view3d");
    const fallback = document.getElementById("rlidar");
    const caption = document.getElementById("rlidar-t");
    if (!view || !fallback || !caption || !window.THREE) return;
    const T = window.THREE;
    let renderer;
    try {
      renderer = new T.WebGLRenderer({ antialias: true, alpha: false, powerPreference: "low-power" });
    } catch (error) {
      // The existing 2D canvas remains usable on a machine without WebGL.
      caption.textContent = "LiDAR: 3D unavailable on this browser";
      return;
    }
    view.hidden = false;
    fallback.hidden = true;
    fallback.style.display = "none"; // #remote canvas otherwise overrides the hidden attribute
    Object.assign(view.style, { position: "relative", touchAction: "none" });
    view.setAttribute("role", "img");
    view.setAttribute("aria-label", "3D LiDAR cloud, people, SP1 path and relay nodes");
    renderer.setClearColor(0x090e16, 1);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    Object.assign(renderer.domElement.style, {
      position: "absolute", inset: "0", width: "100%", height: "100%",
      maxHeight: "none", aspectRatio: "auto", cursor: "grab"
    });
    view.appendChild(renderer.domElement);

    const scene = new T.Scene();
    const camera = new T.PerspectiveCamera(48, 4 / 3, 0.05, 5000);
    camera.up.set(0, 0, 1); // map x/y is horizontal; height is z throughout
    const floor = -0.9; // display datum only; the cloud itself stays relative to the scanner
    const grid = new T.GridHelper(60, 60, 0x35485b, 0x1a2938);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = floor;
    scene.add(grid);
    const cloudMaterial = new T.PointsMaterial({ size: 2.4, vertexColors: true,
      sizeAttenuation: false, transparent: true, opacity: 0.88 });
    const cloud = new T.Points(new T.BufferGeometry(), cloudMaterial);
    cloud.frustumCulled = false;
    scene.add(cloud);
    const pathLine = new T.Line(new T.BufferGeometry(), new T.LineBasicMaterial({ color: 0x58d6eb }));
    pathLine.frustumCulled = false;
    scene.add(pathLine);
    const peopleGroup = new T.Group();
    const nodesGroup = new T.Group();
    scene.add(peopleGroup, nodesGroup);
    const spotter = new T.Group();
    const arrowGeometry = new T.BufferGeometry();
    arrowGeometry.setAttribute("position", new T.Float32BufferAttribute([
      0, 0.65, 0.03, -0.23, -0.25, 0.03, 0.23, -0.25, 0.03
    ], 3));
    spotter.add(new T.Mesh(arrowGeometry, new T.MeshBasicMaterial({ color: 0x7cf3e0, side: T.DoubleSide })));
    const spotterStem = new T.Line(new T.BufferGeometry().setFromPoints([
      new T.Vector3(0, 0, floor), new T.Vector3(0, 0, 0)
    ]), new T.LineBasicMaterial({ color: 0x7cf3e0 }));
    spotter.add(spotterStem);
    scene.add(spotter);

    const legend = document.createElement("div");
    Object.assign(legend.style, { position: "absolute", left: "7px", bottom: "6px", pointerEvents: "none",
      font: "10px/1.3 system-ui, sans-serif", color: "#c7d4e0", textShadow: "0 1px 2px #000" });
    const swatch = document.createElement("span");
    Object.assign(swatch.style, { display: "inline-block", width: "42px", height: "5px", marginRight: "5px",
      verticalAlign: "2px", background: "linear-gradient(90deg,#4187ed,#42dec9,#f4ba65)" });
    legend.append(swatch, document.createTextNode("relative height"));
    view.appendChild(legend);
    const tools = document.createElement("div");
    Object.assign(tools.style, { position: "absolute", right: "5px", top: "5px", display: "flex", gap: "4px" });
    function button(text, title, action) {
      const b = document.createElement("button");
      b.type = "button";
      b.textContent = text;
      b.title = title;
      Object.assign(b.style, { font: "11px/1.2 system-ui, sans-serif", padding: "3px 5px", border: "1px solid #536477",
        borderRadius: "4px", background: "#101b28dd", color: "#e2edf7" });
      b.addEventListener("pointerdown", (event) => event.stopPropagation());
      b.addEventListener("click", action);
      tools.appendChild(b);
      return b;
    }
    view.appendChild(tools);

    const target = new T.Vector3(0, 0, 0);
    let azimuth = 0.7, elevation = 0.62, distance = 12;
    let dragging = false, fitted = false, manualCamera = false;
    let lastData = null, version = null, pathKey = "", peopleKey = "", nodesKey = "";
    let lastSuccess = 0, fetching = false, alive = true, pollTimer, frame;
    const presets = [];
    function preset(text, azimuthValue, elevationValue) {
      const b = button(text, `Use the ${text} view; drag to adjust`, () => {
        azimuth = azimuthValue; elevation = elevationValue; manualCamera = true;
        presets.forEach((item) => item.setAttribute("aria-pressed", String(item === b)));
      });
      b.setAttribute("aria-pressed", String(text === "3/4"));
      presets.push(b);
    }
    preset("Top", 0, Math.PI / 2 - 0.01);
    preset("Side", Math.PI / 2, 0.08);
    preset("3/4", 0.7, 0.62);
    button("Fit", "Fit the whole cloud and path; double-click also fits", () => {
      fit(lastData, true); manualCamera = false;
    });

    function resize() {
      const width = Math.max(1, view.clientWidth), height = Math.max(1, view.clientHeight);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
    const observer = window.ResizeObserver ? new ResizeObserver(resize) : null;
    if (observer) observer.observe(view);
    else window.addEventListener("resize", resize);
    resize();

    view.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || event.target.closest("button")) return;
      dragging = true;
      manualCamera = true;
      presets.forEach((item) => item.setAttribute("aria-pressed", "false"));
      view.setPointerCapture(event.pointerId);
      renderer.domElement.style.cursor = "grabbing";
      dragX = event.clientX;
      dragY = event.clientY;
    });
    let dragX = 0, dragY = 0;
    view.addEventListener("pointermove", (event) => {
      if (!dragging) return;
      azimuth -= (event.clientX - dragX) * 0.008;
      elevation = T.MathUtils.clamp(elevation + (event.clientY - dragY) * 0.008, 0.08, 1.48);
      dragX = event.clientX;
      dragY = event.clientY;
    });
    function stopDrag() {
      dragging = false;
      renderer.domElement.style.cursor = "grab";
    }
    view.addEventListener("pointerup", stopDrag);
    view.addEventListener("pointercancel", stopDrag);
    view.addEventListener("lostpointercapture", () => { if (dragging) stopDrag(); });
    view.addEventListener("wheel", (event) => {
      event.preventDefault();
      distance = T.MathUtils.clamp(distance * Math.exp(T.MathUtils.clamp(event.deltaY, -300, 300) * 0.0015), 2, 2500);
      manualCamera = true;
    }, { passive: false });
    view.addEventListener("dblclick", () => { fit(lastData, true); manualCamera = false; });

    function fit(data, immediate) {
      if (!data) return;
      const positions = [];
      if (data.bounds) positions.push(...data.bounds);
      if (Array.isArray(data.pose)) positions.push(data.pose.slice(0, 2));
      for (const person of data.persons || []) if (located(person)) positions.push([person.x, person.y]);
      for (const node of Object.values(data.nodes || {})) if (located(node)) positions.push([node.x, node.y]);
      for (const point of data.path || []) positions.push(point);
      if (!positions.length) return;
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      for (const point of positions) {
        if (!Number.isFinite(point[0]) || !Number.isFinite(point[1])) continue;
        minX = Math.min(minX, point[0]); maxX = Math.max(maxX, point[0]);
        minY = Math.min(minY, point[1]); maxY = Math.max(maxY, point[1]);
      }
      if (!Number.isFinite(minX)) return;
      const center = new T.Vector3((minX + maxX) / 2, (minY + maxY) / 2, -0.1);
      const extent = Math.max(4, Math.hypot(maxX - minX, maxY - minY) + 2);
      const halfFov = Math.atan(Math.tan(camera.fov * Math.PI / 360) * Math.min(1, camera.aspect));
      const radius = T.MathUtils.clamp(extent / (2 * Math.sin(halfFov)), 5, 2500);
      if (immediate || !fitted) { target.copy(center); distance = radius; }
      else { target.lerp(center, 0.2); distance += (radius - distance) * 0.2; }
      grid.position.x = Math.round(center.x);
      grid.position.y = Math.round(center.y);
      fitted = true;
    }

    function disposeGroup(group) {
      for (const child of [...group.children]) {
        group.remove(child);
        child.traverse((object) => {
          if (object.geometry) object.geometry.dispose();
          const materials = Array.isArray(object.material) ? object.material : [object.material];
          for (const material of materials) if (material) {
            if (material.map) material.map.dispose();
            material.dispose();
          }
        });
      }
    }
    function label(text, color, x, y, z, width = 1.7) {
      const canvas = document.createElement("canvas");
      canvas.width = 256; canvas.height = 48;
      const context = canvas.getContext("2d");
      context.fillStyle = "rgba(9,14,22,0.85)";
      context.fillRect(0, 0, 256, 48);
      context.fillStyle = color;
      context.font = "bold 25px system-ui, sans-serif";
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(text, 128, 25, 244);
      const sprite = new T.Sprite(new T.SpriteMaterial({ map: new T.CanvasTexture(canvas), depthTest: false }));
      sprite.position.set(x, y, z);
      sprite.scale.set(width, width * 48 / 256, 1);
      sprite.renderOrder = 3;
      return sprite;
    }
    function located(object) {
      return object && Number.isFinite(object.x) && Number.isFinite(object.y);
    }
    function lineSegments(group, coordinates, color, opacity = 1) {
      if (!coordinates.length) return;
      const geometry = new T.BufferGeometry();
      geometry.setAttribute("position", new T.Float32BufferAttribute(coordinates, 3));
      group.add(new T.LineSegments(geometry, new T.LineBasicMaterial({ color, transparent: true, opacity })));
    }

    // COCO: nose, eyes, ears, shoulders, elbows, wrists, hips, knees, ankles.
    const bones = [[0, 1], [0, 2], [1, 3], [2, 4], [0, 5], [0, 6], [5, 6],
      [5, 7], [7, 9], [6, 8], [8, 10], [5, 11], [6, 12], [11, 12],
      [11, 13], [13, 15], [12, 14], [14, 16]];

    function drawPeople(data) {
      disposeGroup(peopleGroup);
      const pose = data.pose || [0, 0, 0];
      for (const person of data.persons || []) {
        if (!located(person)) continue;
        const down = person.ps === "D", upright = person.ps === "U";
        const color = down ? 0xff5d68 : upright ? 0xffd17b : 0xd3dce7;
        if (person.age != null && person.age >= 15) {
          const geometry = new T.BufferGeometry().setFromPoints([new T.Vector3(person.x, person.y, floor + .1)]);
          peopleGroup.add(new T.Points(geometry, new T.PointsMaterial({ color, size: 3,
            sizeAttenuation: false, transparent: true, opacity: .3 })));
          continue;
        }
        const opacity = 1;
        const bearing = Math.hypot(pose[0] - person.x, pose[1] - person.y) > 0.05
          ? Math.atan2(person.x - pose[0], person.y - pose[1]) : pose[2];
        const right = [Math.cos(bearing), -Math.sin(bearing)];
        const forward = [Math.sin(bearing), Math.cos(bearing)];
        const kp = person.kp;
        const joints = [];
        if (Array.isArray(kp) && kp.length === 34 && (down || upright)) {
          for (let i = 0; i < 34; i += 2) {
            const x = kp[i], y = kp[i + 1];
            if (!(x >= 0 && x <= 99 && y >= 0 && y <= 99)) { joints.push(null); continue; }
            const lateral = (x / 99 - 0.5) * (down ? 1.7 : 0.65);
            const depth = down ? (y / 99 - 0.5) * 0.65 : 0;
            joints.push([person.x + right[0] * lateral + forward[0] * depth,
              person.y + right[1] * lateral + forward[1] * depth,
              down ? floor + 0.1 : floor + (1 - y / 99) * 1.7]);
          }
        }
        const lines = [];
        if (joints.length) for (const [a, b] of bones) {
          if (joints[a] && joints[b]) lines.push(...joints[a], ...joints[b]);
        }
        if (lines.length >= 12) {
          lineSegments(peopleGroup, lines, color, opacity);
          const dots = new T.BufferGeometry();
          dots.setAttribute("position", new T.Float32BufferAttribute(joints.filter(Boolean).flat(), 3));
          peopleGroup.add(new T.Points(dots, new T.PointsMaterial({ color, size: 4,
            sizeAttenuation: false, transparent: true, opacity })));
        } else {
          // No pose packet yet: a simple vertical or prone 1.7 m marker.
          const a = down ? [person.x - right[0] * 0.85, person.y - right[1] * 0.85, floor + 0.1]
            : [person.x, person.y, floor];
          const b = down ? [person.x + right[0] * 0.85, person.y + right[1] * 0.85, floor + 0.1]
            : [person.x, person.y, upright ? floor + 1.7 : floor + 0.45];
          lineSegments(peopleGroup, [...a, ...b], color, opacity);
          const head = new T.Mesh(new T.SphereGeometry(down || upright ? 0.12 : 0.18, 10, 8),
            new T.MeshBasicMaterial({ color, transparent: true, opacity }));
          head.position.set(...b);
          peopleGroup.add(head);
        }
        const ring = new T.Mesh(new T.RingGeometry(down ? 0.38 : 0.22, down ? 0.43 : 0.26, 24),
          new T.MeshBasicMaterial({ color, side: T.DoubleSide, transparent: true, opacity }));
        ring.position.set(person.x, person.y, floor + 0.02);
        peopleGroup.add(ring);
        const text = `P${person.id}${down ? " DOWN" : upright ? " UPRIGHT" : ""}`;
        peopleGroup.add(label(text, down ? "#ff7b84" : "#ffe1a2", person.x, person.y,
          down ? floor + 0.45 : floor + 2.0, down ? 2.1 : 2.0));
      }
    }

    function drawNodes(data) {
      disposeGroup(nodesGroup);
      for (const [name, node] of Object.entries(data.nodes || {})) {
        if (!located(node)) continue;
        const marker = new T.Mesh(new T.OctahedronGeometry(0.19), new T.MeshBasicMaterial({ color: 0xb99bff }));
        marker.position.set(node.x, node.y, floor + 0.2);
        nodesGroup.add(marker, label(name, "#d2bcff", node.x, node.y, floor + 0.65, 1.1));
      }
    }

    function update(data) {
      if (data.encoding !== "i16le" || !Number.isInteger(data.count) || data.count < 0 || data.count > 30000)
        throw new Error("unsupported cloud payload");
      if (version !== data.v) {
        const binary = atob(data.points);
        if (binary.length !== data.count * 6) throw new Error("incomplete cloud payload");
        const bytes = Uint8Array.from(binary, (c) => c.charCodeAt(0));
        const packed = new DataView(bytes.buffer);
        const positions = new Float32Array(data.count * 3);
        const colors = new Float32Array(data.count * 3);
        const low = new T.Color(0x4187ed), mid = new T.Color(0x42dec9), high = new T.Color(0xf4ba65);
        const color = new T.Color();
        for (let i = 0; i < data.count; i++) {
          for (let axis = 0; axis < 3; axis++)
            positions[i * 3 + axis] = data.origin[axis] + packed.getInt16(i * 6 + axis * 2, true) * data.scale;
          const height = T.MathUtils.clamp((positions[i * 3 + 2] + 1) / 2, 0, 1);
          if (height < 0.5) color.copy(low).lerp(mid, height * 2);
          else color.copy(mid).lerp(high, height * 2 - 1);
          color.toArray(colors, i * 3);
        }
        const geometry = new T.BufferGeometry();
        geometry.setAttribute("position", new T.BufferAttribute(positions, 3));
        geometry.setAttribute("color", new T.BufferAttribute(colors, 3));
        cloud.geometry.dispose();
        cloud.geometry = geometry;
        version = data.v;
      }
      const nextPath = JSON.stringify(data.path || []);
      if (nextPath !== pathKey) {
        const geometry = new T.BufferGeometry().setFromPoints((data.path || [])
          .map((point) => new T.Vector3(point[0], point[1], floor + 0.04)));
        pathLine.geometry.dispose(); pathLine.geometry = geometry; pathKey = nextPath;
      }
      const nextPeople = JSON.stringify([data.pose, (data.persons || []).map((p) =>
        [p.id, p.x, p.y, p.ps, p.kp, p.age != null && p.age >= 15])]);
      if (nextPeople !== peopleKey) { drawPeople(data); peopleKey = nextPeople; }
      const nextNodes = JSON.stringify(data.nodes || {});
      if (nextNodes !== nodesKey) { drawNodes(data); nodesKey = nextNodes; }
      const pose = data.pose || [0, 0, 0];
      spotter.position.set(pose[0], pose[1], 0);
      spotter.rotation.z = -pose[2]; // heading clockwise from +y
      lastData = data;
      if (!manualCamera && (data.count > 0 || (data.path || []).length > 1))
        fit(data, !fitted);
      view.dataset.points = String(data.count);
      view.dataset.people = String((data.persons || []).filter(located).length);
      updateCaption();
    }

    function updateCaption(error = false) {
      if (!lastData) {
        caption.textContent = error ? "3D LiDAR: waiting for connection" : "3D LiDAR: waiting for the first scan";
        caption.classList.toggle("stale", error);
        return;
      }
      const allPeople = lastData.persons || [];
      const people = allPeople.filter((p) => p.age == null || p.age < 15), down = people.filter((p) => p.ps === "D").length;
      const unlocated = people.filter((p) => !located(p)).length;
      const stale = error || lastData.scan_age == null || lastData.scan_age > 3 || Date.now() - lastSuccess > 3500;
      let status = `${lastData.count.toLocaleString()} pts`;
      if (people.length) status += ` | ${people.length} live${down ? `, ${down} down` : ""}`;
      if (unlocated) status += ` | ${unlocated} unlocated`;
      status += stale ? " | scan stale" : lastData.scan_ok === false ? " | scan rejected" : "";
      caption.textContent = `3D LiDAR: ${status}`;
      caption.title = `${caption.textContent}. ${lastData.tilt ? "IMU tilt relative to the level reference" : "Flat scan: no valid tilt sample"}. ` +
        `${allPeople.length - people.length} earlier sightings are faded dots. ` +
        "Heights are relative to the LiDAR plane; person heights are schematic. Drag to orbit, scroll to zoom.";
      caption.classList.toggle("stale", stale || lastData.scan_ok === false);
      cloudMaterial.opacity = stale ? 0.38 : lastData.scan_ok === false ? 0.58 : 0.88;
    }

    async function poll() {
      if (!alive) return;
      if (!document.hidden && !fetching) {
        fetching = true;
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 3500);
        try {
          const response = await fetch("/api/cloud", { cache: "no-store", signal: controller.signal });
          if (!response.ok) throw new Error(`cloud ${response.status}`);
          const data = await response.json();
          lastSuccess = Date.now();
          update(data);
        } catch (error) { updateCaption(true); }
        finally { clearTimeout(timeout); fetching = false; }
      }
      pollTimer = setTimeout(poll, 750); // single flight, including on a slow/disconnected link
    }
    renderer.domElement.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      caption.textContent = "3D LiDAR: graphics interrupted; reload to restore";
      caption.classList.add("stale");
    });
    let previousFrame = 0;
    function animate(now) {
      if (!alive) return;
      frame = requestAnimationFrame(animate);
      if (document.hidden || now - previousFrame < 32) return;
      previousFrame = now;
      camera.position.set(target.x + distance * Math.cos(elevation) * Math.sin(azimuth),
        target.y - distance * Math.cos(elevation) * Math.cos(azimuth), target.z + distance * Math.sin(elevation));
      camera.lookAt(target);
      renderer.render(scene, camera);
    }
    window.addEventListener("pagehide", (event) => {
      if (event.persisted) return;
      alive = false;
      clearTimeout(pollTimer);
      cancelAnimationFrame(frame);
      if (observer) observer.disconnect();
      disposeGroup(scene);
      renderer.dispose();
    });
    updateCaption();
    poll();
    frame = requestAnimationFrame(animate);
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", start, { once: true });
  else start();
})();
