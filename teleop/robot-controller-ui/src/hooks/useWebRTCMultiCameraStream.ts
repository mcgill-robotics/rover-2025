import { createRef, useEffect, useMemo, useRef, useState } from "react";

type CameraStreamState = {
  status: "idle" | "connecting" | "connected" | "error";
  error?: string;
  stream?: MediaStream;
};

type Args = {
  backendBaseUrl: string; // e.g. "http://172.26.38.8:8001"
  cameraIds: string[];
};

export function useWebRTCMultiCameraStream({ backendBaseUrl, cameraIds }: Args) {
  const pcsRef = useRef<Record<string, RTCPeerConnection>>({});
  const [state, setState] = useState<Record<string, CameraStreamState>>({});
  const videoRefs = useMemo(() => {
    const refs: Record<string, React.RefObject<HTMLVideoElement>> = {};
    for (const id of cameraIds) refs[id] = createRef<HTMLVideoElement>();
    return refs;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cameraIds.join("|")]);

  useEffect(() => {
    let cancelled = false;

    async function connectOne(cameraId: string) {
      if (pcsRef.current[cameraId]) {
        return;
      }
      setState((prev) => ({ ...prev, [cameraId]: { status: "connecting" } }));

      const existing = pcsRef.current[cameraId];
      if (existing) {
        try { existing.close(); } catch {}
        delete pcsRef.current[cameraId];
      }

      const pc = new RTCPeerConnection();
      pcsRef.current[cameraId] = pc;

      pc.onconnectionstatechange = () => {
        if (pc.connectionState === "connected") {
          setState((p) => ({ ...p, [cameraId]: { status: "connected" } }));
          
          const videoEl = videoRefs[cameraId]?.current;
          if (videoEl) {
            videoEl.play().catch((e) => {
              console.warn("[webrtc] play() failed on connection", cameraId, e);
            });
          }
        }
      };

      pc.oniceconnectionstatechange = () =>
        console.log("[webrtc]", cameraId, "pc.iceConnectionState =", pc.iceConnectionState);

      pc.ontrack = (ev) => {
        console.log(
          "[webrtc] ontrack", cameraId,
          "streams=", ev.streams?.length ?? 0,
          "track=", ev.track?.kind,
          "readyState=", ev.track?.readyState
        );

        // const videoEl = videoRefs[cameraId]?.current;
        // if (!videoEl) {
        //   console.warn("[webrtc] no video ref for", cameraId);
        //   return;
        // }

        // Some browsers / setups do not populate ev.streams; fall back to a MediaStream
        const stream = (ev.streams && ev.streams.length > 0)
          ? ev.streams[0]
          : new MediaStream([ev.track])

        // if (videoEl.srcObject !== stream) {
        //   videoEl.srcObject = stream;
        // }

        setState((p) => ({
          ...p,
          [cameraId]: { 
            ...p[cameraId], 
            stream 
          }
        }));

        const videoEl = videoRefs[cameraId]?.current;
        if (videoEl && videoEl.srcObject !== stream) {
          videoEl.srcObject = stream;
          videoEl.play().catch((e) => {
            console.warn("[webrtc] video.play() failed for", cameraId, e);
          });
        }

        // Ensure playback starts (autoplay can fail without this)
        videoEl.play().catch((e) => {
          console.warn("[webrtc] video.play() failed for", cameraId, e);
        });
      };
      
      pc.addTransceiver("video", { direction: "recvonly" });

      try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        const base = backendBaseUrl.replace(/\/$/, "");
        const res = await fetch(`${base}/offer?id=${encodeURIComponent(cameraId)}`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(pc.localDescription),
        });

        if (!res.ok) {
          const text = await res.text();
          throw new Error(`offer failed: ${res.status} ${text}`);
        }

        const answer = await res.json();
        await pc.setRemoteDescription(answer);
      } catch (e: any) {
        if (cancelled) return;
        setState((p) => ({ ...p, [cameraId]: { status: "error", error: e?.message ?? String(e) } }));
      }
    }

    for (const id of cameraIds) connectOne(id);

    return () => {
      cancelled = true;
      for (const id of cameraIds) {
        const pc = pcsRef.current[id];
        if (pc) {
          try { pc.close(); } catch {}
          delete pcsRef.current[id];
        }
        const videoEl = videoRefs[id]?.current;
        if (videoEl) videoEl.srcObject = null;
      }
    };
  }, [backendBaseUrl, cameraIds.join("|")]);

  useEffect(() => {
    cameraIds.forEach((id) => {
      const videoEl = videoRefs[id]?.current;
      const stream = state[id]?.stream;

      if (videoEl && stream && videoEl.srcObject !== stream) {
        videoEl.srcObject = stream;
        videoEl.play().catch((e) => {
          console.warn("[webrtc] video.play() failed for", id, e);
        });
      }
    });
  });

  return { videoRefs, state };
}
