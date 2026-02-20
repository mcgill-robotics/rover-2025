import { useEffect, useMemo, useRef, useState } from "react";

type CameraStreamState = {
  status: "idle" | "connecting" | "connected" | "error";
  error?: string;
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
    for (const id of cameraIds) refs[id] = { current: null };
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
        if (cancelled) return;
        const st = pc.connectionState;
        if (st === "connected") setState((p) => ({ ...p, [cameraId]: { status: "connected" } }));
        if (st === "failed" || st === "disconnected" || st === "closed") {
          setState((p) => ({ ...p, [cameraId]: { status: "error", error: `WebRTC state: ${st}` } }));
        }
      };

      pc.ontrack = (ev) => {
        const stream = ev.streams?.[0];
        if (!stream) return;
        const videoEl = videoRefs[cameraId]?.current;
        if (videoEl && videoEl.srcObject !== stream) videoEl.srcObject = stream;
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

  return { videoRefs, state };
}
