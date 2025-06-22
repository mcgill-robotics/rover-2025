import { useEffect, useState } from "react";
import axios from "axios";
import { CAMERAIP } from "@/config/network";

export function useBandwidthStats(isStreaming: boolean) {
  const [bitrateKbps, setBitrate] = useState<string>("0");
  const [pingMs, setPing] = useState<number | null>(null);

  useEffect(() => {
    if (!isStreaming) return;

    const fetchStats = async () => {
      try {
        const res = await axios.get(`http://${CAMERAIP}:8081/bandwidth-stats`);
        const stats = res.data.bandwidth_stats?.[0];

        if (typeof stats?.bitrate_kbps === "number") {
          setBitrate(String(stats.bitrate_kbps));
        }
        if (typeof stats?.rtt_ms === "number") {
          setPing(stats.rtt_ms);
        }
      } catch (err) {
        console.error("Failed to fetch bandwidth stats:", err);
      }
    };

    fetchStats(); // initial fetch
    const intervalId = setInterval(fetchStats, 2000); // repeat every 2s
    return () => clearInterval(intervalId);
  }, [isStreaming]);

  return { bitrateKbps, pingMs };
}
