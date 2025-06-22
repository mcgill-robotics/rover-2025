import { useEffect, useState } from "react";
import axios from "axios";
import { CAMERAIP } from "@/config/network";
import { TARGET_CAMERA_NAMES } from "@/config/camera";

export interface CameraInfo {
  name: string;
  path: string;
}

export function useCameraList() {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);

  useEffect(() => {
    const fetchCameraPaths = async () => {
      try {
        const res = await axios.get(`http://${CAMERAIP}:8081/video-devices`);
        const found: CameraInfo[] = [];

        res.data.devices.forEach((entry: { name: string; devices: string[] }) => {
          TARGET_CAMERA_NAMES.forEach((target) => {
            if (entry.name.includes(target)) {
              found.push({ name: target, path: entry.devices[0] });
            }
          });
        });

        setCameras(found);
      } catch (err) {
        console.error("Failed to fetch camera devices:", err);
      }
    };

    fetchCameraPaths();
  }, []);

  return cameras;
}
