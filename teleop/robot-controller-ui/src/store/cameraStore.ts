import { create } from 'zustand';

export type ViewMode = 'single' | 'multi';

interface CameraInfo {
  camera_id: string;
  name: string;
  device_id: string;
  is_active: boolean;
  fps?: number;
  resolution?: [number, number];
}

interface CameraSlot {
  camera: CameraInfo | null;
  isActive: boolean;
}

interface CameraState {
  viewMode: ViewMode;
  showCameraManager: boolean;
  selectedCameraIndex: number;
  multiCameraSlots: CameraSlot[];
  setViewMode: (mode: ViewMode) => void;
  setShowCameraManager: (show: boolean) => void;
  toggleCameraManager: () => void;
  setSelectedCameraIndex: (index: number) => void;
  setMultiCameraSlots: (slots: CameraSlot[]) => void;
  addCameraToSlot: (camera: CameraInfo, slotIndex?: number) => void;
  removeCameraFromSlot: (slotIndex: number) => void;
}

export const useCameraStore = create<CameraState>((set, get) => ({
  viewMode: 'single',
  showCameraManager: false,
  selectedCameraIndex: 0,
  multiCameraSlots: [],
  setViewMode: (mode) => set({ viewMode: mode }),
  setShowCameraManager: (show) => set({ showCameraManager: show }),
  toggleCameraManager: () => set((state) => ({ showCameraManager: !state.showCameraManager })),
  setSelectedCameraIndex: (index) => set({ selectedCameraIndex: index }),
  setMultiCameraSlots: (slots) => set({ multiCameraSlots: slots }),
  addCameraToSlot: (camera, slotIndex) => {
    const { multiCameraSlots } = get();
    const newSlots = [...multiCameraSlots];
    
    if (slotIndex !== undefined) {
      // Add to specific slot
      if (slotIndex < newSlots.length) {
        newSlots[slotIndex] = { camera, isActive: true };
      }
    } else {
      // Add to first empty slot or create new slot
      const emptySlotIndex = newSlots.findIndex(slot => !slot.camera);
      if (emptySlotIndex !== -1) {
        newSlots[emptySlotIndex] = { camera, isActive: true };
      } else {
        newSlots.push({ camera, isActive: true });
      }
    }
    
    set({ multiCameraSlots: newSlots });
  },
  removeCameraFromSlot: (slotIndex) => {
    const { multiCameraSlots } = get();
    const newSlots = [...multiCameraSlots];
    if (slotIndex < newSlots.length) {
      newSlots[slotIndex] = { camera: null, isActive: false };
      set({ multiCameraSlots: newSlots });
    }
  },
}));
