# Robot Controller UI

A Next.js-based web interface for controlling and monitoring the Mars Rover's teleoperation systems. This UI provides real-time data visualization, motor diagnostics, and system control capabilities.

## 📁 Project Structure

```
robot-controller-ui/
├── README.md                    # This documentation
├── package.json                 # Dependencies and npm scripts
├── next.config.ts              # Next.js configuration
├── tsconfig.json               # TypeScript configuration
├── eslint.config.mjs           # ESLint configuration
├── postcss.config.mjs          # PostCSS configuration
├── .gitignore                  # Git ignore rules
│
├── public/                     # Static assets
│   ├── mcgillRobotics.svg     # Logo
│   └── gamepads/              # Gamepad icons
│       ├── base.svg
│       ├── bumper.svg
│       └── ...
│
└── src/                        # Source code
    ├── app/                    # Next.js App Router
    │   ├── layout.tsx          # Root layout
    │   ├── page.tsx            # Home page
    │   ├── globals.css         # Global styles
    │   ├── arm/                # Arm control page
    │   ├── drive/              # Drive control page
    │   └── status/             # System status page
    │
    ├── components/             # React components
    │   ├── icons/              # Icon components
    │   ├── ui/                 # Reusable UI components
    │   │   ├── ArrowButton.tsx
    │   │   ├── DPad.tsx
    │   │   ├── IconButton.tsx
    │   │   └── PowerButton.tsx
    │   ├── layout/             # Layout components
    │   │   └── navbar/         # Navigation bar
    │   └── sections/           # Feature sections
    │       ├── arm/            # Arm control components
    │       │   ├── control/    # Arm control interface
    │       │   ├── info/       # Arm information display
    │       │   └── view/       # Arm visualization
    │       └── drive/          # Drive system components
    │           ├── camera/     # Camera controls
    │           └── mobility/   # Drive controls & info
    │               ├── control/    # Drive controls
    │               ├── info/       # Motor diagnostics
    │               └── navigation/ # GPS & navigation
    │
    ├── hooks/                  # Custom React hooks
    │   ├── useBandwidthStats.ts
    │   ├── useCameraList.ts
    │   ├── useDriveData.ts
    │   └── useWebRTCStreams.ts
    │
    ├── store/                  # State management (Zustand)
    │   ├── index.ts
    │   ├── rosStore.ts
    │   └── types.ts
    │
    ├── context/                # React contexts
    │   └── NavbarContext.tsx
    │
    ├── config/                 # Configuration files
    │   ├── camera.ts
    │   └── network.ts
    │
    ├── lib/                    # Utility libraries
    │   └── utils.ts
    │
    └── assets/                 # Asset files
        └── images/
            └── logo.png
```

## Getting Started

### Prerequisites

- **Node.js** (v18 or higher)
- **npm** (comes with Node.js)

### Installation

1. **Navigate to the project directory:**
   ```bash
   cd teleop/robot-controller-ui
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

### Running the Development Server

1. **Start the development server:**
   ```bash
   npm run dev
   ```

2. **Open your browser:**
   Navigate to [http://localhost:3000](http://localhost:3000)

   The page will auto-reload when you make changes to the code.

### Building for Production

1. **Create a production build:**
   ```bash
   npm run build
   ```

2. **Start the production server:**
   ```bash
   npm start
   ```

### Other Commands

- **Lint the code:**
  ```bash
  npm run lint
  ```

## 🛠️ Technology Stack

- **Framework:** Next.js 15.3.3 with App Router
- **Language:** TypeScript
- **Styling:** Tailwind CSS v4
- **State Management:** Zustand
- **Charts:** Recharts
- **HTTP Client:** Axios
- **Icons:** Lucide React
- **Build Tool:** Turbopack (Next.js built-in)

## 🔧 Configuration Files

### `next.config.ts`
Next.js configuration for build settings, redirects, and environment variables.

### `tsconfig.json`
TypeScript compiler configuration with path aliases and strict type checking.

### `tailwind.config.js`
Tailwind CSS configuration for custom themes, colors, and responsive breakpoints.

### `eslint.config.mjs`
ESLint configuration for code quality and consistency.

## 📱 Application Pages

### Home Page (`/`)
- Overview dashboard
- System status summary
- Quick navigation to subsystems

### Drive Page (`/drive`)
- Motor diagnostics and control
- Speed monitoring
- Camera feeds
- GPS navigation

### Arm Page (`/arm`)
- Arm joint control
- Position visualization
- Command logging
- PS4 controller interface

### Status Page (`/status`)
- System health monitoring
- Connection status
- Performance metrics

## 🎨 Component Architecture

### UI Components (`src/components/ui/`)
Reusable interface elements:
- **ArrowButton**: Directional navigation buttons
- **DPad**: D-pad controller interface
- **IconButton**: Icon-based action buttons
- **PowerButton**: System power controls

### Section Components (`src/components/sections/`)
Feature-specific components organized by system:

#### Drive Section
- **MobilityPanel**: Main drive interface
- **DriveControl**: Speed and direction controls
- **DriveInfo**: Motor diagnostics display
- **CameraView**: Camera feed integration
- **GPSPanel**: Navigation information

#### Arm Section
- **ArmControl**: Joint position controls
- **ArmInfo**: Joint status and coordinates
- **ArmView**: 3D visualization
- **PS4Controller**: Gamepad interface

### Layout Components (`src/components/layout/`)
- **Navbar**: Main navigation with responsive design
- **Logo**: Brand identity component

## 🔗 State Management

The application uses **Zustand** for state management:

### ROS Store (`src/store/rosStore.ts`)
- WebSocket connection management
- Real-time data from ROS topics
- Motor diagnostics and status
- System health monitoring

### Store Types (`src/store/types.ts`)
TypeScript definitions for all state interfaces and data structures.

## 🪝 Custom Hooks

### `useDriveData`
Manages drive system data including motor diagnostics, speeds, and connection status.

### `useWebRTCStreams`
Handles camera stream connections and WebRTC communication.

### `useCameraList`
Manages available camera sources and switching between feeds.

### `useBandwidthStats`
Monitors network performance and data usage statistics.

## 🚨 Common Troubleshooting

### Installation Issues

**Problem:** `npm install` fails with permission errors
```bash
# Solution: Use npm with proper permissions
sudo npm install -g npm@latest
npm install
```

**Problem:** Node.js version compatibility
```bash
# Check your Node.js version
node --version

# Install Node.js 18+ if needed
# Visit: https://nodejs.org/
```

### Development Server Issues

**Problem:** Port 3000 already in use
```bash
# Solution: Use a different port
npm run dev -- -p 3001

# Or kill the process using port 3000
lsof -ti:3000 | xargs kill -9
```

**Problem:** Hot reload not working
```bash
# Solution: Clear Next.js cache
rm -rf .next
npm run dev
```

### Build Issues

**Problem:** TypeScript compilation errors
```bash
# Solution: Check TypeScript configuration
npx tsc --noEmit

# Fix type errors in your code
# Check tsconfig.json for strict settings
```

**Problem:** Build fails with memory issues
```bash
# Solution: Increase Node.js memory limit
NODE_OPTIONS="--max-old-space-size=4096" npm run build
```

### Runtime Issues

**Problem:** Components not rendering
- Check browser console for JavaScript errors
- Verify all imports are correct
- Ensure components are properly exported

**Problem:** Styles not loading
```bash
# Solution: Rebuild Tailwind CSS
rm -rf .next
npm run dev
```

**Problem:** API connection issues
- Verify backend services are running
- Check network configuration in `src/config/network.ts`
- Inspect browser Network tab for failed requests

### Performance Issues

**Problem:** Slow page loads
- Check for large bundle sizes: `npm run build`
- Optimize images in the `public/` directory
- Review component re-rendering with React DevTools

**Problem:** Memory leaks
- Check for unsubscribed WebSocket connections
- Review useEffect cleanup functions
- Monitor browser memory usage in DevTools

### Environment Issues

**Problem:** Environment variables not loading
```bash
# Create .env.local file in project root
NEXT_PUBLIC_API_URL=http://localhost:8082
NEXT_PUBLIC_WS_URL=ws://localhost:8082/ws
```

**Problem:** CORS errors when connecting to backend
- Verify backend CORS configuration
- Check API URLs in network configuration
- Ensure proper protocol (http/https, ws/wss)

## 🔍 Debugging Tips

### Browser DevTools
1. **Console Tab**: Check for JavaScript errors and warnings
2. **Network Tab**: Monitor API requests and WebSocket connections
3. **React DevTools**: Inspect component state and props
4. **Performance Tab**: Profile rendering performance

### Next.js Debugging
```bash
# Enable debug mode
DEBUG=* npm run dev

# Check build analysis
npm run build -- --analyze
```

### Common Error Messages

**"Module not found"**
- Check import paths and file extensions
- Verify the
