# Robot Controller UI

A modern Next.js-based web interface for controlling and monitoring the Mars Rover's teleoperation systems. This UI provides real-time data visualization, motor diagnostics, and system control capabilities.

## 🏗️ Project Structure

```
robot-controller-ui/
├── README.md                    # This file
├── package.json                 # Dependencies and scripts
├── next.config.js              # Next.js configuration
├── tailwind.config.js          # Tailwind CSS configuration
├── tsconfig.json               # TypeScript configuration
├── public/                     # Static assets
│   ├── favicon.ico
│   └── images/
├── src/                        # Source code
│   ├── app/                    # Next.js App Router pages
│   │   ├── layout.tsx          # Root layout component
│   │   ├── page.tsx            # Home page
│   │   └── globals.css         # Global styles
│   ├── components/             # React components
│   │   ├── ui/                 # Reusable UI components
│   │   │   ├── Button.tsx
│   │   │   ├── Card.tsx
│   │   │   ├── Badge.tsx
│   │   │   └── ...
│   │   ├── sections/           # Main UI sections
│   │   │   ├── drive/          # Drive system components
│   │   │   │   ├── mobility/   # Mobility controls
│   │   │   │   │   ├── controls/
│   │   │   │   │   │   ├── DriveControls.tsx
│   │   │   │   │   │   └── SpeedControl.tsx
│   │   │   │   │   └── info/
│   │   │   │   │       ├── DriveInfo.tsx
│   │   │   │   │       ├── MotorStatus.tsx
│   │   │   │   │       └── DiagnosticsPanel.tsx
│   │   │   │   └── DriveSection.tsx
│   │   │   ├── arm/            # Arm control components
│   │   │   ├── camera/         # Camera feed components
│   │   │   └── system/         # System status components
│   │   ├── layout/             # Layout components
│   │   │   ├── Header.tsx
│   │   │   ├── Sidebar.tsx
│   │   │   └── Footer.tsx
│   │   └── common/             # Common components
│   │       ├── LoadingSpinner.tsx
│   │       ├── ErrorBoundary.tsx
│   │       └── ConnectionStatus.tsx
│   ├── hooks/                  # Custom React hooks
│   │   ├── useWebSocket.ts     # WebSocket connection hook
│   │   ├── useRoverData.ts     # Rover data management hook
│   │   └── useLocalStorage.ts  # Local storage utilities
│   ├── services/               # API and service layers
│   │   ├── api.ts              # REST API client
│   │   ├── websocket.ts        # WebSocket client
│   │   └── types.ts            # TypeScript type definitions
│   ├── utils/                  # Utility functions
│   │   ├── formatters.ts       # Data formatting utilities
│   │   ├── validators.ts       # Input validation
│   │   └── constants.ts        # Application constants
│   └── styles/                 # Additional styles
│       └── components.css      # Component-specific styles
```

## 🎯 Key Features

### Drive System Monitoring
- **Real-time Motor Diagnostics**: Live voltage, current, temperature, and status monitoring for all 4 drive motors (RF, RB, LB, LF)
- **Speed Visualization**: Real-time speed data with graphical indicators
- **Connection Status**: Visual indicators for motor connectivity and health
- **Historical Data**: Trend graphs for motor performance over time

### User Interface Components
- **Responsive Design**: Works on desktop, tablet, and mobile devices
- **Dark/Light Theme**: Toggle between themes for different lighting conditions
- **Real-time Updates**: WebSocket-based live data streaming
- **Error Handling**: Graceful error handling with user-friendly messages
- **Loading States**: Smooth loading indicators for better UX

### System Integration
- **ROS2 Integration**: Direct connection to ROS2 backend services
- **WebSocket Communication**: Real-time bidirectional communication
- **REST API Support**: HTTP endpoints for configuration and control
- **Mock Data Support**: Testing mode with simulated data

## 🚀 Getting Started

### Prerequisites

- **Node.js** (v18 or higher)
- **npm** or **yarn** package manager
- **ROS2 Humble** (for backend integration)
- **Python 3.8+** (for ROS services)

### Installation

1. **Navigate to the UI directory:**
   ```bash
   cd teleop/robot-controller-ui
   ```

2. **Install dependencies:**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Set up environment variables:**
   ```bash
   cp .env.example .env.local
   ```
   
   Edit `.env.local` with your configuration:
   ```env
   NEXT_PUBLIC_API_BASE_URL=http://localhost:8082
   NEXT_PUBLIC_WS_URL=ws://localhost:8082/ws
   NEXT_PUBLIC_ENVIRONMENT=development
   ```

### Development Mode

1. **Start the development server:**
   ```bash
   npm run dev
   # or
   yarn dev
   ```

2. **Open your browser:**
   Navigate to [http://localhost:3000](http://localhost:3000)

### Production Build

1. **Build the application:**
   ```bash
   npm run build
   # or
   yarn build
   ```

2. **Start the production server:**
   ```bash
   npm start
   # or
   yarn start
   ```

## 🔧 Running with Services

The UI requires backend services to function properly. Here are the different ways to run the complete system:

### Option 1: Full System with Hardware
```bash
# Terminal 1: Start ROS services
cd /path/to/ros2_ws
source install/setup.bash
ros2 launch control drive_launch.py

# Terminal 2: Start ROS Manager
cd teleop/services
python3 ros/ros_manager.py

# Terminal 3: Start UI
cd teleop/robot-controller-ui
npm run dev
```

### Option 2: Testing with Mock Data
```bash
# Use the automated testing script
cd teleop
./test_demo.sh

# Choose option 3: Full System Demo
# This will start:
# - Mock firmware node
# - ROS Manager
# - Frontend UI
```

### Option 3: Quick Start Script
```bash
# Use the teleop startup script
cd teleop
./start_teleop.sh

# This handles all service startup automatically
```

## 🌐 API Integration

### REST Endpoints

The UI communicates with these backend endpoints:

- **Health Check**: `GET /api/health`
- **Drive Diagnostics**: `GET /api/drive/diagnostics`
- **Drive Speeds**: `GET /api/drive/speeds`
- **Drive Status**: `GET /api/drive/status`
- **Drive Summary**: `GET /api/drive/summary`

### WebSocket Communication

Real-time data is received via WebSocket at `/ws`:

```typescript
// Example WebSocket message structure
{
  "type": "data_update",
  "data": {
    "motors": {
      "RF": {
        "voltage": 12.1,
        "current": 2.3,
        "temperature": 45.2,
        "state": 1
      },
      // ... other motors
    }
  },
  "timestamp": 1625097600000
}
```

## 🎨 UI Design System

### Component Architecture

The UI follows a modular component architecture:

1. **Layout Components**: Handle overall page structure
2. **Section Components**: Major functional areas (drive, arm, camera)
3. **UI Components**: Reusable interface elements
4. **Hook Components**: Custom React hooks for state management

### Styling

- **Tailwind CSS**: Utility-first CSS framework
- **CSS Modules**: Component-scoped styling
- **Responsive Design**: Mobile-first approach
- **Theme Support**: Dark/light mode toggle

### State Management

- **React Hooks**: Built-in state management
- **Custom Hooks**: Specialized state logic
- **Context API**: Global state sharing
- **Local Storage**: Persistent user preferences

## 🔍 Component Details

### Drive Section (`src/components/sections/drive/`)

The drive section is the main interface for motor control and monitoring:

#### DriveInfo Component
- Displays real-time motor diagnostics
- Shows voltage, current, temperature for each motor
- Provides connection status indicators
- Includes error state handling

#### MotorStatus Component
- Visual status indicators for each motor
- Color-coded health status
- Connection state visualization
- Quick diagnostic overview

#### DiagnosticsPanel Component
- Detailed diagnostic information
- Historical data trends
- Performance metrics
- Alert notifications

### WebSocket Hook (`src/hooks/useWebSocket.ts`)

Manages real-time communication:
- Automatic reconnection
- Message queuing
- Connection state management
- Error handling

### API Service (`src/services/api.ts`)

Handles HTTP communication:
- Request/response management
- Error handling
- Data transformation
- Caching strategies

## 🧪 Testing

### Running Tests
```bash
# Run all tests
npm test

# Run tests in watch mode
npm run test:watch

# Run tests with coverage
npm run test:coverage
```

### Test Structure
- **Unit Tests**: Component and utility testing
- **Integration Tests**: API and service testing
- **E2E Tests**: Full user workflow testing

### Mock Data Testing

The UI includes comprehensive mock data support for testing without hardware:

```bash
# Start with mock data
npm run dev:mock

# This enables:
# - Simulated motor data
# - Fake WebSocket messages
# - Test scenarios
```

## 🚀 Deployment

### Docker Deployment
```bash
# Build Docker image
docker build -t rover-ui .

# Run container
docker run -p 3000:3000 rover-ui
```

### Environment Configuration

Different environments require different configurations:

#### Development
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8082
NEXT_PUBLIC_WS_URL=ws://localhost:8082/ws
NEXT_PUBLIC_ENVIRONMENT=development
```

#### Production
```env
NEXT_PUBLIC_API_BASE_URL=https://rover-api.example.com
NEXT_PUBLIC_WS_URL=wss://rover-api.example.com/ws
NEXT_PUBLIC_ENVIRONMENT=production
```

## 🔧 Configuration

### Next.js Configuration (`next.config.js`)
```javascript
/** @type {import('next').NextConfig} */
const nextConfig = {
  experimental: {
    appDir: true,
  },
  env: {
    CUSTOM_KEY: process.env.CUSTOM_KEY,
  },
}

module.exports = nextConfig
```

### Tailwind Configuration (`tailwind.config.js`)
```javascript
module.exports = {
  content: [
    './src/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        rover: {
          primary: '#1e40af',
          secondary: '#64748b',
        }
      }
    },
  },
  plugins: [],
}
```

## 🐛 Troubleshooting

### Common Issues

1. **WebSocket Connection Failed**
   - Check if ROS Manager is running on port 8082
   - Verify firewall settings
   - Check network connectivity

2. **No Data Displayed**
   - Ensure mock firmware or real hardware is running
   - Check ROS2 topic publications
   - Verify API endpoints are accessible

3. **Build Errors**
   - Clear node_modules and reinstall: `rm -rf node_modules && npm install`
   - Check Node.js version compatibility
   - Verify TypeScript configuration

4. **Performance Issues**
   - Check WebSocket message frequency
   - Monitor browser developer tools
   - Verify system resources

### Debug Mode

Enable debug logging:
```bash
DEBUG=rover:* npm run dev
```

### Health Checks

The UI includes built-in health monitoring:
- API connectivity status
- WebSocket connection state
- Data freshness indicators
- Error rate monitoring

## 📚 Additional Resources

- **ROS2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Next.js Documentation**: [https://nextjs.org/docs](https://nextjs.org/docs)
- **Tailwind CSS**: [https://tailwindcss.com/docs](https://tailwindcss.com/docs)
- **TypeScript**: [https://www.typescriptlang.org/docs/](https://www.typescriptlang.org/docs/)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/new-feature`
3. Make your changes
4. Add tests for new functionality
5. Run the test suite: `npm test`
6. Commit your changes: `git commit -am 'Add new feature'`
7. Push to the branch: `git push origin feature/new-feature`
8. Submit a pull request

## 📄 License

This project is part of the McGill Robotics Mars Rover project. See the main repository for license information.

---

For more information about the complete rover system, see the main project documentation in the repository root.
