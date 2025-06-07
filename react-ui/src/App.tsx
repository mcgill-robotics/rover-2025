// App.tsx
// Main webpage
// App.tsx
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import { useState } from 'react';
import Navbar from './components/Navbar';
import Status from './pages/Status';
import Control from './pages/Control';
import Dashboard from './pages/Dashboard';
import './App.css';

function App() {
  const [isNavbarVisible, setIsNavbarVisible] = useState(true);

  const toggleNavbar = () => {
    setIsNavbarVisible(prevState => !prevState);
  };

  return (
    <>
      <Router>
        {/* Toggle Button (SVG) */}
        <div
          onClick={toggleNavbar}
          style={{
            position: 'absolute',
            top: '15px',
            right: '10px',
            cursor: 'pointer',
            zIndex: 10
          }}
        >
          {/* Change the SVG based on navbar visibility */}
          {isNavbarVisible ? (
            // Normal L Shape (vertical and horizontal)
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M5 5V19H19" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          ) : (
            // Reversed L Shape (horizontal and vertical)
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M5 5H19V19" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          )}
        </div>

        {/* Navbar visibility */}
        {isNavbarVisible && <Navbar />}

        <div style={{ display: 'flex' }}>
          <div className="content" style={{ flex: "1" }}>
            <Routes>
              <Route path="/" element={<Status />} />
              <Route path="/control" element={<Control />}/>
              <Route path="/dashboard" element={<Dashboard />}/>
            </Routes>
          </div>
        </div>
      </Router>
    </>
  );
}

export default App;
