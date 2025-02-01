// App.tsx
// Main webpage
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Navbar from './components/Navbar'
import Home from './pages/Home';
import Control from './pages/Control';
import Dashboard from './pages/Dashboard';
import './App.css'


function App() {

  return (
    <>
      <Router>
        <Navbar /> 

        <div style={{ display: 'flex' }}>

          <div className="content" style={{ border: '1px solid black', flex: "1" }}>
            <Routes>
              <Route path="/" element={<Home />} />
              <Route path="/control" element={<Control />}/>
              <Route path="/dashboard" element={<Dashboard />}/>
            </Routes>
          </div>
        </div>


      </Router>
    </>
  )
}

export default App
