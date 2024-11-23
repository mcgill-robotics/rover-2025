import './App.css'

import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Sandbox from './pages/Sandbox';
import Home from './pages/Home';
import Header from './components/Header';
import Sidebar from './components/Sidebar';
import DriveControl from './pages/DriveControl';


function App() {

  return (
    <>
      <Router>
        <Header title="Header" width="100%" height="30%" border='2px solid black' />

        <div style={{ display: 'flex' }}>

          <Sidebar width="200px" />
          

          <div className="content" style={{ border: '2px solid black', flex: "1" }}>
            <Routes>
              <Route path="/" element={<Home />} />
              <Route path="/home" element={<Home />} />
              <Route path="/sandbox" element={<Sandbox />} />
              <Route path="/drive" element={<DriveControl />}/>
            </Routes>
          </div>
        </div>


      </Router>
    </>
  )
}

export default App
