import { Link } from 'react-router-dom';
import logo from '../assets/logo.png';
import './styles/Navbar.css';

function Navbar() {
    return (
        <div className="nav-container">
            <nav className="navbar">
                <img src={logo} alt="Logo" className="logo" />
                <ul className="nav-links">
                    <li className="option"><Link to="/" title="Rover Status Page">Status</Link></li>
                    <li className="option"><Link to="/control" title="Arms Control">Control</Link></li>
                    <li className="option"><Link to="/dashboard" title="Drive Dashboard">Dashboard</Link></li>
                </ul>
            </nav>
        </div>
    );
}

export default Navbar;