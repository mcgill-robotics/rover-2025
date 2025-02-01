import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <div>
            <nav style={{display: "flex", justifyContent: "space-around"}}>
                <Link to="/">Home</Link>
                <Link to="/control">Control</Link>
                <Link to="/dashboard">Dashboard</Link>
            </nav>
        </div>
    );
}

export default Navbar;