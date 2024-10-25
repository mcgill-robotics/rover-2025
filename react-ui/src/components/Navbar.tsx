import { Link } from 'react-router-dom';

function Navbar() {
    return (
        <div>
            <nav style={{display: "flex", justifyContent: "space-around"}}>
                <Link to="/home">Home</Link>
                |
                <Link to="/sandbox">Sandbox</Link>
            </nav>
        </div>
    );
}

export default Navbar;